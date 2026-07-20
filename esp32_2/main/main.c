#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "sensor.h"
#include "reed_switch.h"
#include "esp_log.h"
#include "esp_system.h"

#include <math.h>

static int8_t last_ble_rssi = -127;
static float filtered_ble_rssi = 0.0f;
static bool ble_rssi_filter_initialized = false;
static volatile uint32_t ble_seen_ms = 0;
#define BLE_RSSI_STABILITY_WINDOW 10
static float filtered_rssi_samples[BLE_RSSI_STABILITY_WINDOW] = {0};
static uint8_t filtered_rssi_sample_count = 0;
static uint8_t filtered_rssi_sample_index = 0;
typedef enum { BLE_PROX_UNKNOWN = 0, BLE_PROX_CLOSE, BLE_PROX_FAR } ble_proximity_t;
static ble_proximity_t ble_prox = BLE_PROX_FAR;
static const bool use_specific_beacon = false;

static const uint8_t target_ble_mac[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static bool last_object_close = false;

// BLE proximity logic follows the elevator flowchart:
// - ROBOT_CLOSE when the beacon RSSI crosses the close threshold.
// - ROBOT_FAR only when the beacon RSSI crosses the far threshold.
// - If packets briefly disappear, keep the last confirmed state until an explicit far signal arrives.
// Additional robot or support boards can be added as needed.
static const uint8_t peer_macs[][6] = {
    {0x30, 0x76, 0xF5, 0xF8, 0x4D, 0x7C}, // controller_esp32 board
};
static const char *peer_names[] = {
    "CONTROLLER_ESP32",
};
static const size_t peer_count = sizeof(peer_macs) / sizeof(peer_macs[0]);
static uint8_t own_mac[6] = {0};
static bool last_path_clear = false;
static bool last_door_sealed = false;
static bool ble_prints_enabled = true; // set to true to re-enable BLE prints
static bool enable_ble = true; // BLE must be enabled so beacon proximity can drive the flow

typedef enum {
    ROBOT_FAR,
    ROBOT_CLOSE,
    ROBOT_WAITING_FOR_ELEVATOR,
} robot_proximity_state_t;

static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    printf("[ESP-NOW] send callback invoked, status=%d\n", status);
}

// ESP-NOW commands sent to the controller board.
#define CMD_CALL_ELEVATOR      "CMD_CALL_ELEVATOR"
#define CMD_HOLD_DOOR_OPEN     "CMD_HOLD_DOOR_OPEN"
#define CMD_RELEASE_DOOR       "CMD_RELEASE_DOOR"
#define CMD_SELECT_FLOOR       "CMD_SELECT_FLOOR"
#define CMD_ROBOT_READY        "CMD_ROBOT_READY"

#define FSM_TICK_MS            100
#define BLE_CONFIRM_MS         10000
#define BLE_ENTER_RSSI         -68
#define BLE_EXIT_RSSI          -78
#define DISTANCE_CONFIRM_MS    5000
#define DOOR_CLOSE_TIMEOUT_MS  12000

#define SENSOR_REED_PIN        REED_PIN
#define SENSOR_DISTANCE_PIN    SENSOR_PIN

typedef enum {
    STATE_ROBOT_FAR,
    STATE_ROBOT_CLOSE,
    STATE_WAITING_FOR_ELEVATOR,
    STATE_WAIT_FOR_DOOR,
    STATE_HOLD_DOOR_OPEN,
    STATE_SELECT_FLOOR,
    STATE_ERROR_STUCK,
} elevator_state_t;

static const uint8_t own_mac_prefix[3] = {0x30, 0x76, 0xF5};

static void print_mac(const uint8_t *mac)
{
    printf("%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static bool add_peer(const uint8_t *mac, const char *name)
{
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, mac, ESP_NOW_ETH_ALEN);
    peer_info.channel = 1;
    peer_info.encrypt = false;

    esp_err_t err = esp_now_add_peer(&peer_info);
    if (err != ESP_OK) {
        printf("[ESP-NOW] Failed to add peer %s: ", name);
        print_mac(mac);
        printf(" (err=%d)\n", err);
        return false;
    }

    printf("[ESP-NOW] Added peer %s: ", name);
    print_mac(mac);
    printf("\n");
    return true;
}

static bool send_espnow_command(const char *command)
{
    if (peer_count == 0) {
        printf("[ESP-NOW] No peer configured, cannot send: %s\n", command);
        return false;
    }

    esp_err_t err = esp_now_send(peer_macs[0], (const uint8_t *)command, strlen(command));
    if (err != ESP_OK) {
        printf("[ESP-NOW] Failed to send %s (err=%d)\n", command, err);
        return false;
    }
    printf("[ESP-NOW] Sent: %s -> ", command);
    print_mac(peer_macs[0]);
    printf("\n");
    return true;
}

static bool is_target_ble_address(const uint8_t *addr)
{
    if (!use_specific_beacon) {
        return true;
    }
    return (memcmp(addr, target_ble_mac, sizeof(target_ble_mac)) == 0);
}

static bool is_object_close(void)
{
    return is_robot_detected();
}

static bool is_path_clear(void)
{
    return !is_robot_detected();
}

static bool is_door_sealed(void)
{
    return is_magnet_present();
}

static void process_controller_command(const char *command)
{
    printf("[ESP-NOW] Controller command received: %s\n", command);
}

static void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (len <= 0 || len >= 64) {
        return;
    }

    char incoming[64] = {0};
    snprintf(incoming, sizeof(incoming), "%.*s", len, data);
    printf("[ESP-NOW] RX from %02X:%02X:%02X:%02X:%02X:%02X -> %s\n",
           recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
           recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5],
           incoming);

    process_controller_command(incoming);
}

static bool is_robot_far(void)
{
    if (!enable_ble) {
        return !is_object_close();
    }
    return (ble_prox == BLE_PROX_FAR);
}

static bool is_robot_close(void)
{
    if (!enable_ble) {
        return is_object_close();
    }
    return (ble_prox == BLE_PROX_CLOSE);
}

static void init_sensor_pins(void)
{
    init_proximity_sensor();
    init_reed_switch();
    printf("[GPIO] Sensor modules initialized: Reed=%d, Distance=%d\n", SENSOR_REED_PIN, SENSOR_DISTANCE_PIN);
}

static void ble_scan_start(void)
{
    esp_ble_scan_params_t scan_params = {
        .scan_type = BLE_SCAN_TYPE_PASSIVE,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 0x50,
        .scan_window = 0x30,
        .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
    };

    esp_err_t err = esp_ble_gap_set_scan_params(&scan_params);
    if (err != ESP_OK) {
        printf("[BLE] Failed to set scan params: %d\n", err);
    }
}

static void update_rssi_stability(float rssi, float *mean, float *stddev)
{
    filtered_rssi_samples[filtered_rssi_sample_index] = rssi;
    filtered_rssi_sample_index = (filtered_rssi_sample_index + 1) % BLE_RSSI_STABILITY_WINDOW;
    if (filtered_rssi_sample_count < BLE_RSSI_STABILITY_WINDOW) {
        filtered_rssi_sample_count++;
    }

    *mean = 0.0f;
    for (uint8_t i = 0; i < filtered_rssi_sample_count; ++i) {
        *mean += filtered_rssi_samples[i];
    }
    *mean /= filtered_rssi_sample_count;

    float variance = 0.0f;
    for (uint8_t i = 0; i < filtered_rssi_sample_count; ++i) {
        float difference = filtered_rssi_samples[i] - *mean;
        variance += difference * difference;
    }
    *stddev = sqrtf(variance / filtered_rssi_sample_count);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            if (param->scan_param_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                esp_err_t err = esp_ble_gap_start_scanning(0);
                if (err != ESP_OK) {
                    printf("[BLE] Failed to start scanning: %d\n", err);
                }
            } else {
                printf("[BLE] Scan parameter set failed, status=%d\n", param->scan_param_cmpl.status);
            }
            break;

        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
                if (is_target_ble_address(param->scan_rst.bda)) {
                    last_ble_rssi = param->scan_rst.rssi;
                    int8_t current_rssi = last_ble_rssi;
                    if (!ble_rssi_filter_initialized) {
                        filtered_ble_rssi = current_rssi;
                        ble_rssi_filter_initialized = true;
                    } else {
                        filtered_ble_rssi = (0.85f * filtered_ble_rssi) + (0.15f * current_rssi);
                    }

                    float mean_rssi;
                    float stddev_rssi;
                    update_rssi_stability(filtered_ble_rssi, &mean_rssi, &stddev_rssi);
                    // Use a two-threshold approach so the flow only enters ROBOT_CLOSE when the
                    // beacon is clearly near, and only returns to ROBOT_FAR on an explicit far signal.
                    if (stddev_rssi < 2.0f && filtered_ble_rssi >= BLE_ENTER_RSSI) {
                        if (ble_prox != BLE_PROX_CLOSE) {
                            ble_prox = BLE_PROX_CLOSE;
                            if (ble_prints_enabled) printf("[BLE] Prox => CLOSE (filtered_rssi=%.1f)\n", filtered_ble_rssi);
                        }
                    } else if (stddev_rssi < 2.0f && filtered_ble_rssi <= BLE_EXIT_RSSI) {
                        if (ble_prox != BLE_PROX_FAR) {
                            ble_prox = BLE_PROX_FAR;
                            if (ble_prints_enabled) printf("[BLE] Prox => FAR (filtered_rssi=%.1f)\n", filtered_ble_rssi);
                        }
                    }

                    if (ble_prints_enabled) {
                        const char *ble_state = ble_prox == BLE_PROX_CLOSE ? "CLOSE" :
                                                ble_prox == BLE_PROX_FAR ? "FAR" : "UNKNOWN";
                        printf("[BLE] ADV MAC=");
                        print_mac(param->scan_rst.bda);
                        printf(" RSSI raw=%d filtered=%.1f Mean RSSI=%.1f StdDev RSSI=%.2f state=%s confirm_ms=%lu\n",
                               current_rssi, filtered_ble_rssi, mean_rssi, stddev_rssi,
                               ble_state, (unsigned long)ble_seen_ms);
                    }
                }
            }
            break;

        default:
            break;
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Suppress verbose BLE/BT logs unless explicitly enabled */
    if (!ble_prints_enabled) {
        esp_log_level_set("*", ESP_LOG_ERROR);
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));

    ESP_ERROR_CHECK(esp_read_mac(own_mac, ESP_MAC_WIFI_STA));
    printf("[ESP-NOW] Robot local MAC: ");
    print_mac(own_mac);
    printf("\n");
    
    for (size_t i = 0; i < peer_count; ++i) {
        if (!add_peer(peer_macs[i], peer_names[i])) {
            printf("[ESP-NOW] Warning: peer %s may not be reachable.\n", peer_names[i]);
        }
    }

    // Send a startup handshake so the controller knows ESP-NOW is ready.
    send_espnow_command("CMD_ROBOT_READY");

    init_sensor_pins();
    {
        int reed_level = gpio_get_level(SENSOR_REED_PIN);
        int dist_level = gpio_get_level(SENSOR_DISTANCE_PIN);
        printf("[SENSOR_INIT] Reed raw=%d, Distance raw=%d\n", reed_level, dist_level);
    }

    // Initialize BLE stack for beacon scanning (disabled by default)
    if (enable_ble) {
        ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        bt_cfg.mode = ESP_BT_MODE_BLE;
        ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
        ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
        ESP_ERROR_CHECK(esp_bluedroid_init());
        ESP_ERROR_CHECK(esp_bluedroid_enable());
        ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));

        ble_scan_start();
    } else {
        printf("[INFO] BLE disabled; only sensor outputs will be shown.\n");
    }
    printf("[FLOW] esp32_2 initialized. Starting BLE+sensor elevator flow.\n");

    elevator_state_t state = STATE_ROBOT_FAR;
    elevator_state_t last_state = STATE_ROBOT_FAR;
    uint32_t distance_confirm_ms = 0;
    uint32_t door_close_wait_ms = 0;
    bool release_sent = false;

    printf("[FLOW] State changed: ROBOT_FAR\n");

    while (1) {
        bool object_close = is_object_close();
        bool door_sealed = is_door_sealed();
        bool path_clear = is_path_clear();

        if (state != last_state) {
            const char *name = "UNKNOWN";
            switch (state) {
                case STATE_ROBOT_FAR: name = "ROBOT_FAR"; break;
                case STATE_ROBOT_CLOSE: name = "ROBOT_CLOSE"; break;
                case STATE_WAITING_FOR_ELEVATOR: name = "WAITING_FOR_ELEVATOR"; break;
                case STATE_WAIT_FOR_DOOR: name = "WAIT_FOR_DOOR"; break;
                case STATE_HOLD_DOOR_OPEN: name = "HOLD_DOOR_OPEN"; break;
                case STATE_SELECT_FLOOR: name = "SELECT_FLOOR"; break;
                case STATE_ERROR_STUCK: name = "ERROR_STUCK"; break;
            }
            printf("[FLOW] State changed: %s\n", name);
            last_state = state;
        }

        if (object_close != last_object_close) {
            if (object_close) {
                printf("[SENSOR] Distance detected: object_close=1\n");
            } else {
                printf("[SENSOR] Distance cleared: object_close=0\n");
            }
            last_object_close = object_close;
        }
        if (path_clear != last_path_clear) {
            if (path_clear) {
                printf("[SENSOR] Path clear: path_clear=1\n");
            } else {
                printf("[SENSOR] Path blocked: path_clear=0\n");
            }
            last_path_clear = path_clear;
        }
        if (door_sealed != last_door_sealed) {
            if (door_sealed) {
                printf("[SENSOR] Reed activated: door_sealed=1\n");
            } else {
                printf("[SENSOR] Reed opened: door_sealed=0\n");
            }
            last_door_sealed = door_sealed;
        }

        switch (state) {
            case STATE_ROBOT_FAR:
                if (is_robot_close()) {
                    printf("[FLOW] Robot BLE detected nearby. Entering close state.\n");
                    state = STATE_ROBOT_CLOSE;
                    ble_seen_ms = 0;
                }
                break;

            case STATE_ROBOT_CLOSE:
                if (is_robot_far()) {
                    printf("[FLOW] Robot BLE lost. Returning to far state.\n");
                    state = STATE_ROBOT_FAR;
                    ble_seen_ms = 0;
                    break;
                }
                if (object_close) {
                    ble_seen_ms += FSM_TICK_MS;
                    if (ble_seen_ms >= BLE_CONFIRM_MS) {
                        printf("[FLOW] BLE close and distance confirmed for 10s. Entering waiting-for-elevator state.\n");
                        state = STATE_WAITING_FOR_ELEVATOR;
                        distance_confirm_ms = 0;
                    }
                } else {
                    ble_seen_ms = 0;
                }
                break;

            case STATE_WAITING_FOR_ELEVATOR:
                if (is_robot_far()) {
                    printf("[FLOW] BLE robot no longer close. Returning to far state.\n");
                    state = STATE_ROBOT_FAR;
                    ble_seen_ms = 0;
                    break;
                }
                if (object_close) {
                    distance_confirm_ms += FSM_TICK_MS;
                    if (distance_confirm_ms >= DISTANCE_CONFIRM_MS) {
                        printf("[FLOW] Robot presence confirmed while BLE waiting. Calling elevator.\n");
                        send_espnow_command(CMD_CALL_ELEVATOR);
                        state = STATE_WAIT_FOR_DOOR;
                    }
                } else {
                    distance_confirm_ms = 0;
                }
                break;

            case STATE_WAIT_FOR_DOOR:
                if (!door_sealed) {
                    printf("[FLOW] Door opened detected by robot. Sending HOLD DOOR OPEN command.\n");
                    send_espnow_command(CMD_HOLD_DOOR_OPEN);
                    state = STATE_HOLD_DOOR_OPEN;
                    release_sent = false;
                }
                break;

            case STATE_HOLD_DOOR_OPEN:
                if (path_clear) {
                    printf("[FLOW] Path clear confirmed. Sending RELEASE DOOR and moving to SELECT_FLOOR.\n");
                    send_espnow_command(CMD_RELEASE_DOOR);
                    state = STATE_SELECT_FLOOR;
                    door_close_wait_ms = 0;
                    release_sent = true;
                }
                break;

            case STATE_SELECT_FLOOR:
                if (door_sealed) {
                    printf("[FLOW] Door resealed on robot side. Sending SELECT FLOOR and resetting flow.\n");
                    send_espnow_command(CMD_SELECT_FLOOR);
                    state = STATE_ROBOT_FAR;
                    distance_confirm_ms = 0;
                    ble_seen_ms = 0;
                    door_close_wait_ms = 0;
                } else {
                    door_close_wait_ms += FSM_TICK_MS;
                    if (door_close_wait_ms >= DOOR_CLOSE_TIMEOUT_MS) {
                        printf("[ERROR] Door failed to reseal within 12s. Entering ERROR_STUCK.\n");
                        state = STATE_ERROR_STUCK;
                    }
                }
                break;

            case STATE_ERROR_STUCK:
                if (!release_sent) {
                    printf("[FSM] ERROR_STUCK: sending CMD_RELEASE_DOOR for safety.\n");
                    send_espnow_command(CMD_RELEASE_DOOR);
                    release_sent = true;
                }
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(FSM_TICK_MS));
    }
}

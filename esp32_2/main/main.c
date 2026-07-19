#include <stdio.h>
#include <string.h>
#include <stdbool.h>
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

static TickType_t ble_last_seen_ticks = 0;
static const bool use_specific_beacon = false;
static const uint8_t target_ble_mac[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static bool last_object_close = false;
static bool last_path_clear = false;
static bool last_door_sealed = false;
static bool ble_prints_enabled = false; // set to true to re-enable BLE prints
static bool enable_ble = true; // enabled now so BLE robot location is used

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

#define CMD_CALL_ELEVATOR      "CMD_CALL_ELEVATOR"
#define CMD_HOLD_DOOR_OPEN     "CMD_HOLD_DOOR_OPEN"
#define CMD_RELEASE_DOOR       "CMD_RELEASE_DOOR"
#define CMD_SELECT_FLOOR       "CMD_SELECT_FLOOR"

#define FSM_TICK_MS            100
#define BLE_CONFIRM_MS         10000
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

static const uint8_t controller_mac[6] = {0x30, 0x76, 0xF5, 0xF8, 0x4D, 0x7C};

static bool send_espnow_command(const char *command)
{
    esp_err_t err = esp_now_send(controller_mac, (const uint8_t *)command, strlen(command));
    if (err != ESP_OK) {
        printf("[ESP-NOW] Failed to send %s (err=%d)\n", command, err);
        return false;
    }
    printf("[ESP-NOW] Sent: %s\n", command);
    return true;
}

static bool is_ble_target_visible(void)
{
    if (ble_last_seen_ticks == 0) {
        return false;
    }

    TickType_t now = xTaskGetTickCount();
    if ((now - ble_last_seen_ticks) <= pdMS_TO_TICKS(500)) {
        return true;
    }
    return false;
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

static bool is_robot_far(void)
{
    return !is_ble_target_visible();
}

static bool is_robot_close(void)
{
    return is_ble_target_visible();
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

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
                if (is_target_ble_address(param->scan_rst.bda) && param->scan_rst.rssi > -80) {
                    ble_last_seen_ticks = xTaskGetTickCount();
                    if (ble_prints_enabled) {
                        printf("[BLE] Beacon rssi=%d\n", param->scan_rst.rssi);
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

    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, controller_mac, sizeof(controller_mac));
    peer_info.channel = 1;
    peer_info.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

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
    uint32_t ble_seen_ms = 0;
    uint32_t distance_confirm_ms = 0;
    uint32_t door_close_wait_ms = 0;
    bool release_sent = false;

    while (1) {
        bool target_seen = false;
        if (enable_ble) {
            target_seen = is_ble_target_visible();
        }
        bool object_close = is_object_close();
        bool door_sealed = is_door_sealed();
        bool path_clear = is_path_clear();

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
                if (target_seen) {
                    ble_seen_ms += FSM_TICK_MS;
                    if (ble_seen_ms >= BLE_CONFIRM_MS) {
                        printf("[FLOW] Robot is close via BLE for 10s. Moving to waiting-for-elevator state.\n");
                        state = STATE_WAITING_FOR_ELEVATOR;
                        distance_confirm_ms = 0;
                    }
                } else {
                    if (ble_seen_ms > 0) {
                        printf("[FLOW] BLE lost before close confirmation. Robot is still far.\n");
                    }
                    ble_seen_ms = 0;
                }
                break;

            case STATE_WAITING_FOR_ELEVATOR:
                if (!target_seen) {
                    printf("[FLOW] BLE robot no longer close. Returning to far detection.\n");
                    state = STATE_ROBOT_FAR;
                    ble_seen_ms = 0;
                    break;
                }
                if (object_close) {
                    distance_confirm_ms += FSM_TICK_MS;
                    if (distance_confirm_ms >= DISTANCE_CONFIRM_MS) {
                        printf("[FLOW] Robot presence confirmed while BLE close. Calling elevator.\n");
                        send_espnow_command(CMD_CALL_ELEVATOR);
                        state = STATE_WAIT_FOR_DOOR;
                    }
                } else {
                    distance_confirm_ms = 0;
                }
                break;

            case STATE_WAIT_FOR_DOOR:
                if (!door_sealed) {
                    printf("[FLOW] Door opened by controller. Sending HOLD DOOR OPEN command.\n");
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
                    printf("[FLOW] Door sealed after release. Sending SELECT FLOOR and resetting flow.\n");
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

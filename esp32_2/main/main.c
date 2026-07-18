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
#include "sensor.h"
#include "reed_switch.h"

static bool ble_target_found = false;

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
    STATE_SEARCHING_BLE,
    STATE_VERIFY_DISTANCE,
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
    // BLE scan runs continuously; check if target was detected in recent scan
    if (ble_target_found) {
        ble_target_found = false;  // Reset flag for next scan
        return true;
    }
    return false;
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

static void init_sensor_pins(void)
{
    init_proximity_sensor();
    init_reed_switch();
    printf("[GPIO] Sensor modules initialized: Reed=%d, Distance=%d\n", SENSOR_REED_PIN, SENSOR_DISTANCE_PIN);
}

static void ble_scan_start(void)
{
    // BLE controller is enabled, scanning can run
    // Note: Full GAP stack scan not configured; placeholder for BLE initialization
    printf("[BLE] BLE controller initialized and ready for scanning\n");
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
    
    // Initialize BLE (minimal configuration)
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    
    // Start initial BLE scan
    ble_scan_start();
    printf("[FSM] Board 2 started. Waiting for BLE verification and sensors.\n");

    elevator_state_t state = STATE_SEARCHING_BLE;
    uint32_t ble_seen_ms = 0;
    uint32_t distance_confirm_ms = 0;
    uint32_t door_close_wait_ms = 0;
    uint32_t ble_scan_restart_ms = 0;
    bool release_sent = false;

    while (1) {
        // Periodically restart BLE scan every 6 seconds
        ble_scan_restart_ms += FSM_TICK_MS;
        if (ble_scan_restart_ms >= 6000) {
            ble_scan_start();
            ble_scan_restart_ms = 0;
        }
        
        bool target_seen = is_ble_target_visible();
        bool object_close = is_object_close();
        bool door_sealed = is_door_sealed();
        bool path_clear = is_path_clear();

        switch (state) {
            case STATE_SEARCHING_BLE:
                if (target_seen) {
                    ble_seen_ms += FSM_TICK_MS;
                    if (ble_seen_ms >= BLE_CONFIRM_MS) {
                        printf("[FSM] BLE target continuously seen for 10s. Transition to VERIFY_DISTANCE.\n");
                        state = STATE_VERIFY_DISTANCE;
                        distance_confirm_ms = 0;
                    }
                } else {
                    if (ble_seen_ms > 0) {
                        printf("[FSM] BLE target lost before confirmation. Resetting search.\n");
                    }
                    ble_seen_ms = 0;
                }
                break;

            case STATE_VERIFY_DISTANCE:
                if (object_close) {
                    distance_confirm_ms += FSM_TICK_MS;
                    if (distance_confirm_ms >= DISTANCE_CONFIRM_MS) {
                        printf("[FSM] Distance verified for 5s. Sending CMD_CALL_ELEVATOR.\n");
                        send_espnow_command(CMD_CALL_ELEVATOR);
                        state = STATE_WAIT_FOR_DOOR;
                    }
                } else {
                    printf("[FSM] Object lost during distance verification. Returning to SEARCHING_BLE.\n");
                    state = STATE_SEARCHING_BLE;
                    ble_seen_ms = 0;
                    distance_confirm_ms = 0;
                }
                break;

            case STATE_WAIT_FOR_DOOR:
                if (!door_sealed) {
                    printf("[FSM] Door opened. Sending CMD_HOLD_DOOR_OPEN.\n");
                    send_espnow_command(CMD_HOLD_DOOR_OPEN);
                    state = STATE_HOLD_DOOR_OPEN;
                    release_sent = false;
                }
                break;

            case STATE_HOLD_DOOR_OPEN:
                if (path_clear) {
                    printf("[FSM] Path clear detected. Sending CMD_RELEASE_DOOR and moving to SELECT_FLOOR.\n");
                    send_espnow_command(CMD_RELEASE_DOOR);
                    state = STATE_SELECT_FLOOR;
                    door_close_wait_ms = 0;
                    release_sent = true;
                }
                break;

            case STATE_SELECT_FLOOR:
                if (door_sealed) {
                    printf("[FSM] Door sealed safely. Sending CMD_SELECT_FLOOR and resetting FSM.\n");
                    send_espnow_command(CMD_SELECT_FLOOR);
                    state = STATE_SEARCHING_BLE;
                    ble_seen_ms = 0;
                    distance_confirm_ms = 0;
                    door_close_wait_ms = 0;
                } else {
                    door_close_wait_ms += FSM_TICK_MS;
                    if (door_close_wait_ms >= DOOR_CLOSE_TIMEOUT_MS) {
                        printf("[FSM] Door did not seal within 12s. Transition to ERROR_STUCK.\n");
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

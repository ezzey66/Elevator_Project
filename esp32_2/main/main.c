#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

// Your exact target MAC address printed on the Teltonika housing
static uint8_t TARGET_ROBOT_BEACON[] = {0x7C, 0xD9, 0xF4, 0x08, 0xD5, 0x85};
static const char *TAG = "BLE_SCANNER";

// Tune this with a real measurement: RSSI seen at 1 meter from the beacon.
#define RSSI_AT_ONE_METER_DBM (-59)
#define PATH_LOSS_EXPONENT    (2.0f)
#define RSSI_SMOOTHING_ALPHA  (0.20f)

#define NEAR_RSSI_THRESHOLD_DBM (-68.0f)
#define AWAY_RSSI_THRESHOLD_DBM (-78.0f)
#define NEAR_CONFIRM_COUNT      (3)
#define AWAY_CONFIRM_COUNT      (5)

static bool rssi_filter_ready = false;
static float smoothed_rssi = 0.0f;
static bool robot_confirmed_near = false;
static int near_count = 0;
static int away_count = 0;

static bool check_step(esp_err_t err, const char *step) {
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s failed: %s", step, esp_err_to_name(err));
        return false;
    }
    return true;
}

static float estimate_distance_meters(int rssi) {
    return powf(10.0f, ((float)RSSI_AT_ONE_METER_DBM - (float)rssi) / (10.0f * PATH_LOSS_EXPONENT));
}

static float update_smoothed_rssi(int raw_rssi) {
    if (!rssi_filter_ready) {
        smoothed_rssi = (float)raw_rssi;
        rssi_filter_ready = true;
    } else {
        smoothed_rssi = (RSSI_SMOOTHING_ALPHA * (float)raw_rssi)
                      + ((1.0f - RSSI_SMOOTHING_ALPHA) * smoothed_rssi);
    }

    return smoothed_rssi;
}

static void update_robot_proximity_state(float filtered_rssi) {
    if (filtered_rssi >= NEAR_RSSI_THRESHOLD_DBM) {
        near_count++;
        away_count = 0;
    } else if (filtered_rssi <= AWAY_RSSI_THRESHOLD_DBM) {
        away_count++;
        near_count = 0;
    } else {
        near_count = 0;
        away_count = 0;
    }

    if (!robot_confirmed_near && near_count >= NEAR_CONFIRM_COUNT) {
        robot_confirmed_near = true;
        printf(">>> SECURITY STATUS: ROBOT_CONFIRMED_NEAR_ELEVATOR\n");
    } else if (robot_confirmed_near && away_count >= AWAY_CONFIRM_COUNT) {
        robot_confirmed_near = false;
        printf(">>> SECURITY STATUS: ROBOT_LEFT_ELEVATOR_AREA\n");
    }
}

// BLE Scanning configuration parameters
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50, // 50ms window interval
    .scan_window            = 0x30  // 30ms active reception
};

// Callback triggered whenever a nearby Bluetooth device transmits a packet
static void esp_ble_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            // Parameters validated, begin continuous tracking
            check_step(esp_ble_gap_start_scanning(0), "esp_ble_gap_start_scanning");
            break;

        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            esp_ble_gap_cb_param_t *scan_result = param;
            
            if (scan_result->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
                // Check if the discovered device's MAC address matches your Teltonika module
                if (memcmp(scan_result->scan_rst.bda, TARGET_ROBOT_BEACON, 6) == 0) {
                    
                    int rssi = scan_result->scan_rst.rssi;
                    float filtered_rssi = update_smoothed_rssi(rssi);
                    float distance_m = estimate_distance_meters((int)filtered_rssi);
                    printf("[SECURITY MATCH] Robot Identity Confirmed!\n");
                    printf("Target MAC found: 7C:D9:F4:08:D5:85 | Raw RSSI: %d dBm | Smoothed RSSI: %.1f dBm\n",
                           rssi, filtered_rssi);
                    printf("Estimated Distance: %.2f meters\n", distance_m);
                    update_robot_proximity_state(filtered_rssi);
                    
                    // Proximity Analysis 
                    if (robot_confirmed_near) {
                        printf(">>> STATUS: Robot is safely inside the immediate loading zone (Very Close).\n");
                    } else if (filtered_rssi >= NEAR_RSSI_THRESHOLD_DBM) {
                        printf(">>> STATUS: Robot is near elevator, waiting for confirmation stability.\n");
                    } else {
                        printf(">>> STATUS: Robot detected but still approaching down the corridor.\n");
                    }
                    printf("-----------------------------------------------------------------\n");
                }
            }
            break;
        }
        default:
            break;
    }
}

void app_main(void) {
    // 1. Initialize Storage Controller
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        if (!check_step(nvs_flash_erase(), "nvs_flash_erase")) {
            return;
        }
        ret = nvs_flash_init();
    }
    if (!check_step(ret, "nvs_flash_init")) {
        return;
    }

    // 2. Release and initialize core hardware memory controllers
    if (!check_step(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT), "esp_bt_controller_mem_release")) {
        return;
    }
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    bt_cfg.mode = ESP_BT_MODE_BLE;
    if (!check_step(esp_bt_controller_init(&bt_cfg), "esp_bt_controller_init")) {
        return;
    }
    if (!check_step(esp_bt_controller_enable(ESP_BT_MODE_BLE), "esp_bt_controller_enable")) {
        return;
    }

    // 3. Initialize the Bluedroid software stack layer
    if (!check_step(esp_bluedroid_init(), "esp_bluedroid_init")) {
        return;
    }
    if (!check_step(esp_bluedroid_enable(), "esp_bluedroid_enable")) {
        return;
    }

    // 4. Register our active tracking callback loop
    if (!check_step(esp_ble_gap_register_callback(esp_ble_gap_cb), "esp_ble_gap_register_callback")) {
        return;
    }

    printf("[BLE SCANNER] Initialized. Scanning for Teltonika Beacon ID: 7C:D9:F4:08:D5:85...\n");
    
    // Set parameters which triggers the sequence engine callback
    if (!check_step(esp_ble_gap_set_scan_params(&ble_scan_params), "esp_ble_gap_set_scan_params")) {
        return;
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

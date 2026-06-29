#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_event.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

static const char *TAG = "BLE_SCANNER";

static const uint8_t TARGET_BDA[ESP_BD_ADDR_LEN] = {
    0x7C, 0xD9, 0xF4, 0x08, 0xD5, 0x85
};

#define RSSI_AT_ONE_METER_DBM (-59)
#define PATH_LOSS_EXPONENT    (2.0f)
#define RSSI_SMOOTHING_ALPHA  (0.50f)  // Very aggressive smoothing for fast response

#define NEAR_RSSI_THRESHOLD_DBM (-60.0f)  // Tightened from -62.0 for closer immediate zone
#define AWAY_RSSI_THRESHOLD_DBM (-75.0f)  // Tightened from -80.0 for better discrimination (15dBm gap)
#define NEAR_CONFIRM_COUNT      (1)       // Reduced from 2 - single packet confirms NEAR
#define AWAY_CONFIRM_COUNT      (2)       // Reduced from 3 for faster away detection
#define SIGNAL_STALE_MS         (3500)    // Reduced from 5000 - more responsive to packet loss
#define SIGNAL_LOST_MS          (7000)    // Reduced from 10000 - match max observed gap

#define DASHBOARD_AP_SSID       "ElevatorBLE-Test"
#define DASHBOARD_AP_PASSWORD   "elevator123"
#define DASHBOARD_AP_CHANNEL    (6)

typedef struct {
    bool seen;
    bool confirmed_near;
    int raw_rssi;
    float smoothed_rssi;
    float estimated_distance_m;
    int64_t last_seen_ms;
} dashboard_data_t;

static SemaphoreHandle_t dashboard_mutex;
static dashboard_data_t dashboard_data;

static bool rssi_filter_ready;
static float smoothed_rssi;
static bool robot_confirmed_near;
static int near_count;
static int away_count;
static int64_t last_rssi_update_ms;  // Track when last packet was received

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x10,  // 20ms interval - very aggressive scanning
    .scan_window = 0x10,    // 20ms window - continuous scanning for reliable packet capture
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
};

static void check_step(const char *step, esp_err_t err)
{
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s failed: %s", step, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "%s ok", step);
    }
}

static float estimate_distance_meters(float rssi)
{
    return powf(10.0f, ((float)RSSI_AT_ONE_METER_DBM - rssi) / (10.0f * PATH_LOSS_EXPONENT));
}

static const char *proximity_status_text(bool near)
{
    return near ? "NEAR ELEVATOR" : "NOT NEAR";
}

static const char *signal_zone_text(float rssi)
{
    if (rssi >= NEAR_RSSI_THRESHOLD_DBM) {
        return "Immediate elevator zone";
    }
    if (rssi <= AWAY_RSSI_THRESHOLD_DBM) {
        return "Outside elevator zone";
    }
    return "Approaching elevator zone";
}

static const char *distance_range_text(float rssi)
{
    if (rssi >= -62.0f) {
        return "very close";
    }
    if (rssi >= NEAR_RSSI_THRESHOLD_DBM) {
        return "close";
    }
    if (rssi >= AWAY_RSSI_THRESHOLD_DBM) {
        return "nearby";
    }
    return "far / weak signal";
}

static float update_smoothed_rssi(int raw_rssi)
{
    if (!rssi_filter_ready) {
        smoothed_rssi = (float)raw_rssi;
        rssi_filter_ready = true;
        last_rssi_update_ms = esp_timer_get_time() / 1000;
        return smoothed_rssi;
    }

    smoothed_rssi = (RSSI_SMOOTHING_ALPHA * (float)raw_rssi) +
                    ((1.0f - RSSI_SMOOTHING_ALPHA) * smoothed_rssi);
    last_rssi_update_ms = esp_timer_get_time() / 1000;
    return smoothed_rssi;
}

// Apply exponential decay to RSSI during gaps between packets
static float apply_rssi_decay(void)
{
    if (!rssi_filter_ready) {
        return smoothed_rssi;
    }
    
    int64_t now_ms = esp_timer_get_time() / 1000;
    int64_t time_since_last_ms = now_ms - last_rssi_update_ms;
    
    if (time_since_last_ms <= 500) {
        // Recent packet, no decay
        return smoothed_rssi;
    }
    
    // Apply exponential decay: weaken signal gradually
    // Decay rate: -0.5 dBm per 500ms, accelerating after 1 second
    float decay = 0.0f;
    if (time_since_last_ms < 1000) {
        decay = (float)(time_since_last_ms - 500) / 1000.0f;  // 0 to 0.5 dBm
    } else if (time_since_last_ms < 2000) {
        decay = 0.5f + (float)(time_since_last_ms - 1000) / 1000.0f;  // 0.5 to 1.5 dBm
    } else {
        decay = 1.5f + (float)(time_since_last_ms - 2000) / 500.0f;  // Accelerating decay
    }
    
    return smoothed_rssi - decay;
}

static void update_robot_proximity_state(float filtered_rssi)
{
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
        ESP_LOGI(TAG, "Robot proximity confirmed: %s", proximity_status_text(robot_confirmed_near));
    } else if (robot_confirmed_near && away_count >= AWAY_CONFIRM_COUNT) {
        robot_confirmed_near = false;
        ESP_LOGI(TAG, "Robot proximity confirmed: %s", proximity_status_text(robot_confirmed_near));
    }
}

static void update_dashboard_data(int raw_rssi, float filtered_rssi)
{
    if (dashboard_mutex == NULL) {
        return;
    }

    if (xSemaphoreTake(dashboard_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        dashboard_data.seen = true;
        dashboard_data.confirmed_near = robot_confirmed_near;
        dashboard_data.raw_rssi = raw_rssi;
        dashboard_data.smoothed_rssi = filtered_rssi;
        dashboard_data.estimated_distance_m = estimate_distance_meters(filtered_rssi);
        dashboard_data.last_seen_ms = esp_timer_get_time() / 1000;
        xSemaphoreGive(dashboard_mutex);
    }
}

static esp_err_t dashboard_page_handler(httpd_req_t *req)
{
    static const char page[] =
        "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Elevator BLE Test</title><style>"
        "body{font-family:Arial,sans-serif;background:#101216;color:#f6f7f9;margin:0;padding:20px}"
        ".wrap{max-width:560px;margin:auto}.top{display:flex;justify-content:space-between;align-items:flex-end;gap:12px}"
        "h1{font-size:22px;margin:0 0 4px}.muted{color:#9da6b3;font-size:13px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:14px}"
        ".panel{background:#20242b;border:1px solid #303743;border-radius:8px;padding:16px}.wide{grid-column:1/-1}"
        ".label{color:#aeb7c4;font-size:13px;margin-bottom:8px}.value{font-size:28px;font-weight:700;line-height:1.1}.small{font-size:15px;color:#d6dbe3;line-height:1.8}"
        ".near{color:#45d483}.warn{color:#ffc15c}.away{color:#ff7a70}.bar{height:10px;background:#343b47;border-radius:999px;overflow:hidden;margin-top:12px}"
        ".fill{height:100%;width:0;background:#45d483;transition:width .25s}.mono{font-family:Consolas,monospace}"
        "@media(max-width:520px){.grid{grid-template-columns:1fr}.wide{grid-column:auto}.value{font-size:24px}}"
        "</style></head><body><div class='wrap'><div class='top'><div><h1>Elevator BLE Test</h1>"
        "<div class='muted'>Robot beacon: 7C:D9:F4:08:D5:85</div></div><div id='seen' class='muted'>waiting</div></div>"
        "<div class='grid'><div class='panel wide'><div class='label'>Elevator decision</div><div id='status' class='value away'>Waiting...</div>"
        "<div class='bar'><div id='strength' class='fill'></div></div></div>"
        "<div class='panel'><div class='label'>Signal zone</div><div id='zone' class='value'>--</div></div>"
        "<div class='panel'><div class='label'>Rough range</div><div id='range' class='value'>--</div></div>"
        "<div class='panel wide small'>Raw RSSI: <b id='raw' class='mono'>--</b> dBm<br>"
        "Smoothed RSSI: <b id='smooth' class='mono'>--</b> dBm<br>"
        "Formula estimate: <b id='distance' class='mono'>--</b> m<br>"
        "Last BLE packet: <b id='packet' class='mono'>--</b></div></div></div><script>"
        "async function tick(){try{const r=await fetch('/data',{cache:'no-store'});const d=await r.json();"
        "const active=d.seen&&d.fresh;const stale=d.seen&&!d.fresh;"
        "const s=document.getElementById('status');s.textContent=active?(d.near?'CONFIRMED NEAR ELEVATOR':'NOT CONFIRMED NEAR'):(stale?'SIGNAL STALE':'Waiting...');"
        "s.className='value '+(active&&d.near?'near':(active?'warn':'away'));document.getElementById('zone').textContent=active?d.zone:'--';"
        "document.getElementById('range').textContent=d.seen?d.range:'--';document.getElementById('distance').textContent=d.seen?d.distance_m.toFixed(2):'--';"
        "document.getElementById('raw').textContent=d.seen?d.raw_rssi:'--';document.getElementById('smooth').textContent=d.seen?d.smoothed_rssi.toFixed(1):'--';"
        "document.getElementById('seen').textContent=active?'live':(stale?'stale':'waiting');"
        "document.getElementById('packet').textContent=d.seen?d.last_seen_age_ms+' ms ago':'waiting';"
        "const pct=active?Math.max(0,Math.min(100,(d.smoothed_rssi+90)*3.2)):0;document.getElementById('strength').style.width=pct+'%';}catch(e){}}"
        "setInterval(tick,500);tick();</script></body></html>";

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t dashboard_data_handler(httpd_req_t *req)
{
    dashboard_data_t snapshot = {0};
    int64_t now_ms = esp_timer_get_time() / 1000;

    if (dashboard_mutex != NULL && xSemaphoreTake(dashboard_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        snapshot = dashboard_data;
        xSemaphoreGive(dashboard_mutex);
    }

    int64_t age_ms = snapshot.seen ? (now_ms - snapshot.last_seen_ms) : -1;
    bool fresh = snapshot.seen && age_ms <= SIGNAL_STALE_MS;
    
    // Apply RSSI decay for display - smoothly transition during gaps
    float decayed_rssi = snapshot.smoothed_rssi;
    if (snapshot.seen && age_ms > 500) {
        float decay = 0.0f;
        if (age_ms < 1000) {
            decay = (float)(age_ms - 500) / 1000.0f;
        } else if (age_ms < 2000) {
            decay = 0.5f + (float)(age_ms - 1000) / 1000.0f;
        } else {
            decay = 1.5f + (float)(age_ms - 2000) / 500.0f;
        }
        decayed_rssi = snapshot.smoothed_rssi - decay;
    }
    
    bool near = fresh && snapshot.confirmed_near;
    const char *zone = fresh ? signal_zone_text(decayed_rssi) : "Signal stale";

    char json[384];
    int len = snprintf(json, sizeof(json),
                       "{\"seen\":%s,\"fresh\":%s,\"near\":%s,\"raw_rssi\":%d,"
                       "\"smoothed_rssi\":%.1f,\"distance_m\":%.2f,"
                       "\"zone\":\"%s\",\"range\":\"%s\","
                       "\"last_seen_age_ms\":%lld}",
                       snapshot.seen ? "true" : "false",
                       fresh ? "true" : "false",
                       near ? "true" : "false",
                       snapshot.raw_rssi,
                       decayed_rssi,
                       estimate_distance_meters(decayed_rssi),
                       zone,
                       distance_range_text(decayed_rssi),
                       (long long)age_ms);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, len);
}

static void start_dashboard_server(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));

    wifi_config_t ap_config = {
        .ap = {
            .ssid = DASHBOARD_AP_SSID,
            .ssid_len = strlen(DASHBOARD_AP_SSID),
            .password = DASHBOARD_AP_PASSWORD,
            .channel = DASHBOARD_AP_CHANNEL,
            .max_connection = 2,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(httpd_start(&server, &server_config));

    httpd_uri_t page_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = dashboard_page_handler,
    };
    httpd_uri_t data_uri = {
        .uri = "/data",
        .method = HTTP_GET,
        .handler = dashboard_data_handler,
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &page_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &data_uri));

    printf("\n[DASHBOARD] Connect phone to Wi-Fi SSID: %s | Password: %s\n",
           DASHBOARD_AP_SSID, DASHBOARD_AP_PASSWORD);
    printf("[DASHBOARD] Open http://192.168.4.1 in your phone browser.\n\n");
}

static void esp_ble_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        check_step("esp_ble_gap_start_scanning", esp_ble_gap_start_scanning(0));
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        if (param->scan_rst.search_evt != ESP_GAP_SEARCH_INQ_RES_EVT) {
            break;
        }

        if (memcmp(param->scan_rst.bda, TARGET_BDA, ESP_BD_ADDR_LEN) == 0) {
            int raw_rssi = param->scan_rst.rssi;
            float filtered_rssi = update_smoothed_rssi(raw_rssi);
            float distance_m = estimate_distance_meters(filtered_rssi);

            update_robot_proximity_state(filtered_rssi);
            update_dashboard_data(raw_rssi, filtered_rssi);

            printf("[TARGET FOUND] %02X:%02X:%02X:%02X:%02X:%02X | raw RSSI: %d dBm | "
                   "smoothed RSSI: %.1f dBm | est distance: %.2f m | status: %s\n",
                   param->scan_rst.bda[0], param->scan_rst.bda[1], param->scan_rst.bda[2],
                   param->scan_rst.bda[3], param->scan_rst.bda[4], param->scan_rst.bda[5],
                   raw_rssi, filtered_rssi, distance_m, proximity_status_text(robot_confirmed_near));
        }
        break;

    default:
        break;
    }
}

// Background task that monitors packet gaps and applies RSSI decay
static void rssi_gap_monitor_task(void *pvParameter)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));  // Check every 500ms
        
        if (!rssi_filter_ready) {
            continue;
        }
        
        int64_t now_ms = esp_timer_get_time() / 1000;
        int64_t time_since_last_ms = now_ms - last_rssi_update_ms;
        
        // Only apply decay during gaps (>500ms without packet)
        if (time_since_last_ms > 500 && time_since_last_ms < SIGNAL_LOST_MS) {
            float decayed_rssi = apply_rssi_decay();
            
            // Update state based on decayed RSSI - allows natural transitions during gaps
            update_robot_proximity_state(decayed_rssi);
        }
    }
}

void app_main(void)
{
    dashboard_mutex = xSemaphoreCreateMutex();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    start_dashboard_server();

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    bt_cfg.mode = ESP_BT_MODE_BLE;

    check_step("esp_bt_controller_init", esp_bt_controller_init(&bt_cfg));
    check_step("esp_bt_controller_enable", esp_bt_controller_enable(ESP_BT_MODE_BLE));
    check_step("esp_bluedroid_init", esp_bluedroid_init());
    check_step("esp_bluedroid_enable", esp_bluedroid_enable());
    check_step("esp_ble_gap_register_callback", esp_ble_gap_register_callback(esp_ble_gap_cb));
    check_step("esp_ble_gap_set_scan_params", esp_ble_gap_set_scan_params(&ble_scan_params));

    // Start background task to monitor packet gaps and apply RSSI decay
    xTaskCreate(rssi_gap_monitor_task, "rssi_monitor", 2048, NULL, 5, NULL);

    printf("[BLE SCANNER] Initialized. Looking for target MAC %02X:%02X:%02X:%02X:%02X:%02X...\n",
           TARGET_BDA[0], TARGET_BDA[1], TARGET_BDA[2], TARGET_BDA[3], TARGET_BDA[4], TARGET_BDA[5]);
}

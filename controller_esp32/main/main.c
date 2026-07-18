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

#define RELAY_CALL_PIN         GPIO_NUM_14
#define RELAY_DOOR_HOLD_PIN    GPIO_NUM_13
#define RELAY_FLOOR_SELECT_PIN GPIO_NUM_12

#define CMD_CALL_ELEVATOR      "CMD_CALL_ELEVATOR"
#define CMD_HOLD_DOOR_OPEN     "CMD_HOLD_DOOR_OPEN"
#define CMD_RELEASE_DOOR       "CMD_RELEASE_DOOR"
#define CMD_SELECT_FLOOR       "CMD_SELECT_FLOOR"

static const uint8_t board2_mac[6] = {0x30, 0x76, 0xF5, 0xF7, 0x57, 0x48};

static volatile bool pulse_call_requested = false;
static volatile bool pulse_select_requested = false;

static void relay_init(void)
{
    gpio_config_t config = {
        .pin_bit_mask = (1ULL << RELAY_CALL_PIN) |
                        (1ULL << RELAY_DOOR_HOLD_PIN) |
                        (1ULL << RELAY_FLOOR_SELECT_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&config);
    gpio_set_level(RELAY_CALL_PIN, 0);
    gpio_set_level(RELAY_DOOR_HOLD_PIN, 0);
    gpio_set_level(RELAY_FLOOR_SELECT_PIN, 0);
}

static void pulse_relay(gpio_num_t pin)
{
    gpio_set_level(pin, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(pin, 0);
}

static void process_incoming_command(const char *command)
{
    if (strcmp(command, CMD_CALL_ELEVATOR) == 0) {
        printf("[CTRL] Received CMD_CALL_ELEVATOR. Triggering elevator call relay.\n");
        pulse_call_requested = true;
    } else if (strcmp(command, CMD_HOLD_DOOR_OPEN) == 0) {
        printf("[CTRL] Received CMD_HOLD_DOOR_OPEN. Engaging HOLD DOOR relay.\n");
        gpio_set_level(RELAY_DOOR_HOLD_PIN, 1);
    } else if (strcmp(command, CMD_RELEASE_DOOR) == 0) {
        printf("[CTRL] Received CMD_RELEASE_DOOR. Releasing HOLD DOOR relay.\n");
        gpio_set_level(RELAY_DOOR_HOLD_PIN, 0);
    } else if (strcmp(command, CMD_SELECT_FLOOR) == 0) {
        printf("[CTRL] Received CMD_SELECT_FLOOR. Triggering floor select relay.\n");
        pulse_select_requested = true;
    } else {
        printf("[CTRL] Unknown command received: %s\n", command);
    }
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

    process_incoming_command(incoming);
}

static void on_data_sent(const esp_now_send_info_t *info, esp_now_send_status_t status)
{
    printf("[ESP-NOW] TX status: %s\n", status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
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
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent));

    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, board2_mac, sizeof(board2_mac));
    peer_info.channel = 1;
    peer_info.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

    relay_init();
    printf("[CTRL] Controller online. Relays initialized and ready to receive floor board commands.\n");

    while (1) {
        if (pulse_call_requested) {
            pulse_call_requested = false;
            printf("[RELAY] Pulsing CALL ELEVATOR relay.\n");
            pulse_relay(RELAY_CALL_PIN);
        }

        if (pulse_select_requested) {
            pulse_select_requested = false;
            printf("[RELAY] Pulsing SELECT FLOOR relay.\n");
            pulse_relay(RELAY_FLOOR_SELECT_PIN);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

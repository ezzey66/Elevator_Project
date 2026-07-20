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
#include "esp_mac.h"
#include "esp_system.h"

// Relay module mapping.
// Adjust only these GPIOs if the physical wiring changes.
#define RELAY_IN1_SERVICE_MODE_PIN GPIO_NUM_13
#define RELAY_IN2_FLOOR_1_PIN      GPIO_NUM_12
#define RELAY_IN3_FLOOR_2_PIN      GPIO_NUM_14
#define RELAY_IN4_DOOR_HOLD_PIN    GPIO_NUM_27

#define RELAY_PULSE_MS             1000

// ESP-NOW commands received from esp32_2.
#define CMD_SERVICE_MODE_ON        "CMD_SERVICE_MODE_ON"
#define CMD_SERVICE_MODE_OFF       "CMD_SERVICE_MODE_OFF"
#define CMD_CALL_FLOOR_1           "CMD_CALL_FLOOR_1"
#define CMD_CALL_FLOOR_2           "CMD_CALL_FLOOR_2"
#define CMD_HOLD_DOOR_OPEN         "CMD_HOLD_DOOR_OPEN"
#define CMD_RELEASE_DOOR           "CMD_RELEASE_DOOR"
#define CMD_ROBOT_READY            "CMD_ROBOT_READY"
#define CMD_CONTROLLER_READY       "CMD_CONTROLLER_READY"

static const uint8_t peer_macs[][6] = {
    {0x30, 0x76, 0xF5, 0xF7, 0x57, 0x48}, // ESP32_2 robot board
};
static const char *peer_names[] = {
    "ESP32_2",
};
static const size_t peer_count = sizeof(peer_macs) / sizeof(peer_macs[0]);
static uint8_t own_mac[6] = {0};

typedef enum {
    CTRL_IDLE = 0,
    CTRL_SERVICE_ACTIVE,
    CTRL_DOOR_HELD,
    CTRL_ERROR,
} controller_state_t;

static volatile bool pulse_floor_1_requested = false;
static volatile bool pulse_floor_2_requested = false;
static volatile bool service_mode_active = false;
static volatile bool hold_door_active = false;
static bool last_service_mode_active = false;
static bool last_hold_door_active = false;
static volatile controller_state_t controller_state = CTRL_IDLE;

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

static bool send_command_to_robot(const char *command)
{
    if (peer_count == 0) {
        printf("[ESP-NOW] No remote peer configured, cannot send: %s\n", command);
        return false;
    }

    esp_err_t err = esp_now_send(peer_macs[0], (const uint8_t *)command, strlen(command));
    if (err != ESP_OK) {
        printf("[ESP-NOW] Failed to send: %s (err=%d)\n", command, err);
        return false;
    }

    printf("[ESP-NOW] Sent: %s -> ", command);
    print_mac(peer_macs[0]);
    printf("\n");
    return true;
}

static void relay_init(void)
{
    gpio_config_t config = {
        .pin_bit_mask = (1ULL << RELAY_IN1_SERVICE_MODE_PIN) |
                        (1ULL << RELAY_IN2_FLOOR_1_PIN) |
                        (1ULL << RELAY_IN3_FLOOR_2_PIN) |
                        (1ULL << RELAY_IN4_DOOR_HOLD_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&config);
    gpio_set_level(RELAY_IN1_SERVICE_MODE_PIN, 0);
    gpio_set_level(RELAY_IN2_FLOOR_1_PIN, 0);
    gpio_set_level(RELAY_IN3_FLOOR_2_PIN, 0);
    gpio_set_level(RELAY_IN4_DOOR_HOLD_PIN, 0);
}

static void pulse_relay(gpio_num_t pin, const char *name)
{
    printf("[RELAY] Pulsing %s.\n", name);
    gpio_set_level(pin, 1);
    vTaskDelay(pdMS_TO_TICKS(RELAY_PULSE_MS));
    gpio_set_level(pin, 0);
}

static void set_service_mode(bool active)
{
    gpio_set_level(RELAY_IN1_SERVICE_MODE_PIN, active ? 1 : 0);
    service_mode_active = active;
    controller_state = active ? CTRL_SERVICE_ACTIVE : CTRL_IDLE;
}

static void set_door_hold(bool active)
{
    gpio_set_level(RELAY_IN4_DOOR_HOLD_PIN, active ? 1 : 0);
    hold_door_active = active;
    controller_state = active ? CTRL_DOOR_HELD :
                       service_mode_active ? CTRL_SERVICE_ACTIVE : CTRL_IDLE;
}

static void process_incoming_command(const char *command)
{
    if (strcmp(command, CMD_SERVICE_MODE_ON) == 0) {
        printf("[CTRL] Service mode ON.\n");
        set_service_mode(true);
    } else if (strcmp(command, CMD_SERVICE_MODE_OFF) == 0) {
        printf("[CTRL] Service mode OFF.\n");
        set_service_mode(false);
    } else if (strcmp(command, CMD_CALL_FLOOR_1) == 0) {
        printf("[CTRL] Floor 1 requested.\n");
        pulse_floor_1_requested = true;
    } else if (strcmp(command, CMD_CALL_FLOOR_2) == 0) {
        printf("[CTRL] Floor 2 requested.\n");
        pulse_floor_2_requested = true;
    } else if (strcmp(command, CMD_HOLD_DOOR_OPEN) == 0) {
        printf("[CTRL] Holding door open.\n");
        set_door_hold(true);
    } else if (strcmp(command, CMD_RELEASE_DOOR) == 0) {
        printf("[CTRL] Releasing door hold.\n");
        set_door_hold(false);
    } else if (strcmp(command, CMD_ROBOT_READY) == 0) {
        printf("[CTRL] Robot reported ready.\n");
    } else {
        printf("[ESP-NOW] Unknown controller command: %s\n", command);
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
    (void)info;
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

    ESP_ERROR_CHECK(esp_read_mac(own_mac, ESP_MAC_WIFI_STA));
    printf("[ESP-NOW] Controller local MAC: ");
    print_mac(own_mac);
    printf("\n");

    for (size_t i = 0; i < peer_count; ++i) {
        if (!add_peer(peer_macs[i], peer_names[i])) {
            printf("[ESP-NOW] Warning: peer %s may not be reachable.\n", peer_names[i]);
        }
    }

    relay_init();
    send_command_to_robot(CMD_CONTROLLER_READY);
    printf("[CTRL] Controller online. Relay mapping: IN1 service, IN2 floor1, IN3 floor2, IN4 door hold.\n");

    while (1) {
        if (pulse_floor_1_requested) {
            pulse_floor_1_requested = false;
            pulse_relay(RELAY_IN2_FLOOR_1_PIN, "FLOOR 1 / IN2");
        }

        if (pulse_floor_2_requested) {
            pulse_floor_2_requested = false;
            pulse_relay(RELAY_IN3_FLOOR_2_PIN, "FLOOR 2 / IN3");
        }

        if (service_mode_active != last_service_mode_active) {
            printf("[STATE] Service mode %s.\n", service_mode_active ? "active" : "inactive");
            last_service_mode_active = service_mode_active;
        }

        if (hold_door_active != last_hold_door_active) {
            printf("[STATE] Door hold %s.\n", hold_door_active ? "active" : "released");
            last_hold_door_active = hold_door_active;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

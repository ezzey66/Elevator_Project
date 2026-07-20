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
#include "esp_system.h"

#define RELAY_CALL_PIN         GPIO_NUM_14
#define RELAY_DOOR_HOLD_PIN    GPIO_NUM_13
#define RELAY_FLOOR_SELECT_PIN GPIO_NUM_12

// ESP-NOW commands received from the robot board.
#define CMD_CALL_ELEVATOR      "CMD_CALL_ELEVATOR"
#define CMD_HOLD_DOOR_OPEN     "CMD_HOLD_DOOR_OPEN"
#define CMD_RELEASE_DOOR       "CMD_RELEASE_DOOR"
#define CMD_SELECT_FLOOR       "CMD_SELECT_FLOOR"
#define CMD_CONTROLLER_READY   "CMD_CONTROLLER_READY"

// Peer list: configure remote ESP32 boards here. The controller sends commands to
// the robot board and receives commands back from it.
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
    CTRL_CALLED,
    CTRL_DOOR_HELD,
    CTRL_WAITING_FOR_FLOOR,
    CTRL_FLOOR_SELECTED,
    CTRL_ERROR,
} controller_state_t;

static volatile bool pulse_call_requested = false;
static volatile bool pulse_select_requested = false;
static volatile bool hold_door_active = false;
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

// Send a command to the first configured ESP-NOW peer.
// This is currently the robot board, but additional peers can be added to the list.
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
        controller_state = CTRL_CALLED;
    } else if (strcmp(command, CMD_HOLD_DOOR_OPEN) == 0) {
        printf("[CTRL] Received CMD_HOLD_DOOR_OPEN. Engaging HOLD DOOR relay.\n");
        gpio_set_level(RELAY_DOOR_HOLD_PIN, 1);
        hold_door_active = true;
        controller_state = CTRL_DOOR_HELD;
    } else if (strcmp(command, CMD_RELEASE_DOOR) == 0) {
        printf("[CTRL] Received CMD_RELEASE_DOOR. Releasing HOLD DOOR relay.\n");
        gpio_set_level(RELAY_DOOR_HOLD_PIN, 0);
        hold_door_active = false;
        controller_state = CTRL_WAITING_FOR_FLOOR;
    } else if (strcmp(command, CMD_SELECT_FLOOR) == 0) {
        printf("[CTRL] Received CMD_SELECT_FLOOR. Triggering floor select relay.\n");
        pulse_select_requested = true;
        controller_state = CTRL_FLOOR_SELECTED;
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
    printf("[ESP-NOW] Received: %s\n", incoming);

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

    // Print local MAC so you can verify peer configuration matches physical boards.
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

    // Send a startup handshake so the remote board can verify ESP-NOW connectivity.
    send_command_to_robot(CMD_CONTROLLER_READY);
    printf("[CTRL] Controller online. Relays initialized.\n");

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

        if (hold_door_active != last_hold_door_active) {
            if (hold_door_active) {
                printf("[STATE] HOLD_DOOR active. Keeping door relay engaged.\n");
            } else {
                printf("[STATE] HOLD_DOOR released. Door relay disengaged.\n");
            }
            last_hold_door_active = hold_door_active;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

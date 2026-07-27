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
#define RELAY_IN1_SERVICE_MODE_PIN GPIO_NUM_14
#define RELAY_IN2_FLOOR_1_PIN      GPIO_NUM_13
#define RELAY_IN3_FLOOR_2_PIN      GPIO_NUM_12
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

static bool send_command_to_robot(const char *command);
static void set_service_mode(bool active);
static void set_door_hold(bool active);

typedef enum {
    ESPNOW_EVENT_TYPE_UNKNOWN = 0,
    ESPNOW_EVENT_TYPE_SERVICE_MODE_ON,
    ESPNOW_EVENT_TYPE_SERVICE_MODE_OFF,
    ESPNOW_EVENT_TYPE_CALL_FLOOR,
    ESPNOW_EVENT_TYPE_HOLD_DOOR_OPEN,
    ESPNOW_EVENT_TYPE_RELEASE_DOOR,
    ESPNOW_EVENT_TYPE_ROBOT_READY,
    ESPNOW_EVENT_TYPE_CONTROLLER_READY,
} espnow_event_type_t;

typedef struct {
    espnow_event_type_t event_type;
    uint8_t floor_id;
} espnow_message_t;

static const char *espnow_event_type_to_string(espnow_event_type_t event_type)
{
    switch (event_type) {
        case ESPNOW_EVENT_TYPE_SERVICE_MODE_ON: return "SERVICE_MODE_ON";
        case ESPNOW_EVENT_TYPE_SERVICE_MODE_OFF: return "SERVICE_MODE_OFF";
        case ESPNOW_EVENT_TYPE_CALL_FLOOR: return "CALL_FLOOR";
        case ESPNOW_EVENT_TYPE_HOLD_DOOR_OPEN: return "HOLD_DOOR_OPEN";
        case ESPNOW_EVENT_TYPE_RELEASE_DOOR: return "RELEASE_DOOR";
        case ESPNOW_EVENT_TYPE_ROBOT_READY: return "ROBOT_READY";
        case ESPNOW_EVENT_TYPE_CONTROLLER_READY: return "CONTROLLER_READY";
        default: return "UNKNOWN";
    }
}

static bool parse_espnow_message(const uint8_t *data, int len, espnow_message_t *message)
{
    if (len != sizeof(*message) || message == NULL) {
        return false;
    }

    memcpy(message, data, sizeof(*message));
    return (message->event_type != ESPNOW_EVENT_TYPE_UNKNOWN);
}

static bool handle_event(espnow_event_type_t event_type, uint8_t floor_id)
{
    switch (event_type) {
        case ESPNOW_EVENT_TYPE_SERVICE_MODE_ON:
            printf("[CTRL] Service mode ON.\n");
            set_service_mode(true);
            return true;
        case ESPNOW_EVENT_TYPE_SERVICE_MODE_OFF:
            printf("[CTRL] Service mode OFF.\n");
            set_service_mode(false);
            return true;
        case ESPNOW_EVENT_TYPE_CALL_FLOOR:
            if (floor_id == 1) {
                printf("[CTRL] Floor 1 requested.\n");
                pulse_floor_1_requested = true;
                return true;
            } else if (floor_id == 2) {
                printf("[CTRL] Floor 2 requested.\n");
                pulse_floor_2_requested = true;
                return true;
            }
            printf("[ESP-NOW] Invalid floor_id for CALL_FLOOR: %u\n", floor_id);
            return false;
        case ESPNOW_EVENT_TYPE_HOLD_DOOR_OPEN:
            printf("[CTRL] Holding door open.\n");
            set_door_hold(true);
            return true;
        case ESPNOW_EVENT_TYPE_RELEASE_DOOR:
            printf("[CTRL] Releasing door hold.\n");
            set_door_hold(false);
            return true;
        case ESPNOW_EVENT_TYPE_ROBOT_READY:
            printf("[CTRL] Robot reported ready.\n");
            return true;
        case ESPNOW_EVENT_TYPE_CONTROLLER_READY:
            printf("[CTRL] Controller ready event received.\n");
            return true;
        default:
            printf("[ESP-NOW] Unknown event type: %s\n", espnow_event_type_to_string(event_type));
            return false;
    }
}

static bool process_incoming_espnow_message(const espnow_message_t *message)
{
    if (message == NULL) {
        return false;
    }
    return handle_event(message->event_type, message->floor_id);
}

static bool process_incoming_command(const char *command)
{
    if (strcmp(command, CMD_SERVICE_MODE_ON) == 0) {
        return handle_event(ESPNOW_EVENT_TYPE_SERVICE_MODE_ON, 0);
    } else if (strcmp(command, CMD_SERVICE_MODE_OFF) == 0) {
        return handle_event(ESPNOW_EVENT_TYPE_SERVICE_MODE_OFF, 0);
    } else if (strcmp(command, CMD_CALL_FLOOR_1) == 0) {
        return handle_event(ESPNOW_EVENT_TYPE_CALL_FLOOR, 1);
    } else if (strcmp(command, CMD_CALL_FLOOR_2) == 0) {
        return handle_event(ESPNOW_EVENT_TYPE_CALL_FLOOR, 2);
    } else if (strcmp(command, CMD_HOLD_DOOR_OPEN) == 0) {
        return handle_event(ESPNOW_EVENT_TYPE_HOLD_DOOR_OPEN, 0);
    } else if (strcmp(command, CMD_RELEASE_DOOR) == 0) {
        return handle_event(ESPNOW_EVENT_TYPE_RELEASE_DOOR, 0);
    } else if (strcmp(command, CMD_ROBOT_READY) == 0) {
        return handle_event(ESPNOW_EVENT_TYPE_ROBOT_READY, 0);
    }

    printf("[ESP-NOW] Unknown controller command: %s\n", command);
    return false;
}

static bool send_espnow_event(espnow_event_type_t event_type, uint8_t floor_id)
{
    switch (event_type) {
        case ESPNOW_EVENT_TYPE_SERVICE_MODE_ON:
            return send_command_to_robot(CMD_SERVICE_MODE_ON);
        case ESPNOW_EVENT_TYPE_SERVICE_MODE_OFF:
            return send_command_to_robot(CMD_SERVICE_MODE_OFF);
        case ESPNOW_EVENT_TYPE_CALL_FLOOR:
            if (floor_id == 1) {
                return send_command_to_robot(CMD_CALL_FLOOR_1);
            } else if (floor_id == 2) {
                return send_command_to_robot(CMD_CALL_FLOOR_2);
            }
            printf("[ESP-NOW] Invalid floor_id for event CALL_FLOOR: %u\n", floor_id);
            return false;
        case ESPNOW_EVENT_TYPE_HOLD_DOOR_OPEN:
            return send_command_to_robot(CMD_HOLD_DOOR_OPEN);
        case ESPNOW_EVENT_TYPE_RELEASE_DOOR:
            return send_command_to_robot(CMD_RELEASE_DOOR);
        case ESPNOW_EVENT_TYPE_ROBOT_READY:
            return send_command_to_robot(CMD_ROBOT_READY);
        case ESPNOW_EVENT_TYPE_CONTROLLER_READY:
            return send_command_to_robot(CMD_CONTROLLER_READY);
        default:
            printf("[ESP-NOW] Unsupported event type for send: %s\n", espnow_event_type_to_string(event_type));
            return false;
    }
}

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


static void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (len <= 0 || len >= 64) {
        return;
    }

    espnow_message_t message = {0};
    if (parse_espnow_message(data, len, &message)) {
        printf("[ESP-NOW] RX structured message from %02X:%02X:%02X:%02X:%02X:%02X -> %s floor=%u\n",
               recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
               recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5],
               espnow_event_type_to_string(message.event_type), message.floor_id);
        if (!process_incoming_espnow_message(&message)) {
            printf("[ESP-NOW] Failed to process structured message. Falling back to text parse.\n");
        }
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
    send_espnow_event(ESPNOW_EVENT_TYPE_CONTROLLER_READY, 0);
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

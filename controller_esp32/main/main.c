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
#include "esp_mac.h"
#include "esp_system.h"

// Relay module mapping.
// Adjust only these GPIOs if the physical wiring changes.
#define RELAY_IN1_SERVICE_MODE_PIN GPIO_NUM_14
#define RELAY_IN2_FLOOR_1_PIN      GPIO_NUM_13
#define RELAY_IN3_FLOOR_2_PIN      GPIO_NUM_12
#define RELAY_IN4_DOOR_HOLD_PIN    GPIO_NUM_27

#define RELAY_PULSE_MS             1000

typedef enum {
   CTRL_IDLE = 0,
   CTRL_SERVICE_ACTIVE,
   CTRL_DOOR_HELD,
   CTRL_ERROR,
} controller_state_t;

typedef enum {
    CTRL_FLOW_IDLE = 0,
    CTRL_FLOW_WAITING_FOR_ORIGIN_RELEASE,
    CTRL_FLOW_WAITING_FOR_DESTINATION_RELEASE,
} controller_flow_state_t;

typedef struct {
    uint8_t floor_id;
    uint8_t peer_addr[6];
    bool active;
} floor_peer_t;

static floor_peer_t floor_peers[8] = {0};
static uint8_t own_mac[6] = {0};
static uint8_t robot_current_floor = 0;
static controller_flow_state_t controller_flow_state = CTRL_FLOW_IDLE;

static volatile bool pulse_floor_1_requested = false;
static volatile bool pulse_floor_2_requested = false;
static volatile bool service_mode_active = false;
static volatile bool hold_door_active = false;
static bool last_service_mode_active = false;
static bool last_hold_door_active = false;
static volatile controller_state_t controller_state = CTRL_IDLE;

typedef enum {
    ESPNOW_EVENT_TYPE_UNKNOWN = 0,
    ESPNOW_EVENT_TYPE_SERVICE_MODE_ON,
    ESPNOW_EVENT_TYPE_SERVICE_MODE_OFF,
    EVENT_REQUEST_ELEVATOR,
    ESPNOW_EVENT_TYPE_HOLD_DOOR_OPEN,
    ESPNOW_EVENT_TYPE_RELEASE_DOOR,
    ESPNOW_EVENT_TYPE_ROBOT_READY,
    ESPNOW_EVENT_TYPE_CONTROLLER_READY,
} espnow_event_type_t;

static void print_mac(const uint8_t *mac);
static bool send_espnow_event(espnow_event_type_t event_type, uint8_t floor_id);
static void set_service_mode(bool active);
static void set_door_hold(bool active);
static void pulse_relay(gpio_num_t pin, const char *name);

typedef struct {
    espnow_event_type_t event_type;
    uint8_t floor_id;
} espnow_message_t;

static const char *espnow_event_type_to_string(espnow_event_type_t event_type)
{
    switch (event_type) {
        case ESPNOW_EVENT_TYPE_SERVICE_MODE_ON: return "SERVICE_MODE_ON";
        case ESPNOW_EVENT_TYPE_SERVICE_MODE_OFF: return "SERVICE_MODE_OFF";
        case EVENT_REQUEST_ELEVATOR: return "REQUEST_ELEVATOR";
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
        case EVENT_REQUEST_ELEVATOR:
            if (floor_id == 2) {
                printf("[CTRL] Floor 2 request received; preparing entry-side mission.\n");
                robot_current_floor = 2;
                pulse_floor_2_requested = true;
                controller_flow_state = CTRL_FLOW_WAITING_FOR_ORIGIN_RELEASE;

                // Notify the destination board that a trip is starting.
                if (!send_espnow_event(EVENT_REQUEST_ELEVATOR, 1)) {
                    printf("[CTRL] Warning: could not notify floor 1 destination board.\n");
                }
                return true;
            } else if (floor_id == 1) {
                printf("[CTRL] Floor 1 request received; destination-side mission is active.\n");
                robot_current_floor = 1;
                controller_flow_state = CTRL_FLOW_WAITING_FOR_DESTINATION_RELEASE;
                return true;
            }
            printf("[ESP-NOW] Invalid floor_id for REQUEST_ELEVATOR: %u\n", floor_id);
            return false;
        case ESPNOW_EVENT_TYPE_HOLD_DOOR_OPEN:
            printf("[CTRL] Holding door open for floor %u.\n", floor_id);
            pulse_relay(RELAY_IN4_DOOR_HOLD_PIN, "DOOR HOLD / IN4");
            set_door_hold(true);
            return true;
        case ESPNOW_EVENT_TYPE_RELEASE_DOOR:
            if (floor_id == 2 && controller_flow_state == CTRL_FLOW_WAITING_FOR_ORIGIN_RELEASE) {
                printf("[CTRL] Origin floor 2 release received; switching to floor 1.\n");
                set_door_hold(false);
                robot_current_floor = 1;
                pulse_floor_1_requested = true;
                controller_flow_state = CTRL_FLOW_WAITING_FOR_DESTINATION_RELEASE;
                return true;
            }
            if (floor_id == 1 && controller_flow_state == CTRL_FLOW_WAITING_FOR_DESTINATION_RELEASE) {
                printf("[CTRL] Destination floor 1 release received; mission complete.\n");
                set_door_hold(false);
                set_service_mode(false);
                robot_current_floor = 1;
                controller_flow_state = CTRL_FLOW_IDLE;
                return true;
            }
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

static bool send_espnow_event(espnow_event_type_t event_type, uint8_t floor_id)
{
    if (floor_id == 0) {
        printf("[ESP-NOW] Cannot send event %s without a target floor.\n", espnow_event_type_to_string(event_type));
        return false;
    }

    uint8_t *peer_addr = NULL;
    for (size_t i = 0; i < sizeof(floor_peers) / sizeof(floor_peers[0]); ++i) {
        if (floor_peers[i].active && floor_peers[i].floor_id == floor_id) {
            peer_addr = floor_peers[i].peer_addr;
            break;
        }
    }

    if (peer_addr == NULL) {
        printf("[ESP-NOW] No peer registered for floor %u, cannot send event %s\n", floor_id, espnow_event_type_to_string(event_type));
        return false;
    }

    espnow_message_t message = {
        .event_type = event_type,
        .floor_id = floor_id,
    };

    esp_err_t err = esp_now_send(peer_addr, (const uint8_t *)&message, sizeof(message));
    if (err != ESP_OK) {
        printf("[ESP-NOW] Failed to send event %s to floor %u (err=%d)\n", espnow_event_type_to_string(event_type), floor_id, err);
        return false;
    }

    printf("[ESP-NOW] Sent event %s floor=%u -> ", espnow_event_type_to_string(event_type), floor_id);
    print_mac(peer_addr);
    printf("\n");
    return true;
}

static void print_mac(const uint8_t *mac)
{
    printf("%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static bool add_peer_for_floor(uint8_t floor_id, const uint8_t *mac)
{
    if (mac == NULL) {
        return false;
    }

    for (size_t i = 0; i < sizeof(floor_peers) / sizeof(floor_peers[0]); ++i) {
        if (floor_peers[i].active && floor_peers[i].floor_id == floor_id) {
            memcpy(floor_peers[i].peer_addr, mac, ESP_NOW_ETH_ALEN);
            return true;
        }
    }

    for (size_t i = 0; i < sizeof(floor_peers) / sizeof(floor_peers[0]); ++i) {
        if (!floor_peers[i].active) {
            floor_peers[i].floor_id = floor_id;
            memcpy(floor_peers[i].peer_addr, mac, ESP_NOW_ETH_ALEN);
            floor_peers[i].active = true;
            break;
        }
    }

    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, mac, ESP_NOW_ETH_ALEN);
    peer_info.channel = 1;
    peer_info.encrypt = false;

    esp_err_t err = esp_now_add_peer(&peer_info);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        printf("[ESP-NOW] Failed to add peer for floor %u: ", floor_id);
        print_mac(mac);
        printf(" (err=%d)\n", err);
        return false;
    }

    printf("[ESP-NOW] Added peer for floor %u: ", floor_id);
    print_mac(mac);
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
        if (message.floor_id != 0) {
            add_peer_for_floor(message.floor_id, recv_info->src_addr);
        }
        if (message.event_type == EVENT_REQUEST_ELEVATOR) {
            if (message.floor_id == 2) {
                robot_current_floor = 2;
            } else if (message.floor_id == 1) {
                robot_current_floor = 1;
            }
        }
        if (!process_incoming_espnow_message(&message)) {
            printf("[ESP-NOW] Failed to process structured message.\n");
        }
        return;
    }

    char incoming[64] = {0};
    snprintf(incoming, sizeof(incoming), "%.*s", len, data);
    printf("[ESP-NOW] RX from %02X:%02X:%02X:%02X:%02X:%02X -> %s\n",
           recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
           recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5],
           incoming);
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

    relay_init();
    send_espnow_event(ESPNOW_EVENT_TYPE_CONTROLLER_READY, 1);
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

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_event.h"

#include "esp_mac.h"

#include "espnow_protocol.h"


/* ============================================================
 * ARES - MULTI-FLOOR CONTROLLER
 * ============================================================ */


/* ============================================================
 * CONFIGURAÇÃO
 * ============================================================ */

#define ESPNOW_CHANNEL          1
#define MAX_FLOORS              8
#define RELAY_PULSE_MS          1000
#define CONTROLLER_FLOOR_ID     0

#define CLI_BUFFER_SIZE         80


/* ============================================================
 * RELÉS
 * ============================================================ */

#define RELAY_FLOOR_1_GPIO      13
#define RELAY_FLOOR_2_GPIO      12
#define RELAY_SERVICE_GPIO      14
#define RELAY_DOOR_HOLD_GPIO    27

#define RELAY_ON                1
#define RELAY_OFF               0


/* ============================================================
 * FLOOR PEER
 * ============================================================ */

typedef struct
{
    bool registered;
    uint8_t floor_id;
    uint8_t mac[ESP_NOW_ETH_ALEN];

} floor_peer_t;


/* ============================================================
 * MISSÃO
 * ============================================================ */

typedef enum
{
    MISSION_IDLE = 0,

    MISSION_WAITING_ORIGIN_DOOR,
    MISSION_AT_ORIGIN,
    MISSION_WAITING_ORIGIN_DOOR_CLOSED,

    MISSION_TRAVELLING_TO_DESTINATION,
    MISSION_WAITING_DESTINATION_DOOR,
    MISSION_AT_DESTINATION,
    MISSION_WAITING_DESTINATION_DOOR_CLOSED,

    MISSION_COMPLETE,
    MISSION_ERROR

} mission_state_t;


typedef struct
{
    bool active;

    uint8_t origin_floor;
    uint8_t destination_floor;

    mission_state_t state;

} mission_t;


/* ============================================================
 * RELÉS DOS PISOS
 * ============================================================ */

typedef struct
{
    uint8_t floor_id;
    gpio_num_t relay_gpio;

} floor_relay_t;


static const floor_relay_t floor_relays[] =
{
    {
        .floor_id = 1,
        .relay_gpio = RELAY_FLOOR_1_GPIO
    },

    {
        .floor_id = 2,
        .relay_gpio = RELAY_FLOOR_2_GPIO
    }
};


static const size_t floor_relay_count =
    sizeof(floor_relays) /
    sizeof(floor_relays[0]);


/* ============================================================
 * ESTADO GLOBAL
 * ============================================================ */

static floor_peer_t floor_peers[MAX_FLOORS + 1] =
{
    0
};


static mission_t current_mission =
{
    .active = false,
    .origin_floor = 0,
    .destination_floor = 0,
    .state = MISSION_IDLE
};


static volatile uint8_t pending_floor_pulse = 0;


/* ============================================================
 * CLI
 * ============================================================ */

static char cli_buffer[CLI_BUFFER_SIZE] =
{
    0
};

static volatile size_t cli_index = 0;

static bool cli_started = false;


/* ============================================================
 * REDESENHAR PROMPT
 * ============================================================ */

static void cli_redraw_prompt(void)
{
    if (!cli_started)
    {
        return;
    }

    /*
     * \r          -> volta ao início da linha
     * \033[2K     -> limpa a linha atual
     */
    printf(
        "\r\033[2KARES> %s",
        cli_buffer
    );

    fflush(stdout);
}


/* ============================================================
 * PREPARAR LOG ASSÍNCRONO
 * ============================================================ */

static void console_begin_async_log(void)
{
    if (cli_started)
    {
        printf(
            "\r\033[2K"
        );
    }
}


/* ============================================================
 * TERMINAR LOG ASSÍNCRONO
 * ============================================================ */

static void console_end_async_log(void)
{
    if (cli_started)
    {
        cli_redraw_prompt();
    }
}


/* ============================================================
 * MAC
 * ============================================================ */

static void print_mac(
    const uint8_t *mac
)
{
    if (mac == NULL)
    {
        return;
    }

    printf(
        "%02X:%02X:%02X:%02X:%02X:%02X",
        mac[0],
        mac[1],
        mac[2],
        mac[3],
        mac[4],
        mac[5]
    );
}


/* ============================================================
 * NOMES DAS MENSAGENS
 * ============================================================ */

static const char *message_type_to_string(
    espnow_message_type_t type
)
{
    switch (type)
    {
        case MSG_NODE_READY:
            return "NODE_READY";

        case MSG_REQUEST_ELEVATOR:
            return "REQUEST_ELEVATOR";

        case MSG_HOLD_DOOR_OPEN:
            return "HOLD_DOOR_OPEN";

        case MSG_RELEASE_DOOR:
            return "RELEASE_DOOR";

        case MSG_DOOR_CLOSED:
            return "DOOR_CLOSED";

        case MSG_CONTROLLER_READY:
            return "CONTROLLER_READY";

        case MSG_START_ORIGIN:
            return "START_ORIGIN";

        case MSG_START_DESTINATION:
            return "START_DESTINATION";

        case MSG_RESET_NODE:
            return "RESET_NODE";

        case MSG_SERVICE_MODE_ON:
            return "SERVICE_MODE_ON";

        case MSG_SERVICE_MODE_OFF:
            return "SERVICE_MODE_OFF";

        default:
            return "UNKNOWN";
    }
}


/* ============================================================
 * NOMES DOS ESTADOS
 * ============================================================ */

static const char *mission_state_to_string(
    mission_state_t state
)
{
    switch (state)
    {
        case MISSION_IDLE:
            return "IDLE";

        case MISSION_WAITING_ORIGIN_DOOR:
            return "WAITING_ORIGIN_DOOR";

        case MISSION_AT_ORIGIN:
            return "AT_ORIGIN";

        case MISSION_WAITING_ORIGIN_DOOR_CLOSED:
            return "WAITING_ORIGIN_DOOR_CLOSED";

        case MISSION_TRAVELLING_TO_DESTINATION:
            return "TRAVELLING_TO_DESTINATION";

        case MISSION_WAITING_DESTINATION_DOOR:
            return "WAITING_DESTINATION_DOOR";

        case MISSION_AT_DESTINATION:
            return "AT_DESTINATION";

        case MISSION_WAITING_DESTINATION_DOOR_CLOSED:
            return "WAITING_DESTINATION_DOOR_CLOSED";

        case MISSION_COMPLETE:
            return "COMPLETE";

        case MISSION_ERROR:
            return "ERROR";

        default:
            return "UNKNOWN";
    }
}


/* ============================================================
 * RELÉ SERVICE MODE
 * ============================================================ */

static void set_service_mode(
    bool enabled
)
{
    gpio_set_level(
        RELAY_SERVICE_GPIO,
        enabled ?
            RELAY_ON :
            RELAY_OFF
    );

    printf(
        "[RELAY] Service Mode -> %s\n",
        enabled ?
            "ON" :
            "OFF"
    );
}


/* ============================================================
 * RELÉ HOLD DOOR
 * ============================================================ */

static void set_door_hold(
    bool enabled
)
{
    gpio_set_level(
        RELAY_DOOR_HOLD_GPIO,
        enabled ?
            RELAY_ON :
            RELAY_OFF
    );

    printf(
        "[RELAY] Door Hold -> %s\n",
        enabled ?
            "ON" :
            "OFF"
    );
}


/* ============================================================
 * GPIO DO RELÉ DO PISO
 * ============================================================ */

static gpio_num_t get_floor_relay_gpio(
    uint8_t floor_id
)
{
    for (
        size_t i = 0;
        i < floor_relay_count;
        i++
    )
    {
        if (
            floor_relays[i].floor_id ==
            floor_id
        )
        {
            return
                floor_relays[i].relay_gpio;
        }
    }

    return GPIO_NUM_NC;
}


/* ============================================================
 * PEDIR PULSO
 * ============================================================ */

static bool request_floor_pulse(
    uint8_t floor_id
)
{
    gpio_num_t gpio =
        get_floor_relay_gpio(
            floor_id
        );

    if (
        gpio ==
        GPIO_NUM_NC
    )
    {
        printf(
            "[RELAY] No physical relay configured "
            "for FLOOR %u.\n",
            floor_id
        );

        return false;
    }

    if (
        pending_floor_pulse !=
        0
    )
    {
        printf(
            "[RELAY] Another floor pulse is pending.\n"
        );

        return false;
    }

    pending_floor_pulse =
        floor_id;

    printf(
        "[RELAY] FLOOR %u pulse requested.\n",
        floor_id
    );

    return true;
}


/* ============================================================
 * EXECUTAR PULSO
 * ============================================================ */

static void process_pending_floor_pulse(void)
{
    uint8_t floor_id =
        pending_floor_pulse;

    if (
        floor_id ==
        0
    )
    {
        return;
    }

    pending_floor_pulse =
        0;

    gpio_num_t gpio =
        get_floor_relay_gpio(
            floor_id
        );

    if (
        gpio ==
        GPIO_NUM_NC
    )
    {
        return;
    }

    console_begin_async_log();

    printf(
        "\n[RELAY] FLOOR %u -> ON\n",
        floor_id
    );

    gpio_set_level(
        gpio,
        RELAY_ON
    );

    vTaskDelay(
        pdMS_TO_TICKS(
            RELAY_PULSE_MS
        )
    );

    gpio_set_level(
        gpio,
        RELAY_OFF
    );

    printf(
        "[RELAY] FLOOR %u -> OFF\n",
        floor_id
    );

    console_end_async_log();
}


/* ============================================================
 * INIT RELÉS
 * ============================================================ */

static void init_relays(void)
{
    gpio_config_t config =
    {
        0
    };

    config.mode =
        GPIO_MODE_OUTPUT;

    config.pin_bit_mask =
        (
            1ULL <<
            RELAY_FLOOR_1_GPIO
        )
        |
        (
            1ULL <<
            RELAY_FLOOR_2_GPIO
        )
        |
        (
            1ULL <<
            RELAY_SERVICE_GPIO
        )
        |
        (
            1ULL <<
            RELAY_DOOR_HOLD_GPIO
        );

    config.pull_up_en =
        GPIO_PULLUP_DISABLE;

    config.pull_down_en =
        GPIO_PULLDOWN_DISABLE;

    config.intr_type =
        GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(
        gpio_config(
            &config
        )
    );

    gpio_set_level(
        RELAY_FLOOR_1_GPIO,
        RELAY_OFF
    );

    gpio_set_level(
        RELAY_FLOOR_2_GPIO,
        RELAY_OFF
    );

    gpio_set_level(
        RELAY_SERVICE_GPIO,
        RELAY_OFF
    );

    gpio_set_level(
        RELAY_DOOR_HOLD_GPIO,
        RELAY_OFF
    );

    printf(
        "[CTRL] Relay configuration:\n"
    );

    printf(
        "  Service Mode -> GPIO %d\n",
        RELAY_SERVICE_GPIO
    );

    printf(
        "  Floor 1      -> GPIO %d\n",
        RELAY_FLOOR_1_GPIO
    );

    printf(
        "  Floor 2      -> GPIO %d\n",
        RELAY_FLOOR_2_GPIO
    );

    printf(
        "  Door Hold    -> GPIO %d\n",
        RELAY_DOOR_HOLD_GPIO
    );
}


/* ============================================================
 * PISO REGISTADO?
 * ============================================================ */

static bool floor_is_registered(
    uint8_t floor_id
)
{
    if (
        floor_id == 0 ||
        floor_id > MAX_FLOORS
    )
    {
        return false;
    }

    return
        floor_peers[
            floor_id
        ].registered;
}


/* ============================================================
 * ADICIONAR ESP-NOW PEER
 * ============================================================ */

static bool add_espnow_peer(
    const uint8_t *mac
)
{
    if (
        mac ==
        NULL
    )
    {
        return false;
    }

    if (
        esp_now_is_peer_exist(
            mac
        )
    )
    {
        return true;
    }

    esp_now_peer_info_t peer =
    {
        0
    };

    memcpy(
        peer.peer_addr,
        mac,
        ESP_NOW_ETH_ALEN
    );

    peer.channel =
        ESPNOW_CHANNEL;

    peer.encrypt =
        false;

    esp_err_t err =
        esp_now_add_peer(
            &peer
        );

    if (
        err !=
        ESP_OK
    )
    {
        printf(
            "[ESP-NOW] Failed to add peer. err=%d\n",
            err
        );

        return false;
    }

    return true;
}


/* ============================================================
 * REGISTAR FLOOR NODE
 * ============================================================ */

static bool register_floor_node(
    uint8_t floor_id,
    const uint8_t *mac
)
{
    if (
        floor_id == 0 ||
        floor_id > MAX_FLOORS ||
        mac == NULL
    )
    {
        printf(
            "[ESP-NOW] Invalid FLOOR ID %u.\n",
            floor_id
        );

        return false;
    }

    if (
        !add_espnow_peer(
            mac
        )
    )
    {
        return false;
    }

    floor_peer_t *peer =
        &floor_peers[
            floor_id
        ];

    peer->registered =
        true;

    peer->floor_id =
        floor_id;

    memcpy(
        peer->mac,
        mac,
        ESP_NOW_ETH_ALEN
    );

    printf(
        "[ESP-NOW] Registered FLOOR %u -> ",
        floor_id
    );

    print_mac(
        mac
    );

    printf("\n");

    return true;
}


/* ============================================================
 * VALIDAR MAC
 * ============================================================ */

static bool floor_mac_matches(
    uint8_t floor_id,
    const uint8_t *mac
)
{
    if (
        !floor_is_registered(
            floor_id
        ) ||
        mac == NULL
    )
    {
        return false;
    }

    return
        memcmp(
            floor_peers[
                floor_id
            ].mac,
            mac,
            ESP_NOW_ETH_ALEN
        ) == 0;
}


/* ============================================================
 * ENVIAR MENSAGEM
 * ============================================================ */

static bool send_controller_message(
    uint8_t target_floor,
    espnow_message_type_t type,
    uint8_t origin_floor,
    uint8_t destination_floor
)
{
    if (
        !floor_is_registered(
            target_floor
        )
    )
    {
        printf(
            "[ESP-NOW] FLOOR %u is not registered.\n",
            target_floor
        );

        return false;
    }

    espnow_message_t message =
    {
        .type =
            type,

        .sender_floor =
            CONTROLLER_FLOOR_ID,

        .target_floor =
            target_floor,

        .origin_floor =
            origin_floor,

        .destination_floor =
            destination_floor
    };

    esp_err_t err =
        esp_now_send(
            floor_peers[
                target_floor
            ].mac,

            (const uint8_t *)&message,

            sizeof(message)
        );

    if (
        err !=
        ESP_OK
    )
    {
        printf(
            "[ESP-NOW] Failed TX %s -> FLOOR %u "
            "(err=%d)\n",
            message_type_to_string(
                type
            ),
            target_floor,
            err
        );

        return false;
    }

    printf(
        "[ESP-NOW] TX %-18s "
        "target=%u origin=%u destination=%u\n",
        message_type_to_string(
            type
        ),
        target_floor,
        origin_floor,
        destination_floor
    );

    return true;
}


/* ============================================================
 * RESET MISSÃO
 * ============================================================ */

static void reset_mission(void)
{
    current_mission.active =
        false;

    current_mission.origin_floor =
        0;

    current_mission.destination_floor =
        0;

    current_mission.state =
        MISSION_IDLE;

    pending_floor_pulse =
        0;
}


/* ============================================================
 * TERMINAR MISSÃO
 * ============================================================ */

static void finish_mission(void)
{
    if (
        !current_mission.active
    )
    {
        return;
    }

    uint8_t origin =
        current_mission.origin_floor;

    uint8_t destination =
        current_mission.destination_floor;

    printf(
        "\n"
        "========================================\n"
    );

    printf(
        "[MISSION] COMPLETE %u -> %u\n",
        origin,
        destination
    );

    printf(
        "========================================\n"
    );

    set_door_hold(
        false
    );

    set_service_mode(
        false
    );

    send_controller_message(
        origin,
        MSG_RESET_NODE,
        origin,
        destination
    );

    if (
        destination !=
        origin
    )
    {
        send_controller_message(
            destination,
            MSG_RESET_NODE,
            origin,
            destination
        );
    }

    current_mission.state =
        MISSION_COMPLETE;

    reset_mission();
}


/* ============================================================
 * INICIAR MISSÃO
 * ============================================================ */

static bool start_mission(
    uint8_t origin,
    uint8_t destination
)
{
    if (
        current_mission.active
    )
    {
        printf(
            "[MISSION] Another mission is already active.\n"
        );

        return false;
    }

    if (
        origin == 0 ||
        destination == 0 ||
        origin > MAX_FLOORS ||
        destination > MAX_FLOORS
    )
    {
        printf(
            "[MISSION] Invalid mission: %u -> %u\n",
            origin,
            destination
        );

        return false;
    }

    if (
        origin ==
        destination
    )
    {
        printf(
            "[MISSION] Origin and destination "
            "cannot be the same.\n"
        );

        return false;
    }

    if (
        !floor_is_registered(
            origin
        )
    )
    {
        printf(
            "[MISSION] Origin FLOOR %u is offline.\n",
            origin
        );

        return false;
    }

    if (
        !floor_is_registered(
            destination
        )
    )
    {
        printf(
            "[MISSION] Destination FLOOR %u is offline.\n",
            destination
        );

        return false;
    }

    if (
        get_floor_relay_gpio(
            origin
        ) ==
        GPIO_NUM_NC
    )
    {
        printf(
            "[MISSION] No relay configured for FLOOR %u.\n",
            origin
        );

        return false;
    }

    if (
        get_floor_relay_gpio(
            destination
        ) ==
        GPIO_NUM_NC
    )
    {
        printf(
            "[MISSION] No relay configured for FLOOR %u.\n",
            destination
        );

        return false;
    }

    current_mission.active =
        true;

    current_mission.origin_floor =
        origin;

    current_mission.destination_floor =
        destination;

    current_mission.state =
        MISSION_WAITING_ORIGIN_DOOR;

    printf(
        "\n"
        "========================================\n"
    );

    printf(
        "[MISSION] START %u -> %u\n",
        origin,
        destination
    );

    printf(
        "========================================\n"
    );

    if (
        !send_controller_message(
            origin,
            MSG_START_ORIGIN,
            origin,
            destination
        )
    )
    {
        printf(
            "[MISSION] Failed to configure origin.\n"
        );

        reset_mission();

        return false;
    }

    if (
        !send_controller_message(
            destination,
            MSG_START_DESTINATION,
            origin,
            destination
        )
    )
    {
        printf(
            "[MISSION] Failed to configure destination.\n"
        );

        send_controller_message(
            origin,
            MSG_RESET_NODE,
            origin,
            destination
        );

        reset_mission();

        return false;
    }

    set_service_mode(
        true
    );

    if (
        !request_floor_pulse(
            origin
        )
    )
    {
        printf(
            "[MISSION] Failed to call FLOOR %u.\n",
            origin
        );

        set_service_mode(
            false
        );

        send_controller_message(
            origin,
            MSG_RESET_NODE,
            origin,
            destination
        );

        send_controller_message(
            destination,
            MSG_RESET_NODE,
            origin,
            destination
        );

        reset_mission();

        return false;
    }

    printf(
        "[MISSION] Waiting for elevator at "
        "origin FLOOR %u.\n",
        origin
    );

    return true;
}


/* ============================================================
 * NODE READY
 * ============================================================ */

static void handle_node_ready(
    const espnow_message_t *message,
    const uint8_t *source_mac
)
{
    uint8_t floor_id =
        message->sender_floor;

    if (
        floor_id == 0 ||
        floor_id > MAX_FLOORS
    )
    {
        printf(
            "[ESP-NOW] Invalid NODE_READY floor=%u\n",
            floor_id
        );

        return;
    }

    if (
        !register_floor_node(
            floor_id,
            source_mac
        )
    )
    {
        return;
    }

    send_controller_message(
        floor_id,
        MSG_CONTROLLER_READY,
        0,
        0
    );

    printf(
        "[CTRL] FLOOR %u ONLINE\n",
        floor_id
    );
}


/* ============================================================
 * MENSAGENS DOS FLOOR NODES
 * ============================================================ */

static void handle_floor_message(
    const espnow_message_t *message
)
{
    if (
        message ==
        NULL
    )
    {
        return;
    }

    uint8_t sender =
        message->sender_floor;

    switch (
        message->type
    )
    {
        /* ====================================================
         * REQUEST ELEVATOR
         * ==================================================== */

        case MSG_REQUEST_ELEVATOR:
        {
            printf(
                "[CTRL] FLOOR %u detected robot and "
                "requested elevator.\n",
                sender
            );

            if (
                current_mission.active
            )
            {
                printf(
                    "[MISSION] Request ignored: "
                    "mission already active.\n"
                );

                return;
            }

            /*
             * Compatibilidade automática atual:
             *
             * 1 -> 2
             * 2 -> 1
             *
             * Posteriormente a CLI / sistema externo
             * deverá definir explicitamente o destino.
             */
            uint8_t destination = 0;

            if (
                sender ==
                1
            )
            {
                destination = 2;
            }

            else if (
                sender ==
                2
            )
            {
                destination = 1;
            }

            else
            {
                printf(
                    "[MISSION] No automatic destination "
                    "configured for FLOOR %u.\n",
                    sender
                );

                return;
            }

            start_mission(
                sender,
                destination
            );

            return;
        }


        /* ====================================================
         * HOLD DOOR
         * ==================================================== */

        case MSG_HOLD_DOOR_OPEN:
        {
            if (
                !current_mission.active
            )
            {
                printf(
                    "[MISSION] HOLD ignored: "
                    "no active mission.\n"
                );

                return;
            }

            if (
                sender ==
                    current_mission.origin_floor &&
                current_mission.state ==
                    MISSION_WAITING_ORIGIN_DOOR
            )
            {
                printf(
                    "[MISSION] Origin FLOOR %u "
                    "door OPEN.\n",
                    sender
                );

                set_door_hold(
                    true
                );

                current_mission.state =
                    MISSION_AT_ORIGIN;

                return;
            }

            if (
                sender ==
                    current_mission.destination_floor &&
                (
                    current_mission.state ==
                        MISSION_TRAVELLING_TO_DESTINATION ||
                    current_mission.state ==
                        MISSION_WAITING_DESTINATION_DOOR
                )
            )
            {
                printf(
                    "[MISSION] Destination FLOOR %u "
                    "door OPEN.\n",
                    sender
                );

                set_door_hold(
                    true
                );

                current_mission.state =
                    MISSION_AT_DESTINATION;

                return;
            }

            printf(
                "[MISSION] Unexpected HOLD from FLOOR %u "
                "state=%s\n",
                sender,
                mission_state_to_string(
                    current_mission.state
                )
            );

            return;
        }


        /* ====================================================
         * RELEASE DOOR
         * ==================================================== */

        case MSG_RELEASE_DOOR:
        {
            if (
                !current_mission.active
            )
            {
                set_door_hold(
                    false
                );

                printf(
                    "[MISSION] RELEASE received "
                    "without active mission.\n"
                );

                return;
            }

            if (
                sender ==
                    current_mission.origin_floor &&
                current_mission.state ==
                    MISSION_AT_ORIGIN
            )
            {
                printf(
                    "[MISSION] Origin FLOOR %u "
                    "released door.\n",
                    sender
                );

                set_door_hold(
                    false
                );

                current_mission.state =
                    MISSION_WAITING_ORIGIN_DOOR_CLOSED;

                printf(
                    "[MISSION] Waiting for origin "
                    "Reed Switch confirmation.\n"
                );

                return;
            }

            if (
                sender ==
                    current_mission.destination_floor &&
                current_mission.state ==
                    MISSION_AT_DESTINATION
            )
            {
                printf(
                    "[MISSION] Destination FLOOR %u "
                    "released door.\n",
                    sender
                );

                set_door_hold(
                    false
                );

                current_mission.state =
                    MISSION_WAITING_DESTINATION_DOOR_CLOSED;

                printf(
                    "[MISSION] Waiting for destination "
                    "Reed Switch confirmation.\n"
                );

                return;
            }

            printf(
                "[MISSION] Unexpected RELEASE from FLOOR %u "
                "state=%s\n",
                sender,
                mission_state_to_string(
                    current_mission.state
                )
            );

            set_door_hold(
                false
            );

            return;
        }


        /* ====================================================
         * DOOR CLOSED
         * ==================================================== */

        case MSG_DOOR_CLOSED:
        {
            if (
                !current_mission.active
            )
            {
                printf(
                    "[MISSION] DOOR_CLOSED ignored: "
                    "no active mission.\n"
                );

                return;
            }

            /*
             * ORIGEM
             */
            if (
                sender ==
                    current_mission.origin_floor &&
                current_mission.state ==
                    MISSION_WAITING_ORIGIN_DOOR_CLOSED
            )
            {
                printf(
                    "[MISSION] Origin FLOOR %u "
                    "door CLOSED.\n",
                    sender
                );

                printf(
                    "[MISSION] Selecting destination "
                    "FLOOR %u.\n",
                    current_mission.destination_floor
                );

                current_mission.state =
                    MISSION_TRAVELLING_TO_DESTINATION;

                if (
                    !request_floor_pulse(
                        current_mission.destination_floor
                    )
                )
                {
                    printf(
                        "[MISSION] Failed to select "
                        "destination.\n"
                    );

                    current_mission.state =
                        MISSION_ERROR;

                    return;
                }

                current_mission.state =
                    MISSION_WAITING_DESTINATION_DOOR;

                printf(
                    "[MISSION] Elevator travelling "
                    "%u -> %u.\n",
                    current_mission.origin_floor,
                    current_mission.destination_floor
                );

                return;
            }

            /*
             * DESTINO
             */
            if (
                sender ==
                    current_mission.destination_floor &&
                current_mission.state ==
                    MISSION_WAITING_DESTINATION_DOOR_CLOSED
            )
            {
                printf(
                    "[MISSION] Destination FLOOR %u "
                    "door CLOSED.\n",
                    sender
                );

                finish_mission();

                return;
            }

            printf(
                "[MISSION] Unexpected DOOR_CLOSED "
                "from FLOOR %u state=%s\n",
                sender,
                mission_state_to_string(
                    current_mission.state
                )
            );

            return;
        }


        default:
        {
            printf(
                "[CTRL] Unsupported message: %s\n",
                message_type_to_string(
                    message->type
                )
            );

            return;
        }
    }
}


/* ============================================================
 * ESP-NOW RX
 * ============================================================ */

static void on_data_recv(
    const esp_now_recv_info_t *recv_info,
    const uint8_t *data,
    int len
)
{
    if (
        recv_info == NULL ||
        data == NULL
    )
    {
        return;
    }

    console_begin_async_log();

    printf("\n");

    if (
        len !=
        sizeof(espnow_message_t)
    )
    {
        printf(
            "[ESP-NOW] Invalid packet size=%d expected=%u\n",
            len,
            (unsigned int)
                sizeof(espnow_message_t)
        );

        console_end_async_log();

        return;
    }

    espnow_message_t message =
    {
        0
    };

    memcpy(
        &message,
        data,
        sizeof(message)
    );

    printf(
        "[ESP-NOW] RX %-18s "
        "sender=%u target=%u origin=%u destination=%u\n",
        message_type_to_string(
            message.type
        ),
        message.sender_floor,
        message.target_floor,
        message.origin_floor,
        message.destination_floor
    );

    printf(
        "[ESP-NOW] Source MAC: "
    );

    print_mac(
        recv_info->src_addr
    );

    printf("\n");

    if (
        message.target_floor !=
        CONTROLLER_FLOOR_ID
    )
    {
        printf(
            "[ESP-NOW] Ignored: target=%u\n",
            message.target_floor
        );

        console_end_async_log();

        return;
    }

    if (
        message.type ==
        MSG_NODE_READY
    )
    {
        handle_node_ready(
            &message,
            recv_info->src_addr
        );

        console_end_async_log();

        return;
    }

    if (
        !floor_is_registered(
            message.sender_floor
        )
    )
    {
        printf(
            "[ESP-NOW] FLOOR %u not registered.\n",
            message.sender_floor
        );

        console_end_async_log();

        return;
    }

    if (
        !floor_mac_matches(
            message.sender_floor,
            recv_info->src_addr
        )
    )
    {
        printf(
            "[ESP-NOW] MAC mismatch for FLOOR %u.\n",
            message.sender_floor
        );

        console_end_async_log();

        return;
    }

    handle_floor_message(
        &message
    );

    console_end_async_log();
}


/* ============================================================
 * ESP-NOW TX
 * ============================================================ */

static void on_data_sent(
    const esp_now_send_info_t *info,
    esp_now_send_status_t status
)
{
    (void)info;

    console_begin_async_log();

    printf(
        "\n[ESP-NOW] TX status: %s\n",
        status ==
            ESP_NOW_SEND_SUCCESS
            ?
            "SUCCESS"
            :
            "FAIL"
    );

    console_end_async_log();
}


/* ============================================================
 * STATUS
 * ============================================================ */

static void print_status(void)
{
    printf(
        "\n"
        "========================================\n"
        "ARES STATUS\n"
        "========================================\n"
    );

    printf(
        "Registered Floor Nodes:\n"
    );

    bool found =
        false;

    for (
        uint8_t floor = 1;
        floor <= MAX_FLOORS;
        floor++
    )
    {
        if (
            floor_peers[
                floor
            ].registered
        )
        {
            found =
                true;

            printf(
                "  FLOOR %u -> ",
                floor
            );

            print_mac(
                floor_peers[
                    floor
                ].mac
            );

            printf("\n");
        }
    }

    if (
        !found
    )
    {
        printf(
            "  None\n"
        );
    }

    printf(
        "\nMission:\n"
    );

    if (
        current_mission.active
    )
    {
        printf(
            "  Route: %u -> %u\n",
            current_mission.origin_floor,
            current_mission.destination_floor
        );

        printf(
            "  State: %s\n",
            mission_state_to_string(
                current_mission.state
            )
        );
    }

    else
    {
        printf(
            "  No active mission.\n"
        );
    }

    printf(
        "========================================\n"
    );
}


/* ============================================================
 * ABORT
 * ============================================================ */

static void abort_mission(void)
{
    if (
        !current_mission.active
    )
    {
        printf(
            "[MISSION] No active mission.\n"
        );

        return;
    }

    uint8_t origin =
        current_mission.origin_floor;

    uint8_t destination =
        current_mission.destination_floor;

    printf(
        "[MISSION] Aborting %u -> %u.\n",
        origin,
        destination
    );

    set_door_hold(
        false
    );

    set_service_mode(
        false
    );

    send_controller_message(
        origin,
        MSG_RESET_NODE,
        origin,
        destination
    );

    if (
        destination !=
        origin
    )
    {
        send_controller_message(
            destination,
            MSG_RESET_NODE,
            origin,
            destination
        );
    }

    reset_mission();

    printf(
        "[MISSION] Controller -> IDLE\n"
    );
}


/* ============================================================
 * AJUDA
 * ============================================================ */

static void print_help(void)
{
    printf(
        "\n"
        "Commands:\n"
        "  mission <origin> <destination>\n"
        "  status\n"
        "  abort\n"
        "  help\n"
    );
}


/* ============================================================
 * PROCESSAR COMANDO CLI
 * ============================================================ */

static void process_cli_command(
    const char *command
)
{
    if (
        command ==
        NULL ||
        strlen(command) ==
        0
    )
    {
        return;
    }

    unsigned int origin = 0;
    unsigned int destination = 0;

    if (
        sscanf(
            command,
            "mission %u %u",
            &origin,
            &destination
        ) ==
        2
    )
    {
        if (
            origin == 0 ||
            destination == 0 ||
            origin > UINT8_MAX ||
            destination > UINT8_MAX
        )
        {
            printf(
                "[CLI] Invalid floor number.\n"
            );

            return;
        }

        start_mission(
            (uint8_t)origin,
            (uint8_t)destination
        );

        return;
    }

    if (
        strcmp(
            command,
            "status"
        ) ==
        0
    )
    {
        print_status();

        return;
    }

    if (
        strcmp(
            command,
            "abort"
        ) ==
        0
    )
    {
        abort_mission();

        return;
    }

    if (
        strcmp(
            command,
            "help"
        ) ==
        0
    )
    {
        print_help();

        return;
    }

    printf(
        "[CLI] Unknown command: %s\n",
        command
    );
}


/* ============================================================
 * CLI TASK
 * ============================================================ */

static void cli_task(
    void *parameter
)
{
    (void)parameter;

    printf(
        "\n"
        "[CLI] Commands:\n"
        "  mission <origin> <destination>\n"
        "  status\n"
        "  abort\n"
        "  help\n\n"
    );

    cli_started =
        true;

    memset(
        cli_buffer,
        0,
        sizeof(cli_buffer)
    );

    cli_index =
        0;

    cli_redraw_prompt();

    while (1)
    {
        int ch =
            getchar();

        if (
            ch ==
            EOF
        )
        {
            vTaskDelay(
                pdMS_TO_TICKS(
                    20
                )
            );

            continue;
        }


        /* ====================================================
         * ENTER
         * ==================================================== */

        if (
            ch == '\r' ||
            ch == '\n'
        )
        {
            /*
             * Algumas consolas enviam CR+LF.
             *
             * Se buffer vazio, ignoramos.
             */
            if (
                cli_index ==
                0
            )
            {
                continue;
            }

            cli_buffer[
                cli_index
            ] = '\0';

            /*
             * Limpar linha atual e mostrar
             * o comando executado.
             */
            printf(
                "\r\033[2KARES> %s\n",
                cli_buffer
            );

            process_cli_command(
                cli_buffer
            );

            cli_index =
                0;

            memset(
                cli_buffer,
                0,
                sizeof(cli_buffer)
            );

            cli_redraw_prompt();

            continue;
        }


        /* ====================================================
         * BACKSPACE
         * ==================================================== */

        if (
            ch == 8 ||
            ch == 127
        )
        {
            if (
                cli_index >
                0
            )
            {
                cli_index--;

                cli_buffer[
                    cli_index
                ] = '\0';

                cli_redraw_prompt();
            }

            continue;
        }


        /* ====================================================
         * CARÁCTER NORMAL
         * ==================================================== */

        if (
            ch >= 32 &&
            ch <= 126
        )
        {
            if (
                cli_index <
                CLI_BUFFER_SIZE - 1
            )
            {
                cli_buffer[
                    cli_index
                ] =
                    (char)ch;

                cli_index++;

                cli_buffer[
                    cli_index
                ] = '\0';

                cli_redraw_prompt();
            }
        }
    }
}


/* ============================================================
 * APP MAIN
 * ============================================================ */

void app_main(void)
{
    /* ========================================================
     * NVS
     * ======================================================== */

    esp_err_t err =
        nvs_flash_init();

    if (
        err ==
            ESP_ERR_NVS_NO_FREE_PAGES ||
        err ==
            ESP_ERR_NVS_NEW_VERSION_FOUND
    )
    {
        ESP_ERROR_CHECK(
            nvs_flash_erase()
        );

        ESP_ERROR_CHECK(
            nvs_flash_init()
        );
    }

    else
    {
        ESP_ERROR_CHECK(
            err
        );
    }


    /* ========================================================
     * NETWORK
     * ======================================================== */

    ESP_ERROR_CHECK(
        esp_netif_init()
    );

    ESP_ERROR_CHECK(
        esp_event_loop_create_default()
    );


    /* ========================================================
     * WIFI
     * ======================================================== */

    wifi_init_config_t wifi_config =
        WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(
        esp_wifi_init(
            &wifi_config
        )
    );

    ESP_ERROR_CHECK(
        esp_wifi_set_mode(
            WIFI_MODE_STA
        )
    );

    ESP_ERROR_CHECK(
        esp_wifi_start()
    );

    ESP_ERROR_CHECK(
        esp_wifi_set_channel(
            ESPNOW_CHANNEL,
            WIFI_SECOND_CHAN_NONE
        )
    );


    /* ========================================================
     * MAC
     * ======================================================== */

    uint8_t controller_mac[
        ESP_NOW_ETH_ALEN
    ] =
    {
        0
    };

    ESP_ERROR_CHECK(
        esp_read_mac(
            controller_mac,
            ESP_MAC_WIFI_STA
        )
    );

    printf(
        "\n"
        "====================================\n"
        "ARES MULTI-FLOOR CONTROLLER\n"
        "Controller MAC: "
    );

    print_mac(
        controller_mac
    );

    printf(
        "\n"
        "====================================\n\n"
    );


    /* ========================================================
     * RELÉS
     * ======================================================== */

    init_relays();


    /* ========================================================
     * ESP-NOW
     * ======================================================== */

    ESP_ERROR_CHECK(
        esp_now_init()
    );

    ESP_ERROR_CHECK(
        esp_now_register_recv_cb(
            on_data_recv
        )
    );

    ESP_ERROR_CHECK(
        esp_now_register_send_cb(
            on_data_sent
        )
    );


    /* ========================================================
     * ESTADO INICIAL
     * ======================================================== */

    reset_mission();

    set_service_mode(
        false
    );

    set_door_hold(
        false
    );

    printf(
        "\n"
        "[CTRL] Waiting for Floor Nodes...\n"
    );


    /* ========================================================
     * CLI
     * ======================================================== */

    BaseType_t task_result =
        xTaskCreate(
            cli_task,
            "ares_cli",
            4096,
            NULL,
            5,
            NULL
        );

    if (
        task_result !=
        pdPASS
    )
    {
        printf(
            "[CLI] Failed to create task.\n"
        );
    }


    /* ========================================================
     * LOOP
     * ======================================================== */

    while (1)
    {
        process_pending_floor_pulse();

        vTaskDelay(
            pdMS_TO_TICKS(
                20
            )
        );
    }
}
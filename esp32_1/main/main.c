#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_event.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

#include "esp_mac.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "sensor.h"
#include "reed_switch.h"
#include "espnow_protocol.h"


/* ============================================================
 * ARES - GENERIC FLOOR NODE
 * Multi-Floor Architecture
 * ============================================================
 *
 * Este ficheiro é igual para todos os pisos.
 *
 * Alterar apenas:
 *
 *      #define FLOOR_ID X
 *
 * Exemplos:
 *
 * Piso 1:
 *      #define FLOOR_ID 1
 *
 * Piso 2:
 *      #define FLOOR_ID 2
 *
 * Piso 3:
 *      #define FLOOR_ID 3
 *
 * O papel do piso não está fixo no firmware.
 *
 * O Controller pode atribuir dinamicamente:
 *
 *      ORIGIN
 *      DESTINATION
 *      IDLE
 *
 * ============================================================ */


/* ============================================================
 * CONFIGURAÇÃO
 * ============================================================ */

#define FLOOR_ID                    1

#define ESPNOW_CHANNEL              1

#define FSM_TICK_MS                 100

#define BLE_CONFIRM_MS              10000
#define DISTANCE_CONFIRM_MS         5000

#define ENTRY_HOLD_MS               5000
#define EXIT_HOLD_MS                5000

#define DOOR_OPEN_TIMEOUT_MS        30000
#define DOOR_CLOSE_TIMEOUT_MS       12000

#define BLE_ENTER_RSSI              55
#define BLE_EXIT_RSSI               60

#define BLE_CLOSE_CONFIRM_SAMPLES   2
#define BLE_FAR_CONFIRM_SAMPLES     2

#define BLE_LOST_TIMEOUT_MS         4000

#define BLE_RSSI_STABILITY_WINDOW   10

#define HANDSHAKE_RETRY_MS          3000


/* ============================================================
 * MAC DO CONTROLLER
 *
 * Controller:
 * 30:76:F5:F8:4D:7C
 * ============================================================ */

static const uint8_t controller_mac[ESP_NOW_ETH_ALEN] =
{
    0x30,
    0x76,
    0xF5,
    0xF8,
    0x4D,
    0x7C
};


/* ============================================================
 * BLE BEACON DO ROBÔ
 *
 * Beacon:
 * 7C:D9:F4:08:D5:85
 * ============================================================ */

static const uint8_t target_ble_mac[6] =
{
    0x7C,
    0xD9,
    0xF4,
    0x08,
    0xD5,
    0x85
};


/* ============================================================
 * PAPEL DO FLOOR NODE
 * ============================================================ */

typedef enum
{
    FLOOR_ROLE_IDLE = 0,
    FLOOR_ROLE_ORIGIN,
    FLOOR_ROLE_DESTINATION

} floor_role_t;


/* ============================================================
 * ESTADOS DA FSM LOCAL
 * ============================================================ */

typedef enum
{
    FLOOR_STATE_IDLE = 0,

    /*
     * Deteção inicial do robô.
     */
    FLOOR_STATE_CONFIRM_DISTANCE,

    /*
     * Pedido enviado ao Controller.
     */
    FLOOR_STATE_WAIT_ASSIGNMENT,

    /*
     * ORIGIN
     */
    FLOOR_STATE_ORIGIN_WAIT_DOOR_OPEN,

    FLOOR_STATE_ORIGIN_WAIT_ROBOT_ENTER,

    FLOOR_STATE_ORIGIN_WAIT_DOOR_CLOSED,

    FLOOR_STATE_ORIGIN_WAIT_RESET,

    /*
     * DESTINATION
     */
    FLOOR_STATE_DEST_WAIT_DOOR_OPEN,

    FLOOR_STATE_DEST_WAIT_ROBOT_DETECTED,

    FLOOR_STATE_DEST_WAIT_ROBOT_CLEAR,

    FLOOR_STATE_DEST_WAIT_DOOR_CLOSED,

    FLOOR_STATE_DEST_WAIT_RESET,

    /*
     * Segurança
     */
    FLOOR_STATE_ERROR

} floor_state_t;


/* ============================================================
 * ESTADO BLE
 * ============================================================ */

typedef enum
{
    BLE_PROX_UNKNOWN = 0,
    BLE_PROX_CLOSE,
    BLE_PROX_FAR

} ble_proximity_t;


/* ============================================================
 * MISSÃO LOCAL
 * ============================================================ */

typedef struct
{
    bool active;

    floor_role_t role;

    uint8_t origin_floor;

    uint8_t destination_floor;

} floor_mission_t;


/* ============================================================
 * VARIÁVEIS GLOBAIS
 * ============================================================ */

static floor_mission_t current_mission =
{
    .active = false,
    .role = FLOOR_ROLE_IDLE,
    .origin_floor = 0,
    .destination_floor = 0
};


static volatile bool controller_ready = false;

static volatile bool start_origin_requested = false;

static volatile bool start_destination_requested = false;

static volatile bool reset_node_requested = false;


/*
 * Estado atual da FSM.
 */
static floor_state_t state =
    FLOOR_STATE_IDLE;


/*
 * BLE.
 */
static ble_proximity_t ble_proximity =
    BLE_PROX_FAR;


static int8_t last_ble_rssi = -127;

static float filtered_ble_rssi = 0.0f;

static bool ble_rssi_filter_initialized = false;

static int64_t last_ble_adv_time_ms = 0;


static float filtered_rssi_samples[
    BLE_RSSI_STABILITY_WINDOW
] = {0};


static uint8_t filtered_rssi_sample_count = 0;

static uint8_t filtered_rssi_sample_index = 0;


/*
 * Timers.
 */
static uint32_t ble_close_time_ms = 0;

static uint32_t distance_confirm_ms = 0;

static uint32_t door_open_wait_ms = 0;

static uint32_t door_close_wait_ms = 0;

static uint32_t entry_hold_ms = 0;

static uint32_t exit_hold_ms = 0;


/*
 * Passagem do robô.
 */
static bool robot_entry_seen = false;

static bool robot_exit_seen = false;

static bool exit_clear_timer_started = false;


/*
 * ESP-NOW handshake.
 */
static int64_t last_handshake_ms = 0;


/*
 * Logs BLE detalhados.
 */
static bool ble_prints_enabled = false;


/* ============================================================
 * UTILITÁRIOS
 * ============================================================ */

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}


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

static const char *state_to_string(
    floor_state_t local_state
)
{
    switch (local_state)
    {
        case FLOOR_STATE_IDLE:
            return "IDLE";

        case FLOOR_STATE_CONFIRM_DISTANCE:
            return "CONFIRM_DISTANCE";

        case FLOOR_STATE_WAIT_ASSIGNMENT:
            return "WAIT_ASSIGNMENT";

        case FLOOR_STATE_ORIGIN_WAIT_DOOR_OPEN:
            return "ORIGIN_WAIT_DOOR_OPEN";

        case FLOOR_STATE_ORIGIN_WAIT_ROBOT_ENTER:
            return "ORIGIN_WAIT_ROBOT_ENTER";

        case FLOOR_STATE_ORIGIN_WAIT_DOOR_CLOSED:
            return "ORIGIN_WAIT_DOOR_CLOSED";

        case FLOOR_STATE_ORIGIN_WAIT_RESET:
            return "ORIGIN_WAIT_RESET";

        case FLOOR_STATE_DEST_WAIT_DOOR_OPEN:
            return "DEST_WAIT_DOOR_OPEN";

        case FLOOR_STATE_DEST_WAIT_ROBOT_DETECTED:
            return "DEST_WAIT_ROBOT_DETECTED";

        case FLOOR_STATE_DEST_WAIT_ROBOT_CLEAR:
            return "DEST_WAIT_ROBOT_CLEAR";

        case FLOOR_STATE_DEST_WAIT_DOOR_CLOSED:
            return "DEST_WAIT_DOOR_CLOSED";

        case FLOOR_STATE_DEST_WAIT_RESET:
            return "DEST_WAIT_RESET";

        case FLOOR_STATE_ERROR:
            return "ERROR";

        default:
            return "UNKNOWN";
    }
}


/* ============================================================
 * SENSORES
 * ============================================================ */

static void init_sensor_pins(void)
{
    init_proximity_sensor();

    init_reed_switch();

    printf(
        "[GPIO] Sensors initialized: Reed=%d Distance=%d\n",
        REED_PIN,
        SENSOR_PIN
    );
}


/*
 * Sensor de distância:
 *
 * true  = objeto detetado
 * false = caminho livre
 */
static bool robot_detected(void)
{
    return is_robot_detected();
}


/*
 * Reed Switch:
 *
 * magnet present = porta fechada
 *
 * Portanto:
 *
 * !is_magnet_present() = porta aberta
 */
static bool door_open(void)
{
    return !is_magnet_present();
}


/* ============================================================
 * BLE MAC
 * ============================================================ */

static bool ble_addr_matches(
    const uint8_t *addr,
    const uint8_t *target
)
{
    if (
        addr == NULL ||
        target == NULL
    )
    {
        return false;
    }


    /*
     * Ordem normal.
     */
    if (
        memcmp(
            addr,
            target,
            6
        ) == 0
    )
    {
        return true;
    }


    /*
     * Ordem inversa.
     */
    for (
        uint8_t i = 0;
        i < 6;
        i++
    )
    {
        if (
            addr[i] !=
            target[5 - i]
        )
        {
            return false;
        }
    }


    return true;
}


/* ============================================================
 * RESET FILTRO BLE
 * ============================================================ */

static void reset_ble_filter(void)
{
    ble_rssi_filter_initialized = false;

    filtered_ble_rssi = 0.0f;

    filtered_rssi_sample_count = 0;

    filtered_rssi_sample_index = 0;


    memset(
        filtered_rssi_samples,
        0,
        sizeof(filtered_rssi_samples)
    );
}


/* ============================================================
 * ESTABILIDADE RSSI
 * ============================================================ */

static void update_rssi_stability(
    float rssi,
    float *mean,
    float *stddev
)
{
    if (
        mean == NULL ||
        stddev == NULL
    )
    {
        return;
    }


    filtered_rssi_samples[
        filtered_rssi_sample_index
    ] = rssi;


    filtered_rssi_sample_index =
        (
            filtered_rssi_sample_index + 1
        ) %
        BLE_RSSI_STABILITY_WINDOW;


    if (
        filtered_rssi_sample_count <
        BLE_RSSI_STABILITY_WINDOW
    )
    {
        filtered_rssi_sample_count++;
    }


    *mean = 0.0f;


    for (
        uint8_t i = 0;
        i < filtered_rssi_sample_count;
        i++
    )
    {
        *mean +=
            filtered_rssi_samples[i];
    }


    *mean /=
        filtered_rssi_sample_count;


    float variance = 0.0f;


    for (
        uint8_t i = 0;
        i < filtered_rssi_sample_count;
        i++
    )
    {
        float difference =
            filtered_rssi_samples[i] -
            *mean;


        variance +=
            difference *
            difference;
    }


    *stddev =
        sqrtf(
            variance /
            filtered_rssi_sample_count
        );
}


/* ============================================================
 * PROXIMIDADE BLE
 * ============================================================ */

static void update_ble_proximity(
    float rssi,
    float stddev
)
{
    static uint8_t close_count = 0;

    static uint8_t far_count = 0;


    /*
     * Estamos a trabalhar com magnitude positiva:
     *
     * -50 dBm -> 50
     * -75 dBm -> 75
     *
     * Menor = mais próximo.
     */
    if (
        rssi <=
        BLE_ENTER_RSSI
    )
    {
        if (
            close_count <
            BLE_CLOSE_CONFIRM_SAMPLES
        )
        {
            close_count++;
        }

        far_count = 0;
    }

    else if (
        rssi >=
        BLE_EXIT_RSSI
    )
    {
        if (
            far_count <
            BLE_FAR_CONFIRM_SAMPLES
        )
        {
            far_count++;
        }

        close_count = 0;
    }

    else
    {
        close_count = 0;
        far_count = 0;
    }


    if (
        ble_proximity !=
            BLE_PROX_CLOSE &&
        close_count >=
            BLE_CLOSE_CONFIRM_SAMPLES
    )
    {
        ble_proximity =
            BLE_PROX_CLOSE;


        printf(
            "[BLE] Proximity -> CLOSE "
            "(RSSI=%.1f stddev=%.2f)\n",
            rssi,
            stddev
        );


        close_count = 0;

        far_count = 0;
    }


    else if (
        ble_proximity !=
            BLE_PROX_FAR &&
        far_count >=
            BLE_FAR_CONFIRM_SAMPLES
    )
    {
        ble_proximity =
            BLE_PROX_FAR;


        printf(
            "[BLE] Proximity -> FAR "
            "(RSSI=%.1f stddev=%.2f)\n",
            rssi,
            stddev
        );


        close_count = 0;

        far_count = 0;
    }
}


/* ============================================================
 * BLE TIMEOUT
 * ============================================================ */

static void check_ble_timeout(void)
{
    if (
        last_ble_adv_time_ms ==
        0
    )
    {
        return;
    }


    if (
        (
            now_ms() -
            last_ble_adv_time_ms
        ) >=
        BLE_LOST_TIMEOUT_MS
    )
    {
        /*
         * Mantemos a última proximidade confirmada.
         * Apenas reiniciamos o filtro RSSI.
         */
        reset_ble_filter();


        last_ble_adv_time_ms = 0;


        if (
            ble_prints_enabled
        )
        {
            printf(
                "[BLE] Beacon timeout. "
                "Keeping previous proximity state.\n"
            );
        }
    }
}


/* ============================================================
 * BLE CALLBACK
 * ============================================================ */

static void gap_event_handler(
    esp_gap_ble_cb_event_t event,
    esp_ble_gap_cb_param_t *param
)
{
    switch (event)
    {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        {
            if (
                param->scan_param_cmpl.status ==
                ESP_BT_STATUS_SUCCESS
            )
            {
                esp_err_t err =
                    esp_ble_gap_start_scanning(
                        0
                    );


                if (
                    err !=
                    ESP_OK
                )
                {
                    printf(
                        "[BLE] Failed to start scan. err=%d\n",
                        err
                    );
                }
            }

            else
            {
                printf(
                    "[BLE] Scan params failed. status=%d\n",
                    param->scan_param_cmpl.status
                );
            }


            break;
        }


        case ESP_GAP_BLE_SCAN_RESULT_EVT:
        {
            if (
                param->scan_rst.search_evt !=
                ESP_GAP_SEARCH_INQ_RES_EVT
            )
            {
                break;
            }


            if (
                !ble_addr_matches(
                    param->scan_rst.bda,
                    target_ble_mac
                )
            )
            {
                break;
            }


            last_ble_rssi =
                param->scan_rst.rssi;


            last_ble_adv_time_ms =
                now_ms();


            float current_rssi =
                (
                    last_ble_rssi < 0
                )
                ?
                -(float)last_ble_rssi
                :
                (float)last_ble_rssi;


            if (
                !ble_rssi_filter_initialized
            )
            {
                filtered_ble_rssi =
                    current_rssi;


                ble_rssi_filter_initialized =
                    true;
            }

            else
            {
                filtered_ble_rssi =
                    (
                        0.65f *
                        filtered_ble_rssi
                    )
                    +
                    (
                        0.35f *
                        current_rssi
                    );
            }


            float mean_rssi = 0.0f;

            float stddev_rssi = 0.0f;


            update_rssi_stability(
                filtered_ble_rssi,
                &mean_rssi,
                &stddev_rssi
            );


            update_ble_proximity(
                filtered_ble_rssi,
                stddev_rssi
            );


            if (
                ble_prints_enabled
            )
            {
                printf(
                    "[BLE] MAC="
                );


                print_mac(
                    param->scan_rst.bda
                );


                printf(
                    " raw=%d filtered=%.1f "
                    "mean=%.1f stddev=%.2f\n",
                    last_ble_rssi,
                    filtered_ble_rssi,
                    mean_rssi,
                    stddev_rssi
                );
            }


            break;
        }


        default:
        {
            break;
        }
    }
}


/* ============================================================
 * INICIALIZAR BLE
 * ============================================================ */

static void start_ble(void)
{
    ESP_ERROR_CHECK(
        esp_bt_controller_mem_release(
            ESP_BT_MODE_CLASSIC_BT
        )
    );


    esp_bt_controller_config_t bt_config =
        BT_CONTROLLER_INIT_CONFIG_DEFAULT();


    bt_config.mode =
        ESP_BT_MODE_BLE;


    ESP_ERROR_CHECK(
        esp_bt_controller_init(
            &bt_config
        )
    );


    ESP_ERROR_CHECK(
        esp_bt_controller_enable(
            ESP_BT_MODE_BLE
        )
    );


    ESP_ERROR_CHECK(
        esp_bluedroid_init()
    );


    ESP_ERROR_CHECK(
        esp_bluedroid_enable()
    );


    ESP_ERROR_CHECK(
        esp_ble_gap_register_callback(
            gap_event_handler
        )
    );


    esp_ble_scan_params_t scan_params =
    {
        .scan_type =
            BLE_SCAN_TYPE_PASSIVE,

        .own_addr_type =
            BLE_ADDR_TYPE_PUBLIC,

        .scan_filter_policy =
            BLE_SCAN_FILTER_ALLOW_ALL,

        .scan_interval =
            0x50,

        .scan_window =
            0x30,

        .scan_duplicate =
            BLE_SCAN_DUPLICATE_DISABLE
    };


    ESP_ERROR_CHECK(
        esp_ble_gap_set_scan_params(
            &scan_params
        )
    );
}


/* ============================================================
 * ESP-NOW TX
 * ============================================================ */

static bool send_floor_message(
    espnow_message_type_t type
)
{
    espnow_message_t message =
    {
        .type =
            type,

        /*
         * Piso físico que está a enviar.
         */
        .sender_floor =
            FLOOR_ID,

        /*
         * 0 = Controller.
         */
        .target_floor =
            0,

        .origin_floor =
            current_mission.origin_floor,

        .destination_floor =
            current_mission.destination_floor
    };


    esp_err_t err =
        esp_now_send(
            controller_mac,
            (const uint8_t *)&message,
            sizeof(message)
        );


    if (
        err !=
        ESP_OK
    )
    {
        printf(
            "[ESP-NOW] TX failed: %s err=%d\n",
            message_type_to_string(
                type
            ),
            err
        );


        return false;
    }


    printf(
        "[ESP-NOW] TX %-18s "
        "sender=%u origin=%u destination=%u\n",
        message_type_to_string(
            type
        ),
        FLOOR_ID,
        current_mission.origin_floor,
        current_mission.destination_floor
    );


    return true;
}


/* ============================================================
 * NODE READY
 * ============================================================ */

static void send_node_ready(void)
{
    uint8_t old_origin =
        current_mission.origin_floor;


    uint8_t old_destination =
        current_mission.destination_floor;


    /*
     * NODE_READY não pertence a uma missão.
     */
    current_mission.origin_floor =
        0;


    current_mission.destination_floor =
        0;


    send_floor_message(
        MSG_NODE_READY
    );


    current_mission.origin_floor =
        old_origin;


    current_mission.destination_floor =
        old_destination;


    last_handshake_ms =
        now_ms();
}


/* ============================================================
 * RESET DA MISSÃO
 * ============================================================ */

static void reset_local_mission(void)
{
    current_mission.active =
        false;


    current_mission.role =
        FLOOR_ROLE_IDLE;


    current_mission.origin_floor =
        0;


    current_mission.destination_floor =
        0;


    start_origin_requested =
        false;


    start_destination_requested =
        false;


    reset_node_requested =
        false;


    ble_close_time_ms =
        0;


    distance_confirm_ms =
        0;


    door_open_wait_ms =
        0;


    door_close_wait_ms =
        0;


    entry_hold_ms =
        0;


    exit_hold_ms =
        0;


    robot_entry_seen =
        false;


    robot_exit_seen =
        false;


    exit_clear_timer_started =
        false;


    state =
        FLOOR_STATE_IDLE;


    printf(
        "[FLOOR %u] Mission reset -> IDLE\n",
        FLOOR_ID
    );
}


/* ============================================================
 * PROCESSAR MENSAGEM DO CONTROLLER
 * ============================================================ */

static bool process_controller_message(
    const espnow_message_t *message
)
{
    if (
        message ==
        NULL
    )
    {
        return false;
    }


    /*
     * Controller identifica-se como sender_floor 0.
     */
    if (
        message->sender_floor !=
        0
    )
    {
        printf(
            "[ESP-NOW] Invalid sender=%u\n",
            message->sender_floor
        );


        return false;
    }


    /*
     * A mensagem deve ser para este Floor Node.
     */
    if (
        message->target_floor !=
        FLOOR_ID
    )
    {
        return false;
    }


    printf(
        "[ESP-NOW] RX %-18s "
        "target=%u origin=%u destination=%u\n",
        message_type_to_string(
            message->type
        ),
        message->target_floor,
        message->origin_floor,
        message->destination_floor
    );


    switch (
        message->type
    )
    {
        case MSG_CONTROLLER_READY:
        {
            controller_ready =
                true;


            printf(
                "[FLOOR %u] Controller READY.\n",
                FLOOR_ID
            );


            return true;
        }


        case MSG_START_ORIGIN:
        {
            if (
                message->origin_floor !=
                FLOOR_ID
            )
            {
                printf(
                    "[FLOOR %u] Invalid START_ORIGIN "
                    "origin=%u\n",
                    FLOOR_ID,
                    message->origin_floor
                );


                return false;
            }


            current_mission.active =
                true;


            current_mission.role =
                FLOOR_ROLE_ORIGIN;


            current_mission.origin_floor =
                message->origin_floor;


            current_mission.destination_floor =
                message->destination_floor;


            start_origin_requested =
                true;


            printf(
                "[FLOOR %u] ROLE=ORIGIN "
                "mission %u -> %u\n",
                FLOOR_ID,
                current_mission.origin_floor,
                current_mission.destination_floor
            );


            return true;
        }


        case MSG_START_DESTINATION:
        {
            if (
                message->destination_floor !=
                FLOOR_ID
            )
            {
                printf(
                    "[FLOOR %u] Invalid START_DESTINATION "
                    "destination=%u\n",
                    FLOOR_ID,
                    message->destination_floor
                );


                return false;
            }


            current_mission.active =
                true;


            current_mission.role =
                FLOOR_ROLE_DESTINATION;


            current_mission.origin_floor =
                message->origin_floor;


            current_mission.destination_floor =
                message->destination_floor;


            start_destination_requested =
                true;


            printf(
                "[FLOOR %u] ROLE=DESTINATION "
                "mission %u -> %u\n",
                FLOOR_ID,
                current_mission.origin_floor,
                current_mission.destination_floor
            );


            return true;
        }


        case MSG_RESET_NODE:
        {
            printf(
                "[FLOOR %u] RESET_NODE received.\n",
                FLOOR_ID
            );


            reset_node_requested =
                true;


            return true;
        }


        case MSG_SERVICE_MODE_ON:
        {
            return true;
        }


        case MSG_SERVICE_MODE_OFF:
        {
            return true;
        }


        default:
        {
            printf(
                "[FLOOR %u] Unexpected message: %s\n",
                FLOOR_ID,
                message_type_to_string(
                    message->type
                )
            );


            return false;
        }
    }
}


/* ============================================================
 * ESP-NOW RX CALLBACK
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


    /*
     * Ignorar qualquer dispositivo que não seja
     * o Controller conhecido.
     */
    if (
        memcmp(
            recv_info->src_addr,
            controller_mac,
            ESP_NOW_ETH_ALEN
        ) != 0
    )
    {
        printf(
            "[ESP-NOW] Message ignored from unknown MAC: "
        );


        print_mac(
            recv_info->src_addr
        );


        printf("\n");


        return;
    }


    if (
        len !=
        sizeof(espnow_message_t)
    )
    {
        printf(
            "[ESP-NOW] Invalid message size: "
            "%d expected=%u\n",
            len,
            (unsigned int)
                sizeof(espnow_message_t)
        );


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


    process_controller_message(
        &message
    );
}


/* ============================================================
 * ESP-NOW TX CALLBACK
 * ============================================================ */

static void on_data_sent(
    const esp_now_send_info_t *info,
    esp_now_send_status_t status
)
{
    (void)info;


    if (
        ble_prints_enabled
    )
    {
        printf(
            "[ESP-NOW] TX status: %s\n",
            status ==
                ESP_NOW_SEND_SUCCESS
                ?
                "SUCCESS"
                :
                "FAIL"
        );
    }
}


/* ============================================================
 * INICIALIZAR ESP-NOW
 * ============================================================ */

static void init_espnow(void)
{
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


    esp_now_peer_info_t peer =
    {
        0
    };


    memcpy(
        peer.peer_addr,
        controller_mac,
        ESP_NOW_ETH_ALEN
    );


    peer.channel =
        ESPNOW_CHANNEL;


    peer.encrypt =
        false;


    if (
        !esp_now_is_peer_exist(
            controller_mac
        )
    )
    {
        ESP_ERROR_CHECK(
            esp_now_add_peer(
                &peer
            )
        );
    }
}


/* ============================================================
 * PROCESSAR AÇÕES PENDENTES
 * ============================================================ */

static void process_pending_controller_actions(void)
{
    if (
        reset_node_requested
    )
    {
        reset_local_mission();

        return;
    }


    /*
     * Este piso foi definido como ORIGIN.
     */
    if (
        start_origin_requested
    )
    {
        start_origin_requested =
            false;


        state =
            FLOOR_STATE_ORIGIN_WAIT_DOOR_OPEN;


        door_open_wait_ms =
            0;


        door_close_wait_ms =
            0;


        entry_hold_ms =
            0;


        robot_entry_seen =
            false;


        printf(
            "[FLOW] FLOOR %u -> ORIGIN flow started.\n",
            FLOOR_ID
        );
    }


    /*
     * Este piso foi definido como DESTINATION.
     */
    if (
        start_destination_requested
    )
    {
        start_destination_requested =
            false;


        state =
            FLOOR_STATE_DEST_WAIT_DOOR_OPEN;


        door_open_wait_ms =
            0;


        door_close_wait_ms =
            0;


        exit_hold_ms =
            0;


        robot_exit_seen =
            false;


        exit_clear_timer_started =
            false;


        printf(
            "[FLOW] FLOOR %u -> DESTINATION flow started.\n",
            FLOOR_ID
        );
    }
}


/* ============================================================
 * FSM DO FLOOR NODE
 * ============================================================ */

static void run_floor_fsm(void)
{
    static floor_state_t previous_state =
        FLOOR_STATE_ERROR;


    bool detected =
        robot_detected();


    bool is_door_open =
        door_open();


    /*
     * Mostrar transições apenas quando mudam.
     */
    if (
        state !=
        previous_state
    )
    {
        printf(
            "[FLOW] State -> %s\n",
            state_to_string(
                state
            )
        );


        previous_state =
            state;
    }


    switch (
        state
    )
    {
        /* ====================================================
         * IDLE
         * ==================================================== */

        case FLOOR_STATE_IDLE:
        {
            /*
             * Qualquer piso sem missão ativa pode detetar
             * a chegada do robô e tornar-se origem.
             */
            if (
                !current_mission.active &&
                ble_proximity ==
                    BLE_PROX_CLOSE
            )
            {
                ble_close_time_ms +=
                    FSM_TICK_MS;


                if (
                    ble_close_time_ms >=
                    BLE_CONFIRM_MS
                )
                {
                    printf(
                        "[FLOW] BLE close confirmed "
                        "at FLOOR %u.\n",
                        FLOOR_ID
                    );


                    state =
                        FLOOR_STATE_CONFIRM_DISTANCE;


                    distance_confirm_ms =
                        0;
                }
            }

            else
            {
                ble_close_time_ms =
                    0;
            }


            break;
        }


        /* ====================================================
         * CONFIRMAR SENSOR DE DISTÂNCIA
         * ==================================================== */

        case FLOOR_STATE_CONFIRM_DISTANCE:
        {
            if (
                ble_proximity !=
                BLE_PROX_CLOSE
            )
            {
                printf(
                    "[FLOW] BLE no longer close. "
                    "Returning to IDLE.\n"
                );


                state =
                    FLOOR_STATE_IDLE;


                ble_close_time_ms =
                    0;


                distance_confirm_ms =
                    0;


                break;
            }


            if (
                detected
            )
            {
                distance_confirm_ms +=
                    FSM_TICK_MS;


                if (
                    distance_confirm_ms >=
                    DISTANCE_CONFIRM_MS
                )
                {
                    printf(
                        "[FLOW] Robot confirmed at FLOOR %u. "
                        "Requesting elevator.\n",
                        FLOOR_ID
                    );


                    send_floor_message(
                        MSG_REQUEST_ELEVATOR
                    );


                    state =
                        FLOOR_STATE_WAIT_ASSIGNMENT;


                    distance_confirm_ms =
                        0;
                }
            }

            else
            {
                distance_confirm_ms =
                    0;
            }


            break;
        }


        /* ====================================================
         * ESPERAR ATRIBUIÇÃO
         * ==================================================== */

        case FLOOR_STATE_WAIT_ASSIGNMENT:
        {
            /*
             * Espera START_ORIGIN.
             *
             * A missão é criada pelo Controller.
             */
            break;
        }


        /* ====================================================
         * ORIGIN - ESPERAR PORTA ABERTA
         * ==================================================== */

        case FLOOR_STATE_ORIGIN_WAIT_DOOR_OPEN:
        {
            if (
                is_door_open
            )
            {
                printf(
                    "[FLOW] Origin door opened. "
                    "Requesting HOLD.\n"
                );


                send_floor_message(
                    MSG_HOLD_DOOR_OPEN
                );


                state =
                    FLOOR_STATE_ORIGIN_WAIT_ROBOT_ENTER;


                door_open_wait_ms =
                    0;


                entry_hold_ms =
                    0;


                robot_entry_seen =
                    false;


                break;
            }


            door_open_wait_ms +=
                FSM_TICK_MS;


            if (
                door_open_wait_ms >=
                DOOR_OPEN_TIMEOUT_MS
            )
            {
                printf(
                    "[ERROR] Origin door open timeout.\n"
                );


                state =
                    FLOOR_STATE_ERROR;
            }


            break;
        }


        /* ====================================================
         * ORIGIN - ROBÔ ENTRA
         * ==================================================== */

        case FLOOR_STATE_ORIGIN_WAIT_ROBOT_ENTER:
        {
            /*
             * O robô cruza o sensor.
             */
            if (
                detected &&
                !robot_entry_seen
            )
            {
                robot_entry_seen =
                    true;


                entry_hold_ms =
                    0;


                printf(
                    "[FLOW] Robot detected entering elevator "
                    "at FLOOR %u.\n",
                    FLOOR_ID
                );
            }


            /*
             * Quando deixa de bloquear o sensor significa
             * que entrou completamente.
             */
            if (
                robot_entry_seen &&
                !detected
            )
            {
                if (
                    entry_hold_ms <
                    ENTRY_HOLD_MS
                )
                {
                    entry_hold_ms +=
                        FSM_TICK_MS;
                }


                if (
                    entry_hold_ms >=
                    ENTRY_HOLD_MS
                )
                {
                    printf(
                        "[FLOW] Robot entered elevator. "
                        "Releasing door hold.\n"
                    );


                    send_floor_message(
                        MSG_RELEASE_DOOR
                    );


                    printf(
                        "[FLOW] Waiting for origin door "
                        "to close.\n"
                    );


                    state =
                        FLOOR_STATE_ORIGIN_WAIT_DOOR_CLOSED;


                    door_close_wait_ms =
                        0;


                    entry_hold_ms =
                        0;
                }
            }


            break;
        }


        /* ====================================================
         * ORIGIN - ESPERAR REED CONFIRMAR PORTA FECHADA
         * ==================================================== */

        case FLOOR_STATE_ORIGIN_WAIT_DOOR_CLOSED:
        {
            if (
                !is_door_open
            )
            {
                printf(
                    "[FLOW] Origin door closed. "
                    "Sending DOOR_CLOSED.\n"
                );


                send_floor_message(
                    MSG_DOOR_CLOSED
                );


                state =
                    FLOOR_STATE_ORIGIN_WAIT_RESET;


                door_close_wait_ms =
                    0;


                break;
            }


            door_close_wait_ms +=
                FSM_TICK_MS;


            if (
                door_close_wait_ms >=
                DOOR_CLOSE_TIMEOUT_MS
            )
            {
                printf(
                    "[ERROR] Origin door close timeout.\n"
                );


                state =
                    FLOOR_STATE_ERROR;
            }


            break;
        }


        /* ====================================================
         * ORIGIN TERMINOU
         * ==================================================== */

        case FLOOR_STATE_ORIGIN_WAIT_RESET:
        {
            /*
             * A partir deste momento é o Controller
             * que seleciona o destino.
             *
             * Este Floor Node espera RESET_NODE.
             */
            break;
        }


        /* ====================================================
         * DESTINATION - ESPERAR PORTA ABERTA
         * ==================================================== */

        case FLOOR_STATE_DEST_WAIT_DOOR_OPEN:
        {
            if (
                is_door_open
            )
            {
                printf(
                    "[FLOW] Destination door opened. "
                    "Requesting HOLD.\n"
                );


                send_floor_message(
                    MSG_HOLD_DOOR_OPEN
                );


                state =
                    FLOOR_STATE_DEST_WAIT_ROBOT_DETECTED;


                door_open_wait_ms =
                    0;


                robot_exit_seen =
                    false;


                exit_hold_ms =
                    0;


                exit_clear_timer_started =
                    false;


                break;
            }


            door_open_wait_ms +=
                FSM_TICK_MS;


            if (
                door_open_wait_ms >=
                DOOR_OPEN_TIMEOUT_MS
            )
            {
                printf(
                    "[ERROR] Destination door open timeout.\n"
                );


                state =
                    FLOOR_STATE_ERROR;
            }


            break;
        }


        /* ====================================================
         * DESTINATION - DETETAR ROBÔ
         * ==================================================== */

        case FLOOR_STATE_DEST_WAIT_ROBOT_DETECTED:
        {
            if (
                detected
            )
            {
                robot_exit_seen =
                    true;


                printf(
                    "[FLOW] Robot detected leaving elevator "
                    "at FLOOR %u.\n",
                    FLOOR_ID
                );


                state =
                    FLOOR_STATE_DEST_WAIT_ROBOT_CLEAR;


                exit_hold_ms =
                    0;


                exit_clear_timer_started =
                    false;
            }


            break;
        }


        /* ====================================================
         * DESTINATION - ESPERAR ROBÔ LIMPAR SENSOR
         * ==================================================== */

        case FLOOR_STATE_DEST_WAIT_ROBOT_CLEAR:
        {
            if (
                robot_exit_seen &&
                !detected
            )
            {
                if (
                    !exit_clear_timer_started
                )
                {
                    exit_clear_timer_started =
                        true;


                    exit_hold_ms =
                        0;


                    printf(
                        "[FLOW] Exit path clear. "
                        "Holding door for another %d ms.\n",
                        EXIT_HOLD_MS
                    );
                }


                if (
                    exit_hold_ms <
                    EXIT_HOLD_MS
                )
                {
                    exit_hold_ms +=
                        FSM_TICK_MS;
                }


                if (
                    exit_hold_ms >=
                    EXIT_HOLD_MS
                )
                {
                    printf(
                        "[FLOW] Robot cleared destination "
                        "FLOOR %u. Releasing door.\n",
                        FLOOR_ID
                    );


                    send_floor_message(
                        MSG_RELEASE_DOOR
                    );


                    printf(
                        "[FLOW] Waiting for destination "
                        "door to close.\n"
                    );


                    state =
                        FLOOR_STATE_DEST_WAIT_DOOR_CLOSED;


                    door_close_wait_ms =
                        0;


                    exit_hold_ms =
                        0;


                    exit_clear_timer_started =
                        false;
                }
            }

            else if (
                detected
            )
            {
                exit_clear_timer_started =
                    false;


                exit_hold_ms =
                    0;
            }


            break;
        }


        /* ====================================================
         * DESTINATION - ESPERAR PORTA FECHADA
         * ==================================================== */

        case FLOOR_STATE_DEST_WAIT_DOOR_CLOSED:
        {
            if (
                !is_door_open
            )
            {
                printf(
                    "[FLOW] Destination door closed. "
                    "Sending DOOR_CLOSED.\n"
                );


                send_floor_message(
                    MSG_DOOR_CLOSED
                );


                state =
                    FLOOR_STATE_DEST_WAIT_RESET;


                door_close_wait_ms =
                    0;


                break;
            }


            door_close_wait_ms +=
                FSM_TICK_MS;


            if (
                door_close_wait_ms >=
                DOOR_CLOSE_TIMEOUT_MS
            )
            {
                printf(
                    "[ERROR] Destination door "
                    "close timeout.\n"
                );


                state =
                    FLOOR_STATE_ERROR;
            }


            break;
        }


        /* ====================================================
         * DESTINATION TERMINOU
         * ==================================================== */

        case FLOOR_STATE_DEST_WAIT_RESET:
        {
            /*
             * Esperar MSG_RESET_NODE.
             */
            break;
        }


        /* ====================================================
         * ERRO / SEGURANÇA
         * ==================================================== */

        case FLOOR_STATE_ERROR:
        {
            printf(
                "[ERROR] FLOOR %u entering safe state. "
                "Releasing door hold.\n",
                FLOOR_ID
            );


            send_floor_message(
                MSG_RELEASE_DOOR
            );


            /*
             * Não repetir RELEASE constantemente.
             *
             * Esperamos o Controller efetuar reset.
             */
            if (
                current_mission.role ==
                FLOOR_ROLE_ORIGIN
            )
            {
                state =
                    FLOOR_STATE_ORIGIN_WAIT_RESET;
            }

            else
            {
                state =
                    FLOOR_STATE_DEST_WAIT_RESET;
            }


            break;
        }


        default:
        {
            state =
                FLOOR_STATE_ERROR;


            break;
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
     * LOGS
     * ======================================================== */

    if (
        !ble_prints_enabled
    )
    {
        esp_log_level_set(
            "*",
            ESP_LOG_ERROR
        );
    }


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
     * MAC LOCAL
     * ======================================================== */

    uint8_t own_mac[
        ESP_NOW_ETH_ALEN
    ] =
    {
        0
    };


    ESP_ERROR_CHECK(
        esp_read_mac(
            own_mac,
            ESP_MAC_WIFI_STA
        )
    );


    printf(
        "\n"
        "========================================\n"
    );


    printf(
        "ARES MULTI-FLOOR NODE\n"
    );


    printf(
        "Floor ID: %u\n",
        FLOOR_ID
    );


    printf(
        "Node MAC: "
    );


    print_mac(
        own_mac
    );


    printf(
        "\nController MAC: "
    );


    print_mac(
        controller_mac
    );


    printf(
        "\nBeacon MAC: "
    );


    print_mac(
        target_ble_mac
    );


    printf(
        "\n"
        "========================================\n\n"
    );


    /* ========================================================
     * SENSORES
     * ======================================================== */

    init_sensor_pins();


    printf(
        "[SENSOR] Reed raw=%d\n",
        gpio_get_level(
            REED_PIN
        )
    );


    printf(
        "[SENSOR] Distance raw=%d\n",
        gpio_get_level(
            SENSOR_PIN
        )
    );


    /* ========================================================
     * ESP-NOW
     * ======================================================== */

    init_espnow();


    send_node_ready();


    /* ========================================================
     * BLE
     * ======================================================== */

    start_ble();


    printf(
        "[BLE] Waiting for beacon...\n"
    );


    printf(
        "[FLOW] FLOOR %u initialized.\n\n",
        FLOOR_ID
    );


    /* ========================================================
     * LOOP PRINCIPAL
     * ======================================================== */

    while (1)
    {
        /*
         * Atualizar estado BLE.
         */
        check_ble_timeout();


        /*
         * Repetir handshake enquanto o Controller
         * ainda não respondeu.
         */
        if (
            !controller_ready &&
            (
                now_ms() -
                last_handshake_ms
            ) >=
            HANDSHAKE_RETRY_MS
        )
        {
            printf(
                "[ESP-NOW] Retrying NODE_READY...\n"
            );


            send_node_ready();
        }


        /*
         * Processar comandos recebidos.
         */
        process_pending_controller_actions();


        /*
         * Executar FSM local.
         */
        run_floor_fsm();


        /*
         * Tick de 100 ms.
         */
        vTaskDelay(
            pdMS_TO_TICKS(
                FSM_TICK_MS
            )
        );
    }
}
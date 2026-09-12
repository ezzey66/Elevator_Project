#ifndef ESPNOW_PROTOCOL_H
#define ESPNOW_PROTOCOL_H

#include <stdint.h>

/*
 * Tipos de mensagem trocados entre
 * Controller e Floor Nodes.
 */
typedef enum
{
    MSG_UNKNOWN = 0,

    /* Floor Node -> Controller */
    MSG_NODE_READY,
    MSG_REQUEST_ELEVATOR,
    MSG_HOLD_DOOR_OPEN,
    MSG_RELEASE_DOOR,
    MSG_DOOR_CLOSED,

    /* Controller -> Floor Node */
    MSG_CONTROLLER_READY,
    MSG_START_ORIGIN,
    MSG_START_DESTINATION,
    MSG_RESET_NODE,

    /* Comandos globais */
    MSG_SERVICE_MODE_ON,
    MSG_SERVICE_MODE_OFF

} espnow_message_type_t;


/*
 * Mensagem genérica ARES.
 *
 * sender_floor:
 *      Piso físico que enviou a mensagem.
 *      Controller = 0.
 *
 * target_floor:
 *      Piso a que a mensagem se destina.
 *      0 = Controller / global.
 *
 * origin_floor:
 *      Piso de origem da missão.
 *
 * destination_floor:
 *      Piso de destino da missão.
 */
typedef struct
{
    espnow_message_type_t type;

    uint8_t sender_floor;
    uint8_t target_floor;

    uint8_t origin_floor;
    uint8_t destination_floor;

} espnow_message_t;

#endif
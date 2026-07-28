#include "sanpo_app.h"

#include "../config/sanpo_board_config.h"
#include "../host/sanpo_st_protocol.h"
#include "../motor/sanpo_motor_manager.h"
#include "usbd_cdc_if.h"

#include <string.h>

#define APP_CMD_BOARD_INFO     0x01U
#define APP_CMD_MOVE_JOINT     0x02U
#define APP_CMD_GET_STATE      0x03U
#define APP_CMD_STOP_ALL       0x04U
#define APP_CMD_HEARTBEAT      0x05U
#define APP_CMD_HOME           0x06U
#define APP_CMD_CLEAR_FAULT    0x07U
#define APP_CMD_STAGE_GROUP    0x08U
#define APP_CMD_EXECUTE_GROUP  0x09U
#define APP_CMD_GROUP_STATUS   0x0AU

#define APP_STATUS_OK          0x00U
#define APP_STATUS_BAD_FRAME   0x01U
#define APP_STATUS_BAD_PARAM   0x02U
#define APP_STATUS_CAN_ERROR   0x03U
#define APP_STATUS_OFFLINE     0x04U

typedef struct
{
    uint16_t length;
    uint8_t data[64];
} UsbPacket;

typedef struct
{
    uint8_t channel;
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
    uint32_t tick_ms;
} CanPacket;

typedef struct
{
    uint16_t length;
    uint8_t data[18];
} TxPacket;

static UsbPacket usb_ring[SANPO_USB_RING_LENGTH];
static volatile uint8_t usb_write_index;
static volatile uint8_t usb_read_index;

/* 两个CAN中断分别使用独立环形缓冲区，避免双中断生产者竞争。 */
static CanPacket can1_ring[SANPO_CAN_RING_LENGTH];
static volatile uint8_t can1_write_index;
static volatile uint8_t can1_read_index;
static CanPacket can2_ring[SANPO_CAN_RING_LENGTH];
static volatile uint8_t can2_write_index;
static volatile uint8_t can2_read_index;

static TxPacket tx_ring[SANPO_TX_RING_LENGTH];
static uint8_t tx_write_index;
static uint8_t tx_read_index;

static volatile uint8_t owns_can;
static uint32_t last_heartbeat_ms;

static int32_t ReadI32Le(const uint8_t *data)
{
    return (int32_t)((uint32_t)data[0] |
                     ((uint32_t)data[1] << 8) |
                     ((uint32_t)data[2] << 16) |
                     ((uint32_t)data[3] << 24));
}

static uint16_t ReadU16Le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void WriteI16Le(uint8_t *data, int16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)((uint16_t)value >> 8);
}

static void PushResponse(uint8_t command, uint8_t status,
                         const uint8_t *payload, uint8_t payload_length)
{
    SanpoStFrame frame = {0};
    TxPacket *packet;
    uint8_t next;

    if (payload_length > 6U) {
        return;
    }
    next = (uint8_t)((tx_write_index + 1U) % SANPO_TX_RING_LENGTH);
    if (next == tx_read_index) {
        return;
    }

    frame.channel = SANPO_APP_USB_CHANNEL;
    frame.can_id = SANPO_APP_RESPONSE_ID;
    frame.dlc = (uint8_t)(payload_length + 2U);
    frame.data[0] = (uint8_t)(command | 0x80U);
    frame.data[1] = status;
    if ((payload != NULL) && (payload_length > 0U)) {
        memcpy(&frame.data[2], payload, payload_length);
    }

    packet = &tx_ring[tx_write_index];
    packet->length = SanpoSt_Encode(&frame, packet->data,
                                    sizeof(packet->data));
    if (packet->length > 0U) {
        tx_write_index = next;
    }
}

static void ProcessCommand(const SanpoStFrame *frame)
{
    uint8_t command;
    int result;

    if ((frame == NULL) || (frame->dlc == 0U)) {
        return;
    }

    command = frame->data[0];
    last_heartbeat_ms = HAL_GetTick();
    owns_can = 1U;

    switch (command) {
    case APP_CMD_BOARD_INFO: {
        uint8_t info[6] = {
            SANPO_BOARD_ID,
            SANPO_ARM_ID,
            SANPO_PHYSICAL_CAN1,
            SANPO_PHYSICAL_CAN2,
            SANPO_JOINT_COUNT,
            1U /* 应用协议版本 */
        };
        PushResponse(command, APP_STATUS_OK, info, sizeof(info));
        break;
    }

    case APP_CMD_MOVE_JOINT:
        if (frame->dlc != 8U) {
            PushResponse(command, APP_STATUS_BAD_FRAME, NULL, 0U);
        } else {
            uint8_t joint = frame->data[1];
            float angle_deg = ReadI32Le(&frame->data[2]) * 0.01f;
            float speed_rpm = ReadU16Le(&frame->data[6]) * 0.01f;
            result = SanpoMotor_MoveJoint(joint, angle_deg, speed_rpm);
            PushResponse(command,
                         (result == 0) ? APP_STATUS_OK :
                         (result >= -3) ? APP_STATUS_BAD_PARAM :
                                          APP_STATUS_CAN_ERROR,
                         &joint, 1U);
        }
        break;

    case APP_CMD_GET_STATE:
        if (frame->dlc != 2U) {
            PushResponse(command, APP_STATUS_BAD_FRAME, NULL, 0U);
        } else {
            uint8_t joint = frame->data[1];
            const SanpoJointState *state = SanpoMotor_GetState(joint);
            uint8_t payload[6] = {0};
            if (state == NULL) {
                PushResponse(command, APP_STATUS_BAD_PARAM, NULL, 0U);
                break;
            }
            payload[0] = joint;
            WriteI16Le(&payload[1], (int16_t)(state->actual_deg * 100.0f));
            WriteI16Le(&payload[3], (int16_t)(state->speed_rpm * 100.0f));
            payload[5] = state->fault;
            PushResponse(command, state->online ? APP_STATUS_OK :
                                                   APP_STATUS_OFFLINE,
                         payload, sizeof(payload));
        }
        break;

    case APP_CMD_STOP_ALL:
        result = SanpoMotor_StopAll();
        PushResponse(command, (result == 0) ? APP_STATUS_OK :
                                             APP_STATUS_CAN_ERROR,
                     NULL, 0U);
        break;

    case APP_CMD_HEARTBEAT:
        PushResponse(command, APP_STATUS_OK, NULL, 0U);
        break;

    case APP_CMD_HOME:
        if (frame->dlc != 2U) {
            PushResponse(command, APP_STATUS_BAD_FRAME, NULL, 0U);
        } else {
            uint8_t joint = frame->data[1];
            result = SanpoMotor_Home(joint);
            PushResponse(command, (result == 0) ? APP_STATUS_OK :
                                                 APP_STATUS_CAN_ERROR,
                         &joint, 1U);
        }
        break;

    case APP_CMD_CLEAR_FAULT:
        if (frame->dlc != 2U) {
            PushResponse(command, APP_STATUS_BAD_FRAME, NULL, 0U);
        } else {
            uint8_t joint = frame->data[1];
            result = SanpoMotor_ClearFault(joint);
            PushResponse(command, (result == 0) ? APP_STATUS_OK :
                                                 APP_STATUS_CAN_ERROR,
                         &joint, 1U);
        }
        break;

    case APP_CMD_STAGE_GROUP:
        if (frame->dlc != 6U) {
            PushResponse(command, APP_STATUS_BAD_FRAME, NULL, 0U);
        } else {
            uint8_t joint = frame->data[1];
            float angle_deg = ReadI32Le(&frame->data[2]) * 0.01f;
            result = SanpoMotor_StageGroupJoint(joint, angle_deg);
            PushResponse(command,
                         (result == 0) ? APP_STATUS_OK :
                                         APP_STATUS_BAD_PARAM,
                         &joint, 1U);
        }
        break;

    case APP_CMD_EXECUTE_GROUP:
        if (frame->dlc != 3U) {
            PushResponse(command, APP_STATUS_BAD_FRAME, NULL, 0U);
        } else {
            uint16_t duration_ms = ReadU16Le(&frame->data[1]);
            result = SanpoMotor_ExecuteGroup(duration_ms);
            PushResponse(command,
                         (result == 0) ? APP_STATUS_OK :
                         (result == -4) ? APP_STATUS_OFFLINE :
                         (result >= -5) ? APP_STATUS_BAD_PARAM :
                                          APP_STATUS_CAN_ERROR,
                         NULL, 0U);
        }
        break;

    case APP_CMD_GROUP_STATUS: {
        const SanpoGroupState *state = SanpoMotor_GetGroupState();
        uint8_t payload[6] = {
            state->staged_mask,
            state->active_mask,
            state->done_mask,
            state->fault_mask,
            state->sequence,
            state->active
        };
        PushResponse(command, APP_STATUS_OK, payload, sizeof(payload));
        break;
    }

    default:
        PushResponse(command, APP_STATUS_BAD_PARAM, NULL, 0U);
        break;
    }
}

void SanpoApp_Init(void)
{
    usb_write_index = 0U;
    usb_read_index = 0U;
    can1_write_index = 0U;
    can1_read_index = 0U;
    can2_write_index = 0U;
    can2_read_index = 0U;
    tx_write_index = 0U;
    tx_read_index = 0U;
    owns_can = 0U;
    last_heartbeat_ms = HAL_GetTick();
    SanpoMotor_Init();
}

uint8_t SanpoApp_IsUsbFrame(const uint8_t *data, uint16_t length)
{
    return SanpoSt_IsApplicationFrame(data, length);
}

void SanpoApp_UsbPush(const uint8_t *data, uint16_t length)
{
    uint8_t next;

    if ((data == NULL) || (length == 0U) ||
        (length > sizeof(usb_ring[0].data))) {
        return;
    }
    next = (uint8_t)((usb_write_index + 1U) % SANPO_USB_RING_LENGTH);
    if (next == usb_read_index) {
        return;
    }
    usb_ring[usb_write_index].length = length;
    memcpy(usb_ring[usb_write_index].data, data, length);
    usb_write_index = next;
    owns_can = 1U;
}

uint8_t SanpoApp_OwnsCan(void)
{
    return owns_can;
}

static void PushCan(CAN_HandleTypeDef *hcan, uint32_t fifo,
                    CanPacket *ring, volatile uint8_t *write_index,
                    volatile uint8_t *read_index)
{
    CAN_RxHeaderTypeDef header;
    CanPacket packet = {0};
    uint8_t next;

    if (HAL_CAN_GetRxMessage(hcan, fifo, &header, packet.data) != HAL_OK) {
        return;
    }
    next = (uint8_t)((*write_index + 1U) % SANPO_CAN_RING_LENGTH);
    if (next == *read_index) {
        return;
    }
    packet.channel = (hcan->Instance == CAN1) ? 1U : 2U;
    packet.id = (uint16_t)header.StdId;
    packet.dlc = (uint8_t)header.DLC;
    packet.tick_ms = HAL_GetTick();
    ring[*write_index] = packet;
    *write_index = next;
}

void SanpoApp_CanRxFifo0(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1) {
        PushCan(hcan, CAN_RX_FIFO0, can1_ring,
                &can1_write_index, &can1_read_index);
    } else {
        PushCan(hcan, CAN_RX_FIFO0, can2_ring,
                &can2_write_index, &can2_read_index);
    }
}

void SanpoApp_CanRxFifo1(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1) {
        PushCan(hcan, CAN_RX_FIFO1, can1_ring,
                &can1_write_index, &can1_read_index);
    } else {
        PushCan(hcan, CAN_RX_FIFO1, can2_ring,
                &can2_write_index, &can2_read_index);
    }
}

static void ProcessCanRing(CanPacket *ring, volatile uint8_t *write_index,
                           volatile uint8_t *read_index)
{
    while (*read_index != *write_index) {
        CanPacket packet = ring[*read_index];
        *read_index = (uint8_t)((*read_index + 1U) %
                                SANPO_CAN_RING_LENGTH);
        SanpoMotor_OnCan(packet.channel, packet.id, packet.data,
                         packet.dlc, packet.tick_ms);
    }
}

void SanpoApp_Process(void)
{
    uint32_t now_ms = HAL_GetTick();

    while (usb_read_index != usb_write_index) {
        SanpoStFrame frame;
        UsbPacket packet = usb_ring[usb_read_index];
        usb_read_index = (uint8_t)((usb_read_index + 1U) %
                                   SANPO_USB_RING_LENGTH);
        if (SanpoSt_Decode(packet.data, packet.length, &frame) == 0) {
            ProcessCommand(&frame);
        }
    }

    ProcessCanRing(can1_ring, &can1_write_index, &can1_read_index);
    ProcessCanRing(can2_ring, &can2_write_index, &can2_read_index);

    if (owns_can != 0U) {
        SanpoMotor_Process(now_ms);
        if ((now_ms - last_heartbeat_ms) > SANPO_PC_TIMEOUT_MS) {
            (void)SanpoMotor_StopAll();
            owns_can = 0U;
        }
    }

    if (tx_read_index != tx_write_index) {
        TxPacket *packet = &tx_ring[tx_read_index];
        if (CDC_Transmit_FS(packet->data, packet->length) == USBD_OK) {
            tx_read_index = (uint8_t)((tx_read_index + 1U) %
                                      SANPO_TX_RING_LENGTH);
        }
    }
}

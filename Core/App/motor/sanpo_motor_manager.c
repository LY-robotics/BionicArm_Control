#include "sanpo_motor_manager.h"

#include "../config/sanpo_board_config.h"
#include "../config/sanpo_joint_config.h"
#include "can.h"

#include <string.h>

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static SanpoJointState joint_state[SANPO_JOINT_COUNT];
static float group_target_deg[SANPO_JOINT_COUNT];
static SanpoGroupState group_state;
static uint8_t poll_index;
static uint32_t next_poll_ms;

static int32_t DegreesToCount(float degrees)
{
    return (int32_t)(degrees * 16384.0f / 360.0f);
}

static void WriteI32Le(uint8_t *data, int32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

static CAN_HandleTypeDef *GetCan(uint8_t channel)
{
    if (channel == 1U) {
        return &hcan1;
    }
    if (channel == 2U) {
        return &hcan2;
    }
    return NULL;
}

static int SendCan(uint8_t channel, uint16_t id,
                   const uint8_t *data, uint8_t length)
{
    CAN_HandleTypeDef *hcan = GetCan(channel);
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;

    if ((hcan == NULL) || (data == NULL) || (length > 8U) ||
        (id > 0x7FFU)) {
        return -1;
    }

    header.StdId = id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = length;
    header.TransmitGlobalTime = DISABLE;
    return (HAL_CAN_AddTxMessage(hcan, &header, (uint8_t *)data, &mailbox) ==
            HAL_OK) ? 0 : -2;
}

static const SanpoJointConfig *GetConfig(uint8_t joint_id)
{
    if ((joint_id == 0U) || (joint_id > SANPO_JOINT_COUNT)) {
        return NULL;
    }
    return &g_sanpo_joints[joint_id - 1U];
}

void SanpoMotor_Init(void)
{
    memset(joint_state, 0, sizeof(joint_state));
    memset(group_target_deg, 0, sizeof(group_target_deg));
    memset(&group_state, 0, sizeof(group_state));
    for (uint8_t index = 0U; index < SANPO_JOINT_COUNT; ++index) {
        joint_state[index].mode = SANPO_MOTOR_OFFLINE;
    }
    poll_index = 0U;
    next_poll_ms = HAL_GetTick() + SANPO_STATE_POLL_MS;
}

int SanpoMotor_StageGroupJoint(uint8_t joint_id, float angle_deg)
{
    const SanpoJointConfig *config = GetConfig(joint_id);

    if (config == NULL) {
        return -1;
    }
    if ((angle_deg < config->min_deg) || (angle_deg > config->max_deg)) {
        return -2;
    }
    if (group_state.active != 0U) {
        return -3;
    }

    group_target_deg[joint_id - 1U] = angle_deg;
    group_state.staged_mask |= (uint8_t)(1U << (joint_id - 1U));
    return 0;
}

int SanpoMotor_ExecuteGroup(uint16_t duration_ms)
{
    const uint8_t all_mask = (uint8_t)((1U << SANPO_JOINT_COUNT) - 1U);
    float command_rpm[SANPO_JOINT_COUNT] = {0.0f};
    uint8_t moving_mask = 0U;

    if ((duration_ms < SANPO_GROUP_MIN_DURATION_MS) ||
        (duration_ms > SANPO_GROUP_MAX_DURATION_MS)) {
        return -1;
    }
    if (group_state.staged_mask != all_mask) {
        return -2;
    }
    if (group_state.active != 0U) {
        return -3;
    }

    /* 先完成全部校验，避免部分关节已经运动后才发现另一轴不合法。 */
    for (uint8_t index = 0U; index < SANPO_JOINT_COUNT; ++index) {
        const SanpoJointConfig *config = &g_sanpo_joints[index];
        float delta;
        float rpm;

        if ((joint_state[index].online == 0U) ||
            (joint_state[index].fault != 0U)) {
            return -4;
        }
        delta = group_target_deg[index] - joint_state[index].actual_deg;
        if (delta < 0.0f) {
            delta = -delta;
        }
        if (delta < 0.01f) {
            continue;
        }

        rpm = delta * config->gear_ratio * 60000.0f /
              (360.0f * (float)duration_ms);
        if (rpm < SANPO_GROUP_MIN_MOTOR_RPM) {
            rpm = SANPO_GROUP_MIN_MOTOR_RPM;
        }
        if (rpm > config->max_rpm) {
            return -5;
        }
        command_rpm[index] = rpm;
        moving_mask |= (uint8_t)(1U << index);
    }

    group_state.sequence++;
    group_state.done_mask = (uint8_t)(all_mask & (uint8_t)(~moving_mask));
    group_state.fault_mask = 0U;
    group_state.active_mask = moving_mask;
    group_state.active = (moving_mask != 0U) ? 1U : 0U;

    for (uint8_t index = 0U; index < SANPO_JOINT_COUNT; ++index) {
        if ((moving_mask & (uint8_t)(1U << index)) == 0U) {
            continue;
        }
        if (SanpoMotor_MoveJoint((uint8_t)(index + 1U),
                                 group_target_deg[index],
                                 command_rpm[index]) != 0) {
            (void)SanpoMotor_StopAll();
            group_state.active = 0U;
            return -6;
        }
    }

    group_state.staged_mask = 0U;
    return 0;
}

const SanpoGroupState *SanpoMotor_GetGroupState(void)
{
    return &group_state;
}

int SanpoMotor_MoveJoint(uint8_t joint_id, float angle_deg, float speed_rpm)
{
    const SanpoJointConfig *config = GetConfig(joint_id);
    float motor_angle;
    uint8_t speed[5] = {0xB2U};
    uint8_t current[5] = {0xB3U};
    uint8_t position[5] = {0xC2U};

    if (config == NULL) {
        return -1;
    }
    if ((angle_deg < config->min_deg) || (angle_deg > config->max_deg)) {
        return -2;
    }
    if ((speed_rpm <= 0.0f) || (speed_rpm > config->max_rpm)) {
        return -3;
    }

    motor_angle = (angle_deg - config->zero_deg) *
                  (float)config->direction * config->gear_ratio;
    WriteI32Le(&speed[1], (int32_t)(speed_rpm * 100.0f));
    WriteI32Le(&current[1], (int32_t)(config->max_current_a * 1000.0f));
    WriteI32Le(&position[1], DegreesToCount(motor_angle));

    if (SendCan(config->can_channel, config->motor_id,
                speed, sizeof(speed)) != 0) {
        return -4;
    }
    if (SendCan(config->can_channel, config->motor_id,
                current, sizeof(current)) != 0) {
        return -5;
    }
    if (SendCan(config->can_channel, config->motor_id,
                position, sizeof(position)) != 0) {
        return -6;
    }

    joint_state[joint_id - 1U].target_deg = angle_deg;
    joint_state[joint_id - 1U].done = 0U;
    joint_state[joint_id - 1U].mode = SANPO_MOTOR_MOVING;
    return 0;
}

int SanpoMotor_StopAll(void)
{
    uint8_t disable = 0xCFU;
    int result = 0;

    for (uint8_t index = 0U; index < SANPO_JOINT_COUNT; ++index) {
        if (SendCan(g_sanpo_joints[index].can_channel,
                    g_sanpo_joints[index].motor_id, &disable, 1U) != 0) {
            result = -1;
        }
        joint_state[index].mode = SANPO_MOTOR_DISABLED;
    }
    group_state.active = 0U;
    group_state.active_mask = 0U;
    return result;
}

int SanpoMotor_Home(uint8_t joint_id)
{
    const SanpoJointConfig *config = GetConfig(joint_id);
    uint8_t command = 0xC4U;

    return (config == NULL) ? -1 :
           SendCan(config->can_channel, config->motor_id, &command, 1U);
}

int SanpoMotor_ClearFault(uint8_t joint_id)
{
    const SanpoJointConfig *config = GetConfig(joint_id);
    uint8_t command = 0xAFU;

    return (config == NULL) ? -1 :
           SendCan(config->can_channel, config->motor_id, &command, 1U);
}

const SanpoJointState *SanpoMotor_GetState(uint8_t joint_id)
{
    return ((joint_id > 0U) && (joint_id <= SANPO_JOINT_COUNT)) ?
           &joint_state[joint_id - 1U] : NULL;
}

void SanpoMotor_OnCan(uint8_t channel, uint16_t id, const uint8_t *data,
                      uint8_t length, uint32_t now_ms)
{
    for (uint8_t index = 0U; index < SANPO_JOINT_COUNT; ++index) {
        const SanpoJointConfig *config = &g_sanpo_joints[index];
        if ((config->can_channel != channel) || (config->motor_id != id)) {
            continue;
        }

        joint_state[index].online = 1U;
        joint_state[index].last_rx_ms = now_ms;

        if ((length >= 8U) && (data[0] == 0xA4U)) {
            int16_t current_raw = (int16_t)((uint16_t)data[2] |
                                             ((uint16_t)data[3] << 8));
            int16_t speed_raw = (int16_t)((uint16_t)data[4] |
                                           ((uint16_t)data[5] << 8));
            uint16_t angle_raw = (uint16_t)data[6] |
                                 ((uint16_t)data[7] << 8);
            float error;

            joint_state[index].current_a = current_raw * 0.001f;
            joint_state[index].speed_rpm = speed_raw * 0.01f;
            joint_state[index].actual_deg =
                ((angle_raw * 360.0f / 16384.0f) / config->gear_ratio) *
                (float)config->direction + config->zero_deg;

            error = joint_state[index].target_deg -
                    joint_state[index].actual_deg;
            if (error < 0.0f) {
                error = -error;
            }
            if ((joint_state[index].mode == SANPO_MOTOR_MOVING) &&
                (error < 0.5f) &&
                (joint_state[index].speed_rpm < 0.5f) &&
                (joint_state[index].speed_rpm > -0.5f)) {
                joint_state[index].done = 1U;
                joint_state[index].mode = SANPO_MOTOR_READY;
            }
        } else if ((length >= 8U) && (data[0] == 0xAEU)) {
            joint_state[index].fault = data[7];
            if (data[7] != 0U) {
                joint_state[index].mode = SANPO_MOTOR_FAULT;
            }
        }
        return;
    }
}

void SanpoMotor_Process(uint32_t now_ms)
{
    uint8_t query = 0xA4U;
    const uint8_t all_mask = (uint8_t)((1U << SANPO_JOINT_COUNT) - 1U);

    for (uint8_t index = 0U; index < SANPO_JOINT_COUNT; ++index) {
        if ((joint_state[index].online != 0U) &&
            ((now_ms - joint_state[index].last_rx_ms) >
             SANPO_MOTOR_TIMEOUT_MS)) {
            joint_state[index].online = 0U;
            joint_state[index].mode = SANPO_MOTOR_OFFLINE;
        }
        if (group_state.active != 0U) {
            uint8_t bit = (uint8_t)(1U << index);
            if ((joint_state[index].fault != 0U) ||
                (joint_state[index].online == 0U)) {
                group_state.fault_mask |= bit;
            } else if (joint_state[index].done != 0U) {
                group_state.done_mask |= bit;
                group_state.active_mask &= (uint8_t)(~bit);
            }
        }
    }

    if (group_state.active != 0U) {
        if (group_state.fault_mask != 0U) {
            (void)SanpoMotor_StopAll();
        } else if (group_state.done_mask == all_mask) {
            group_state.active = 0U;
            group_state.active_mask = 0U;
        }
    }

    if ((int32_t)(now_ms - next_poll_ms) >= 0) {
        const SanpoJointConfig *config = &g_sanpo_joints[poll_index];
        (void)SendCan(config->can_channel, config->motor_id, &query, 1U);
        poll_index = (uint8_t)((poll_index + 1U) % SANPO_JOINT_COUNT);
        next_poll_ms = now_ms + SANPO_STATE_POLL_MS;
    }
}

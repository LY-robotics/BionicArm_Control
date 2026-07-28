#ifndef SANPO_JOINT_CONFIG_H
#define SANPO_JOINT_CONFIG_H

#include "sanpo_board_config.h"

typedef struct
{
    uint8_t joint_id;
    uint8_t motor_id;
    uint8_t can_channel; /* STM32本地CAN：1或2 */
    int8_t direction;    /* 1或-1 */
    float gear_ratio;
    float zero_deg;
    float min_deg;
    float max_deg;
    float max_rpm;
    float max_current_a;
} SanpoJointConfig;

/*
 * 参考值来自用户提供的机械臂SDK。
 * CAN通道、方向、零点和最大电流必须逐轴实测后修改。
 */
static const SanpoJointConfig g_sanpo_joints[SANPO_JOINT_COUNT] =
{
    {1U, 51U, 1U, 1, 3.0f, 0.0f, -110.0f, 110.0f, 5.0f, 0.5f},
    {2U, 56U, 1U, 1, 3.0f, 0.0f, -170.0f, 120.0f, 5.0f, 0.5f},
    {3U, 16U, 1U, 1, 4.0f, 0.0f, -180.0f, 180.0f, 5.0f, 0.5f},
    {4U, 11U, 1U, 1, 4.2f, 0.0f, -90.0f, 150.0f, 5.0f, 0.5f},
    {5U, 22U, 1U, 1, 1.0f, 0.0f, -120.0f, 120.0f, 5.0f, 0.5f}
};

#endif

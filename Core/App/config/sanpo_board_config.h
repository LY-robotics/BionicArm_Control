#ifndef SANPO_BOARD_CONFIG_H
#define SANPO_BOARD_CONFIG_H

#include <stdint.h>

/*
 * 默认生成芯片1固件。
 * 编译芯片2前，将1改成2；不需要修改CubeIDE预处理宏。
 */
#define SANPO_BUILD_TARGET_MCU  1U

#if SANPO_BUILD_TARGET_MCU == 1U
#define SANPO_BOARD_ID        1U
#define SANPO_ARM_ID          1U
#define SANPO_PHYSICAL_CAN1   1U
#define SANPO_PHYSICAL_CAN2   2U
#elif SANPO_BUILD_TARGET_MCU == 2U
#define SANPO_BOARD_ID        2U
#define SANPO_ARM_ID          2U
#define SANPO_PHYSICAL_CAN1   3U
#define SANPO_PHYSICAL_CAN2   4U
#else
#error "SANPO_BUILD_TARGET_MCU must be 1 or 2"
#endif

/* ST帧中供板级应用使用的保留ID，使用前必须确认总线中没有同ID设备。 */
#define SANPO_APP_REQUEST_ID       0x7F0U
#define SANPO_APP_RESPONSE_ID      0x7F1U
#define SANPO_APP_USB_CHANNEL      0xFEU

#define SANPO_JOINT_COUNT          5U
#define SANPO_USB_RING_LENGTH      4U
#define SANPO_CAN_RING_LENGTH      16U
#define SANPO_TX_RING_LENGTH       8U

#define SANPO_CONTROL_PERIOD_MS    10U
#define SANPO_STATE_POLL_MS        20U
#define SANPO_PC_TIMEOUT_MS        2000U
#define SANPO_MOTOR_TIMEOUT_MS     500U

/* 五关节协同动作参数。 */
#define SANPO_GROUP_MIN_DURATION_MS  200U
#define SANPO_GROUP_MAX_DURATION_MS  60000U
#define SANPO_GROUP_MIN_MOTOR_RPM    0.10f

#endif

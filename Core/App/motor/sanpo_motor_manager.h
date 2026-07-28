#ifndef SANPO_MOTOR_MANAGER_H
#define SANPO_MOTOR_MANAGER_H

#include <stdint.h>

typedef enum
{
    SANPO_MOTOR_OFFLINE = 0,
    SANPO_MOTOR_READY,
    SANPO_MOTOR_MOVING,
    SANPO_MOTOR_FAULT,
    SANPO_MOTOR_DISABLED
} SanpoMotorMode;

typedef struct
{
    float target_deg;
    float actual_deg;
    float speed_rpm;
    float current_a;
    uint32_t last_rx_ms;
    uint8_t fault;
    uint8_t online;
    uint8_t done;
    SanpoMotorMode mode;
} SanpoJointState;

typedef struct
{
    uint8_t staged_mask;
    uint8_t active_mask;
    uint8_t done_mask;
    uint8_t fault_mask;
    uint8_t sequence;
    uint8_t active;
} SanpoGroupState;

void SanpoMotor_Init(void);
int SanpoMotor_MoveJoint(uint8_t joint_id, float angle_deg, float speed_rpm);
int SanpoMotor_StageGroupJoint(uint8_t joint_id, float angle_deg);
int SanpoMotor_ExecuteGroup(uint16_t duration_ms);
const SanpoGroupState *SanpoMotor_GetGroupState(void);
int SanpoMotor_StopAll(void);
int SanpoMotor_Home(uint8_t joint_id);
int SanpoMotor_ClearFault(uint8_t joint_id);
const SanpoJointState *SanpoMotor_GetState(uint8_t joint_id);
void SanpoMotor_OnCan(uint8_t channel, uint16_t id, const uint8_t *data,
                      uint8_t length, uint32_t now_ms);
void SanpoMotor_Process(uint32_t now_ms);

#endif

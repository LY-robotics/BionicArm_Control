#ifndef SANPO_APP_H
#define SANPO_APP_H

#include "can.h"
#include <stdint.h>

void SanpoApp_Init(void);
void SanpoApp_Process(void);

uint8_t SanpoApp_IsUsbFrame(const uint8_t *data, uint16_t length);
void SanpoApp_UsbPush(const uint8_t *data, uint16_t length);

uint8_t SanpoApp_OwnsCan(void);
void SanpoApp_CanRxFifo0(CAN_HandleTypeDef *hcan);
void SanpoApp_CanRxFifo1(CAN_HandleTypeDef *hcan);

#endif

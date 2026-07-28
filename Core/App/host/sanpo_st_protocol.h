#ifndef SANPO_ST_PROTOCOL_H
#define SANPO_ST_PROTOCOL_H

#include <stdint.h>

typedef struct
{
    uint8_t channel;
    uint16_t can_id;
    uint8_t dlc;
    uint8_t data[8];
} SanpoStFrame;

int SanpoSt_Decode(const uint8_t *data, uint16_t length, SanpoStFrame *frame);
uint16_t SanpoSt_Encode(const SanpoStFrame *frame, uint8_t *output,
                       uint16_t capacity);
uint8_t SanpoSt_IsApplicationFrame(const uint8_t *data, uint16_t length);

#endif

#include "sanpo_st_protocol.h"

#include "../config/sanpo_board_config.h"

#include <string.h>

#define ST_FIXED_SIZE 10U

static uint16_t ReadU16Be(const uint8_t *data)
{
    return ((uint16_t)data[0] << 8) | data[1];
}

static void WriteU16Be(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;
}

int SanpoSt_Decode(const uint8_t *data, uint16_t length, SanpoStFrame *frame)
{
    uint8_t dlc;

    if ((data == NULL) || (frame == NULL) || (length < ST_FIXED_SIZE)) {
        return -1;
    }
    if ((data[0] != 'S') || (data[1] != 'T')) {
        return -2;
    }

    dlc = data[7];
    if ((dlc > 8U) || (length != (uint16_t)(ST_FIXED_SIZE + dlc))) {
        return -3;
    }
    if ((data[length - 2U] != '\r') || (data[length - 1U] != '\n')) {
        return -4;
    }

    frame->channel = data[2];
    frame->can_id = ReadU16Be(&data[5]);
    frame->dlc = dlc;
    memset(frame->data, 0, sizeof(frame->data));
    memcpy(frame->data, &data[8], dlc);
    return 0;
}

uint16_t SanpoSt_Encode(const SanpoStFrame *frame, uint8_t *output,
                       uint16_t capacity)
{
    uint16_t length;

    if ((frame == NULL) || (output == NULL) || (frame->dlc > 8U)) {
        return 0U;
    }
    length = ST_FIXED_SIZE + frame->dlc;
    if (capacity < length) {
        return 0U;
    }

    output[0] = 'S';
    output[1] = 'T';
    output[2] = frame->channel;
    output[3] = 0U;
    output[4] = 0U;
    WriteU16Be(&output[5], frame->can_id);
    output[7] = frame->dlc;
    memcpy(&output[8], frame->data, frame->dlc);
    output[8U + frame->dlc] = '\r';
    output[9U + frame->dlc] = '\n';
    return length;
}

uint8_t SanpoSt_IsApplicationFrame(const uint8_t *data, uint16_t length)
{
    SanpoStFrame frame;

    return ((SanpoSt_Decode(data, length, &frame) == 0) &&
            (frame.channel == SANPO_APP_USB_CHANNEL) &&
            (frame.can_id == SANPO_APP_REQUEST_ID)) ? 1U : 0U;
}

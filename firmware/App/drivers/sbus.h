// SBUS frame decoding (25 bytes, 16 x 11-bit channels + flags). Pure C, no HAL.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define SBUS_FRAME_LEN    25u
#define SBUS_NUM_CHANNELS 16u

typedef struct {
    uint16_t ch[SBUS_NUM_CHANNELS];  // raw 0..2047
    bool frame_lost;                 // receiver missed a frame (it repeats the last one)
    bool failsafe;                   // receiver lost the transmitter link
} sbus_frame_t;

// Returns false when header/end byte do not match (caller discards the frame).
bool sbus_decode(const uint8_t frame[SBUS_FRAME_LEN], sbus_frame_t *out);

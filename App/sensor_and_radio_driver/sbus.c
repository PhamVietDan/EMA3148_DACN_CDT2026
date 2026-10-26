// SBUS frame decoding (25 bytes, 16 x 11-bit channels + flags). Pure C, no HAL.
#include "sensor_and_radio_driver/sbus.h"

#define SBUS_HEADER    0x0Fu
#define FLAG_LOST_BIT  0x04u  // byte 23 bit 2
#define FLAG_FS_BIT    0x08u  // byte 23 bit 3

bool sbus_decode(const uint8_t frame[SBUS_FRAME_LEN], sbus_frame_t *out)
{
    // End byte is 0x00 for SBUS; SBUS2 receivers cycle 0x04/0x14/0x24/0x34.
    uint8_t end = frame[24];
    if (frame[0] != SBUS_HEADER || !(end == 0x00u || (end & 0x0Fu) == 0x04u)) {
        return false;
    }
    // 16 channels packed LSB-first in bytes 1..22.
    uint32_t bits = 0;
    uint32_t nbits = 0;
    uint32_t byte_idx = 1;
    for (uint32_t c = 0; c < SBUS_NUM_CHANNELS; c++) {
        while (nbits < 11u) {
            bits |= (uint32_t)frame[byte_idx++] << nbits;
            nbits += 8u;
        }
        out->ch[c] = (uint16_t)(bits & 0x07FFu);
        bits >>= 11;
        nbits -= 11u;
    }
    out->frame_lost = (frame[23] & FLAG_LOST_BIT) != 0u;
    out->failsafe = (frame[23] & FLAG_FS_BIT) != 0u;
    return true;
}

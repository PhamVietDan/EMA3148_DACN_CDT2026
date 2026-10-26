// RAM black box (PLAN M6): fixed-size records at 50 Hz in CCM RAM, newest ~30 s kept, dumped over USB.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t t_ms;
    int16_t roll, pitch, yaw;   // 0.01 deg
    int16_t p, q, r;            // 0.1 deg/s
    int16_t roll_sp, pitch_sp;  // 0.01 deg
    int16_t yawr_sp;            // 0.1 deg/s
    uint16_t thr;               // 0.001
    uint16_t m_us[4];           // M1..M4 ESC pulse
    uint8_t arm, fs, mode, rc_valid;
} bb_rec_t;

#define BB_RECORDS 1500u  // 30 s at 50 Hz, 54 KB of the 64 KB CCM

void blackbox_reset(void);
void blackbox_add(const bb_rec_t *r);    // overwrites the oldest record when full
size_t blackbox_count(void);
const bb_rec_t *blackbox_get(size_t i);  // 0 = oldest
size_t blackbox_format(const bb_rec_t *r, char *buf, size_t size);  // "L,..." line with CRLF
const char *blackbox_header(void);       // "L,t_ms,..." column names with CRLF

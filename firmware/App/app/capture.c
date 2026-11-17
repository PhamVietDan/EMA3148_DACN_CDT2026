// High-rate capture for rate-loop tuning (see capture.h).
#include "app/capture.h"

#include <stdio.h>

#include "lib/mathx.h"

// capture_add sums cap_in_t as a flat float array.
_Static_assert(sizeof(cap_in_t) == 19u * sizeof(float), "cap_in_t must hold floats only");

// Plain .bss (SRAM). Only one task touches it in each state, so no locking is needed.
static cap_rec_t buf[CAP_RECORDS];
static volatile uint8_t state = CAP_IDLE;
static size_t count;
static uint32_t decim = CAP_DECIM_DEFAULT;
static cap_in_t acc;  // running sum over the current record
static uint32_t acc_n;
static uint32_t acc_dt_us;

cap_state_t capture_state(void)
{
    return (cap_state_t)state;
}

uint32_t capture_decim(void)
{
    return decim;
}

bool capture_arm(uint32_t d)
{
    if (state != CAP_IDLE || d < CAP_DECIM_MIN || d > CAP_DECIM_MAX) {
        return false;
    }
    decim = d;
    count = 0;
    acc = (cap_in_t){0};
    acc_n = 0;
    acc_dt_us = 0;
    state = CAP_RECORDING;  // written last: task_ctrl starts only after the reset above
    return true;
}

static int16_t to_i16(float v)
{
    return (int16_t)clampf(v + (v >= 0.0f ? 0.5f : -0.5f), -32768.0f, 32767.0f);
}

static uint16_t to_u16(float v)
{
    return (uint16_t)clampf(v + 0.5f, 0.0f, 65535.0f);
}

void capture_add(const cap_in_t *in, uint32_t dt_us)
{
    if (state != CAP_RECORDING) {
        return;
    }
    const float *src = (const float *)in;
    float *sum = (float *)&acc;
    for (size_t i = 0; i < sizeof acc / sizeof(float); i++) {
        sum[i] += src[i];
    }
    acc_dt_us += dt_us;
    if (++acc_n < decim) {
        return;
    }
    const float k = 1.0f / (float)acc_n;
    const float r2d10 = MATHX_RAD2DEG * 10.0f * k;
    cap_rec_t *r = &buf[count];
    r->dt_us = (uint16_t)((acc_dt_us > 65535u) ? 65535u : acc_dt_us);
    for (int i = 0; i < 3; i++) {
        r->rate[i] = to_i16(acc.rate[i] * r2d10);
        r->sp[i] = to_i16(acc.sp[i] * r2d10);
    }
    for (int i = 0; i < 8; i++) {
        r->term[i] = to_i16(acc.term[i] * 1e4f * k);
    }
    r->thr = to_u16(acc.thr * 1000.0f * k);
    for (int i = 0; i < 4; i++) {
        r->m_us[i] = to_u16(acc.m_us[i] * k);
    }
    acc = (cap_in_t){0};
    acc_n = 0;
    acc_dt_us = 0;
    if (++count >= CAP_RECORDS) {
        state = CAP_DONE;
    }
}

size_t capture_count(void)
{
    return (state == CAP_DONE) ? count : 0u;
}

const cap_rec_t *capture_get(size_t i)
{
    return (state == CAP_DONE && i < count) ? &buf[i] : NULL;
}

void capture_release(void)
{
    state = CAP_IDLE;
}

const char *capture_header(void)
{
    return "C,i,dt_us,p,q,r,p_sp,q_sp,r_sp,P_roll,I_roll,D_roll,P_pitch,I_pitch,D_pitch,P_yaw,I_yaw,"
           "thr,m1,m2,m3,m4\r\n";
}

size_t capture_format(size_t index, const cap_rec_t *r, char *out, size_t size)
{
    int n = snprintf(out, size, "C,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u\r\n",
                     (unsigned)index, r->dt_us, r->rate[0], r->rate[1], r->rate[2], r->sp[0], r->sp[1], r->sp[2],
                     r->term[0], r->term[1], r->term[2], r->term[3], r->term[4], r->term[5], r->term[6],
                     r->term[7], r->thr, r->m_us[0], r->m_us[1], r->m_us[2], r->m_us[3]);
    return (n > 0 && (size_t)n < size) ? (size_t)n : 0u;
}

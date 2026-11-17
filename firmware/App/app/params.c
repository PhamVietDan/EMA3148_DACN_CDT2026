// Tunable parameter table (PLAN §8): name, default, min, max; flash image with CRC. Pure C (host-tested).
#include "app/params.h"

#include <stddef.h>
#include <string.h>

#include "lib/crc32.h"

#define P(field, def, min, max, flags) {#field, (uint16_t)offsetof(params_t, field), def, min, max, flags}

// Defaults are the M5 starting values (small P, I = D = 0, PLAN §8); tuned in M7.
static const param_desc_t table[] = {
    P(rate_p_rp,     0.05f,    0.0f,   1.0f,    0),
    P(rate_i_rp,     0.0f,     0.0f,   2.0f,    0),
    P(rate_d_rp,     0.0f,     0.0f,   0.05f,   0),
    P(rate_p_y,      0.05f,    0.0f,   1.0f,    0),
    P(rate_i_y,      0.0f,     0.0f,   2.0f,    0),
    P(rate_ilim,     0.3f,     0.0f,   0.5f,    0),
    P(ang_p_rp,      4.0f,     0.0f,   20.0f,   0),
    P(max_angle,     25.0f,    5.0f,   45.0f,   0),
    P(max_rate_rp,   200.0f,   30.0f,  720.0f,  0),
    P(max_yaw_rate,  120.0f,   30.0f,  360.0f,  0),
    P(gyro_lpf_hz,   150.0f,   20.0f,  500.0f,  PARAM_DISARMED_ONLY),
    P(acc_lpf_hz,    25.0f,    1.0f,   100.0f,  PARAM_DISARMED_ONLY),
    P(dterm_lpf_hz,  80.0f,    10.0f,  250.0f,  PARAM_DISARMED_ONLY),
    P(kf_sg,         4.0f,     0.1f,   50.0f,   0),
    P(kf_sacc,       3.0f,     0.1f,   50.0f,   0),
    P(motor_idle_us, 1000.0f,  950.0f, 1200.0f, 0),
    P(rc_deadband,   0.03f,    0.0f,   0.2f,    0),
    P(lc_angle,      30.0f,    15.0f,  90.0f,   0),
    P(lc_time_ms,    1000.0f,  200.0f, 5000.0f, 0),
    // Fitted to the 2026-10-28 stand captures: roll vibration ~81 Hz at mean motor 0.40, ~92 Hz at 0.57.
    P(notch_lo_hz,   55.0f,    0.0f,   300.0f,  0),
    P(notch_hi_hz,   120.0f,   20.0f,  400.0f,  0),
    P(notch_q,       2.5f,     0.5f,   10.0f,   0),
    P(mag_off_x,     0.0f,     -500.0f, 500.0f, 0),
    P(mag_off_y,     0.0f,     -500.0f, 500.0f, 0),
    P(mag_off_z,     0.0f,     -500.0f, 500.0f, 0),
    P(alt_sigma_a,   0.3f,     0.01f,  5.0f,    0),
    P(alt_sigma_tof, 0.05f,    0.005f, 1.0f,    0),
    // 0 = no ToF step detection (flat floor; user 2026-10-29: most altitude errors came from false steps)
    P(alt_step_th,   0.0f,     0.0f,   1.0f,    0),
    P(trim_roll,     0.0f,     -5.0f,  5.0f,    0),
    P(trim_pitch,    0.0f,     -5.0f,  5.0f,    0),
    // Starting values from hover_thr 0.41 (~24 m/s^2 per unit throttle): vz loop ~2.4 rad/s, z loop 1 rad/s.
    // Conservative on purpose; tuned in M10 from logs.
    P(alt_p,         1.0f,     0.0f,   5.0f,    0),
    P(vz_p,          0.10f,    0.0f,   0.5f,    0),
    P(vz_i,          0.10f,    0.0f,   0.5f,    0),
    P(vz_max,        0.5f,     0.1f,   1.5f,    0),
    P(alt_thr_rng,   0.15f,    0.05f,  0.3f,    0),
    // 2.5/s: a 10 deg heading error asks 25 deg/s back, well inside the yaw rate loop; tuned in flight.
    P(yaw_hold_p,    2.5f,     0.0f,   10.0f,   0),
    // Learnt in flight 2026-11-10: 0.451 (0.40 early in the pack). In Angle, 0.05 of stick over hover is ~1 m/s^2
    // of climb: the expo halves that around the hover point (the "sudden climb" at take-off, log 20:02).
    P(hover_thr,     0.45f,    0.0f,   0.8f,    0),
    P(thr_mid,       0.45f,    0.1f,   0.9f,    0),
    P(thr_expo,      0.4f,     0.0f,   1.0f,    0),
    // Calibrated 2026-11-10: rocking -4.75/-4.80 (twice), 1.2 m hand slides -3% (PROGRESS "hiệu chuẩn flow").
    // Sensor moved 2026-11-14 (flow_rot 180): -4.46/-4.27 rocking at 0.9 m (log 17:27).
    P(flow_k,        -4.4f,    -20.0f, 20.0f,   0),
    P(flow_scale,    1.0f,     0.5f,   2.0f,    0),
    P(flow_qmin,     50.0f,    0.0f,   255.0f,  0),  // hand tests 67..112 on this floor
    P(flow_sigma,    0.2f,     0.02f,  2.0f,    0),  // ~1 count / k
    // Model (tests/test_pos_hold.c: 0.15 s tilt lag, 30 ms flow delay): brake from 0.8 m/s in ~0.5 m at <= 6.3 deg,
    // 0.5 m/s^2 push held within 0.27 m. Conservative on purpose; tuned from flight logs.
    P(pos_p,         0.8f,     0.0f,   3.0f,    0),
    P(vel_p,         10.0f,    0.0f,   40.0f,   0),
    P(vel_i,         3.0f,     0.0f,   20.0f,   0),
    P(vel_max,       1.0f,     0.2f,   3.0f,    0),
    P(pos_ang_max,   12.0f,    3.0f,   25.0f,   0),
    P(takeoff_alt,   0.8f,     0.0f,   2.0f,    0),
    // PLAN §7.4b: 0.3. Hand test 2026-11-13 (motors off): +-2-3 cm noise, 0.88 m table read 0.85-0.93 m.
    P(alt_sigma_baro, 0.3f,    0.05f,  2.0f,    0),
    // Sensor moved 2026-11-14, hand test log 17:27: both axes reversed (180), rocking k -4.46/-4.27 at 0.9 m.
    // Rounded to a quarter turn. Not while armed: the hold would steer on a different axis mid-flight.
    P(flow_rot,      180.0f,   0.0f,   270.0f,  PARAM_DISARMED_ONLY),
};

#define N_PARAMS (sizeof table / sizeof table[0])
#define IMAGE_MAGIC   0x4D525044u  // "DPRM"
#define IMAGE_VERSION 1u
#define HEADER_LEN    12u
#define ENTRY_LEN     8u

size_t params_count(void)
{
    return N_PARAMS;
}

const param_desc_t *params_desc(size_t i)
{
    return (i < N_PARAMS) ? &table[i] : NULL;
}

int params_find(const char *name)
{
    for (size_t i = 0; i < N_PARAMS; i++) {
        if (strcmp(table[i].name, name) == 0) {
            return (int)i;
        }
    }
    return -1;
}

static float *slot(params_t *p, size_t i)
{
    return (float *)((uint8_t *)p + table[i].offset);
}

void params_defaults(params_t *p)
{
    for (size_t i = 0; i < N_PARAMS; i++) {
        *slot(p, i) = table[i].def;
    }
}

float params_get(const params_t *p, size_t i)
{
    return *(const float *)((const uint8_t *)p + table[i].offset);
}

param_result_t params_set(params_t *p, size_t i, float value)
{
    if (i >= N_PARAMS) {
        return PARAM_ERR_NAME;
    }
    // !(a <= b) also rejects NaN.
    if (!(value >= table[i].min && value <= table[i].max)) {
        return PARAM_ERR_RANGE;
    }
    *slot(p, i) = value;
    return PARAM_OK;
}

static uint32_t fnv1a(const char *s)
{
    uint32_t h = 2166136261u;
    while (*s != '\0') {
        h ^= (uint8_t)*s++;
        h *= 16777619u;
    }
    return h;
}

static void put_u32(uint8_t *b, uint32_t v)
{
    b[0] = (uint8_t)v;
    b[1] = (uint8_t)(v >> 8);
    b[2] = (uint8_t)(v >> 16);
    b[3] = (uint8_t)(v >> 24);
}

static uint32_t get_u32(const uint8_t *b)
{
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

size_t params_image_build(const params_t *p, uint8_t *buf, size_t size)
{
    size_t len = HEADER_LEN + N_PARAMS * ENTRY_LEN;
    if (size < len) {
        return 0;
    }
    for (size_t i = 0; i < N_PARAMS; i++) {
        uint8_t *e = &buf[HEADER_LEN + i * ENTRY_LEN];
        float v = params_get(p, i);
        uint32_t bits;
        memcpy(&bits, &v, sizeof bits);
        put_u32(e, fnv1a(table[i].name));
        put_u32(e + 4, bits);
    }
    put_u32(&buf[0], IMAGE_MAGIC);
    buf[4] = (uint8_t)IMAGE_VERSION;
    buf[5] = 0;
    buf[6] = (uint8_t)N_PARAMS;
    buf[7] = (uint8_t)(N_PARAMS >> 8);
    put_u32(&buf[8], crc32_update(0, &buf[HEADER_LEN], N_PARAMS * ENTRY_LEN));
    return len;
}

param_load_t params_image_load(params_t *p, const uint8_t *buf, size_t size, unsigned *n_loaded)
{
    *n_loaded = 0;
    if (size < HEADER_LEN) {
        return PARAM_LOAD_CORRUPT;
    }
    uint32_t magic = get_u32(&buf[0]);
    if (magic == 0xFFFFFFFFu) {
        return PARAM_LOAD_EMPTY;
    }
    size_t count = (size_t)buf[6] | ((size_t)buf[7] << 8);
    if (magic != IMAGE_MAGIC || buf[4] != IMAGE_VERSION || count > (PARAMS_IMAGE_MAX - HEADER_LEN) / ENTRY_LEN ||
        size < HEADER_LEN + count * ENTRY_LEN ||
        crc32_update(0, &buf[HEADER_LEN], count * ENTRY_LEN) != get_u32(&buf[8])) {
        return PARAM_LOAD_CORRUPT;
    }
    for (size_t k = 0; k < count; k++) {
        const uint8_t *e = &buf[HEADER_LEN + k * ENTRY_LEN];
        uint32_t h = get_u32(e);
        uint32_t bits = get_u32(e + 4);
        float v;
        memcpy(&v, &bits, sizeof v);
        for (size_t i = 0; i < N_PARAMS; i++) {
            if (fnv1a(table[i].name) == h) {
                if (params_set(p, i, v) == PARAM_OK) {
                    (*n_loaded)++;
                }
                break;
            }
        }
    }
    return PARAM_LOAD_OK;
}

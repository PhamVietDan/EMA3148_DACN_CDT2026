// Tunable parameter table (PLAN §8): name, default, min, max; flash image with CRC. Pure C (host-tested).
#include "control_logic/params.h"

#include <stddef.h>
#include <string.h>

#include "math_algorithm/crc32.h"

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

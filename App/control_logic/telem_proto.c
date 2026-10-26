// ASCII telemetry line builder (PLAN §9.1). Transport-agnostic: returns text, caller sends it.
#include "control_logic/telem_proto.h"

#include <stdbool.h>

#include "math_algorithm/mathx.h"

typedef struct {
    char *buf;
    size_t size, len;
} line_t;

static void put_char(line_t *l, char c)
{
    if (l->len + 1u < l->size) {
        l->buf[l->len++] = c;
    }
}

static void put_uint(line_t *l, uint32_t v)
{
    char tmp[10];
    int n = 0;
    do {
        tmp[n++] = (char)('0' + v % 10u);
        v /= 10u;
    } while (v != 0u && n < (int)sizeof tmp);
    while (n > 0) {
        put_char(l, tmp[--n]);
    }
}

// Fixed-point decimal without printf %f (newlib-nano float printing would pull in malloc).
static void put_fixed(line_t *l, float v, uint32_t decimals)
{
    uint32_t scale = 1u;
    for (uint32_t i = 0; i < decimals; i++) {
        scale *= 10u;
    }
    float scaled = v * (float)scale;
    bool neg = scaled < 0.0f;
    if (neg) {
        scaled = -scaled;
    }
    if (scaled > 2.0e9f) {
        scaled = 2.0e9f;  // out of range for the field; clip rather than overflow
    }
    uint32_t t = (uint32_t)(scaled + 0.5f);
    if (neg && t != 0u) {
        put_char(l, '-');
    }
    put_uint(l, t / scale);
    if (decimals > 0u) {
        put_char(l, '.');
        uint32_t frac = t % scale;
        for (uint32_t d = scale / 10u; d > 0u; d /= 10u) {
            put_char(l, (char)('0' + (frac / d) % 10u));
        }
    }
}

static void sep_uint(line_t *l, uint32_t v)
{
    put_char(l, ',');
    put_uint(l, v);
}

static void sep_fixed(line_t *l, float v, uint32_t decimals)
{
    put_char(l, ',');
    put_fixed(l, v, decimals);
}

size_t telem_proto_build_T(char *buf, size_t size, const telem_frame_t *f)
{
    line_t l = {buf, size, 0};
    put_char(&l, 'T');
    sep_uint(&l, f->t_ms);
    sep_uint(&l, f->arm);
    sep_uint(&l, f->mode);
    sep_uint(&l, f->fs);
    sep_fixed(&l, f->roll * MATHX_RAD2DEG, 2);
    sep_fixed(&l, f->pitch * MATHX_RAD2DEG, 2);
    sep_fixed(&l, f->yaw * MATHX_RAD2DEG, 2);
    sep_fixed(&l, f->p * MATHX_RAD2DEG, 1);
    sep_fixed(&l, f->q * MATHX_RAD2DEG, 1);
    sep_fixed(&l, f->r * MATHX_RAD2DEG, 1);
    sep_fixed(&l, f->roll_sp * MATHX_RAD2DEG, 2);
    sep_fixed(&l, f->pitch_sp * MATHX_RAD2DEG, 2);
    sep_fixed(&l, f->yawr_sp * MATHX_RAD2DEG, 1);
    sep_fixed(&l, f->z, 3);
    sep_fixed(&l, f->vz, 3);
    sep_fixed(&l, f->z_sp, 3);
    sep_fixed(&l, f->tof, 3);
    sep_fixed(&l, f->thr, 3);
    for (int i = 0; i < 4; i++) {
        sep_fixed(&l, f->motor[i], 3);
    }
    sep_uint(&l, f->ctrl_us_max);
    sep_uint(&l, f->jitter_us_max);
    sep_fixed(&l, f->h_terrain, 3);
    sep_fixed(&l, f->baro_rel, 3);
    sep_uint(&l, f->alt_src);
    put_char(&l, '\r');
    put_char(&l, '\n');
    l.buf[l.len] = '\0';
    return l.len;
}

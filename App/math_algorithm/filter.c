// Discrete low-pass filters (pure C, float). Coefficients are computed for a fixed sample rate.
#include "math_algorithm/filter.h"

#include <math.h>

#include "math_algorithm/mathx.h"

void pt1_init(pt1_t *f, float cutoff_hz, float sample_hz)
{
    // k = dt / (RC + dt), RC = 1 / (2*pi*fc)
    float dt = 1.0f / sample_hz;
    float rc = 1.0f / (2.0f * MATHX_PI * cutoff_hz);
    f->k = dt / (rc + dt);
    f->state = 0.0f;
    f->primed = false;
}

float pt1_apply(pt1_t *f, float x)
{
    if (!f->primed) {
        f->state = x;
        f->primed = true;
    }
    f->state += f->k * (x - f->state);
    return f->state;
}

void biquad_lpf_init(biquad_t *f, float cutoff_hz, float sample_hz)
{
    // RBJ Audio EQ Cookbook low-pass (public formulas), Q = 1/sqrt(2).
    const float q = 0.70710678f;
    float w0 = 2.0f * MATHX_PI * cutoff_hz / sample_hz;
    float sn = sinf(w0);
    float cs = cosf(w0);
    float alpha = sn / (2.0f * q);
    float a0 = 1.0f + alpha;
    f->b0 = ((1.0f - cs) * 0.5f) / a0;
    f->b1 = (1.0f - cs) / a0;
    f->b2 = f->b0;
    f->a1 = (-2.0f * cs) / a0;
    f->a2 = (1.0f - alpha) / a0;
    f->z1 = 0.0f;
    f->z2 = 0.0f;
    f->primed = false;
}

float biquad_apply(biquad_t *f, float x)
{
    if (!f->primed) {
        // Steady-state for a constant input x (DC gain is 1): avoids a start-up ramp from 0.
        f->z1 = x - f->b0 * x;
        f->z2 = f->b2 * x - f->a2 * x;
        f->primed = true;
    }
    float y = f->b0 * x + f->z1;
    f->z1 = f->b1 * x - f->a1 * y + f->z2;
    f->z2 = f->b2 * x - f->a2 * y;
    return y;
}

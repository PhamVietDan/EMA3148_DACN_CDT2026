// Small float-only math helpers shared by lib/ and app/.
#pragma once

#define MATHX_PI      3.14159265f
#define MATHX_DEG2RAD (MATHX_PI / 180.0f)
#define MATHX_RAD2DEG (180.0f / MATHX_PI)
#define MATHX_G       9.80665f

static inline float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

static inline float wrap_pi(float a)
{
    while (a > MATHX_PI) {
        a -= 2.0f * MATHX_PI;
    }
    while (a < -MATHX_PI) {
        a += 2.0f * MATHX_PI;
    }
    return a;
}

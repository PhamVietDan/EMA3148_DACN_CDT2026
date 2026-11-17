// Host tests for App/app/levelcal.c: upright and upside-down (on the motor caps) give the same trim.
#include <math.h>

#include "app/levelcal.h"
#include "lib/mathx.h"
#include "test_common.h"

// Specific force (FRD, at rest) of an IMU mounted at roll r / pitch p relative to a level prop plane.
static void level_force(float r, float p, float f[3])
{
    f[0] = MATHX_G * sinf(p);
    f[1] = -MATHX_G * sinf(r) * cosf(p);
    f[2] = -MATHX_G * cosf(r) * cosf(p);
}

static levelcal_result_t run(const float f[3], float wobble, float *roll, float *pitch, bool *inv)
{
    levelcal_t c;
    levelcal_start(&c);
    for (uint32_t i = 0; i < LEVELCAL_SAMPLES; i++) {
        float a[3] = {f[0], f[1], f[2] + ((i & 1u) ? wobble : -wobble)};
        (void)levelcal_add(&c, a);
    }
    return levelcal_finish(&c, roll, pitch, inv);
}

void test_levelcal(void)
{
    const float r0 = 1.0f * MATHX_DEG2RAD, p0 = -0.5f * MATHX_DEG2RAD;
    float f[3], neg[3], roll, pitch;
    bool inv;
    level_force(r0, p0, f);
    CHECK(run(f, 0.0f, &roll, &pitch, &inv) == LEVELCAL_OK && !inv && fabsf(roll - r0) < 1e-4f &&
              fabsf(pitch - p0) < 1e-4f,
          "upright: roll %.3f pitch %.3f deg", (double)(roll * MATHX_RAD2DEG), (double)(pitch * MATHX_RAD2DEG));
    // Flipped 180 deg about any horizontal axis onto the motor caps: the body-frame force is negated.
    for (int i = 0; i < 3; i++) {
        neg[i] = -f[i];
    }
    CHECK(run(neg, 0.0f, &roll, &pitch, &inv) == LEVELCAL_OK && inv && fabsf(roll - r0) < 1e-4f &&
              fabsf(pitch - p0) < 1e-4f,
          "inverted: roll %.3f pitch %.3f deg", (double)(roll * MATHX_RAD2DEG), (double)(pitch * MATHX_RAD2DEG));
    CHECK(run(f, 0.3f, &roll, &pitch, &inv) == LEVELCAL_MOVED, "motion not refused");
    level_force(20.0f * MATHX_DEG2RAD, 0.0f, f);
    CHECK(run(f, 0.0f, &roll, &pitch, &inv) == LEVELCAL_TILTED, "20 deg tilt not refused");
}

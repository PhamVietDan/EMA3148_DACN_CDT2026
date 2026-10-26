// Host tests for App/math_algorithm/mixer.c and App/math_algorithm/pid.c (PLAN §12): mixer signs per §7.6, saturation, anti-windup.
#include <math.h>

#include "math_algorithm/mixer.h"
#include "math_algorithm/pid.h"
#include "test_common.h"

static const float IDLE = 0.1f;

static void mix(float thr, float r, float p, float y, float m[4], mixer_sat_t *sat)
{
    const float tq[3] = {r, p, y};
    mixer_quadx(thr, tq, IDLE, m, sat);
}

static void test_mixer_signs(void)
{
    float m[4];
    mixer_sat_t sat;
    // M1 front-right, M2 rear-left, M3 front-left, M4 rear-right.
    mix(0.5f, 0.1f, 0.0f, 0.0f, m, &sat);  // roll right: left motors (M2, M3) up
    CHECK(m[1] > m[0] && m[2] > m[3] && fabsf(m[1] - m[2]) < 1e-6f, "roll + must raise M2/M3");
    mix(0.5f, 0.0f, 0.1f, 0.0f, m, &sat);  // nose up: front motors (M1, M3) up
    CHECK(m[0] > m[1] && m[2] > m[3] && fabsf(m[0] - m[2]) < 1e-6f, "pitch + must raise M1/M3");
    mix(0.5f, 0.0f, 0.0f, 0.1f, m, &sat);  // yaw clockwise: CCW motors (M1, M2) up
    CHECK(m[0] > m[2] && m[1] > m[3] && fabsf(m[0] - m[1]) < 1e-6f, "yaw + must raise M1/M2");
    mix(0.5f, 0.0f, 0.0f, 0.0f, m, &sat);
    float expect = IDLE + (1.0f - IDLE) * 0.5f;
    CHECK(fabsf(m[0] - expect) < 1e-6f && fabsf(m[3] - expect) < 1e-6f, "zero torque: all = throttle");
    // Mean of the motors equals the throttle level when nothing saturates (pure differential torque).
    mix(0.5f, 0.1f, -0.05f, 0.03f, m, &sat);
    CHECK(fabsf((m[0] + m[1] + m[2] + m[3]) / 4.0f - expect) < 1e-6f, "torque changed the mean thrust");
}

static void test_mixer_saturation(void)
{
    float m[4];
    mixer_sat_t sat;
    // Full throttle + roll: throttle is pulled down so the roll difference survives.
    mix(1.0f, 0.2f, 0.0f, 0.0f, m, &sat);
    CHECK(fabsf((m[1] - m[0]) - 0.4f) < 1e-5f, "roll authority lost at full throttle (diff %f)", (double)(m[1] - m[0]));
    CHECK(m[1] <= 1.0f && m[0] >= IDLE, "motor out of range");
    CHECK(!sat.pos[0] && !sat.neg[0], "throttle shift is not a torque saturation");

    // Low throttle: lifted to keep the low motors at idle instead of clipping them.
    mix(0.0f, 0.2f, 0.0f, 0.0f, m, &sat);
    CHECK(fabsf(m[0] - IDLE) < 1e-6f && fabsf((m[1] - m[0]) - 0.4f) < 1e-5f, "low-throttle roll clipped");

    // Too much roll/pitch for the span: scaled, flagged in the commanded direction, still in range.
    mix(0.5f, 0.8f, -0.6f, 0.0f, m, &sat);
    for (int i = 0; i < 4; i++) {
        CHECK(m[i] >= IDLE - 1e-6f && m[i] <= 1.0f + 1e-6f, "M%d = %f out of range", i + 1, (double)m[i]);
    }
    CHECK(sat.pos[0] && !sat.neg[0] && sat.neg[1] && !sat.pos[1], "roll/pitch saturation flags wrong");
    // Unscaled rp: M1 -1.4, M2 +1.4, M3 +0.2, M4 -0.2 -> order M2 > M3 > M4 > M1 must survive the scaling.
    CHECK(m[1] > m[2] && m[2] > m[3] && m[3] > m[0], "scaled roll/pitch lost their ordering");

    // Yaw has lowest priority: with roll/pitch using the whole span, yaw is cut and flagged, roll kept.
    mix(0.5f, 0.45f, 0.0f, 0.5f, m, &sat);
    CHECK(sat.pos[2] && !sat.pos[0], "yaw should be limited before roll");
    CHECK(fabsf((m[1] + m[2]) - (m[0] + m[3]) - 4.0f * 0.45f) < 1e-4f, "roll torque reduced to make room for yaw");
}

static void test_pid(void)
{
    pid_ctrl_t p;
    const float hz = 1000.0f, dt = 1.0f / hz;
    pid_init(&p, 80.0f, hz);
    pid_gains_t g = {.kp = 0.1f, .ki = 0.0f, .kd = 0.0f, .kff = 0.0f, .i_limit = 0.3f};
    float out = pid_update(&p, &g, 1.0f, 0.0f, dt, true, false, false);
    CHECK(fabsf(out - 0.1f) < 1e-6f, "P only: %f", (double)out);

    // Integrator: 1 s of error 1 rad/s at ki 0.2 -> 0.2, then clamped at 0.3.
    pid_init(&p, 80.0f, hz);
    g = (pid_gains_t){.kp = 0.0f, .ki = 0.2f, .kd = 0.0f, .kff = 0.0f, .i_limit = 0.3f};
    for (int i = 0; i < 1000; i++) {
        out = pid_update(&p, &g, 1.0f, 0.0f, dt, true, false, false);
    }
    CHECK(fabsf(out - 0.2f) < 1e-3f, "I after 1 s: %f", (double)out);
    for (int i = 0; i < 2000; i++) {
        out = pid_update(&p, &g, 1.0f, 0.0f, dt, true, false, false);
    }
    CHECK(fabsf(out - 0.3f) < 1e-6f, "I limit: %f", (double)out);

    // Anti-windup: saturated positive -> no further positive growth, but it may unwind.
    pid_init(&p, 80.0f, hz);
    for (int i = 0; i < 500; i++) {
        out = pid_update(&p, &g, 1.0f, 0.0f, dt, true, true, false);
    }
    CHECK(fabsf(out) < 1e-6f, "integrated into a positive saturation: %f", (double)out);
    for (int i = 0; i < 500; i++) {
        out = pid_update(&p, &g, -1.0f, 0.0f, dt, true, true, false);
    }
    CHECK(out < -0.09f, "could not unwind out of the saturated direction: %f", (double)out);

    // integrate = false holds I (on the ground / idle it is reset by the caller).
    float held = out;
    out = pid_update(&p, &g, 5.0f, 0.0f, dt, false, false, false);
    CHECK(fabsf(out - held) < 1e-6f, "I moved while integrate = false");

    // D on measurement: a setpoint step gives no D kick; a measurement ramp gives -kd * rate.
    pid_init(&p, 1000.0f, hz);  // high cutoff so the PT1 settles fast
    g = (pid_gains_t){.kp = 0.0f, .ki = 0.0f, .kd = 0.01f, .kff = 0.0f, .i_limit = 0.3f};
    pid_update(&p, &g, 0.0f, 0.0f, dt, true, false, false);
    out = pid_update(&p, &g, 10.0f, 0.0f, dt, true, false, false);
    CHECK(fabsf(out) < 1e-6f, "setpoint step kicked D: %f", (double)out);
    for (int i = 0; i < 200; i++) {
        out = pid_update(&p, &g, 0.0f, (float)i * 2.0f * dt, dt, true, false, false);  // meas rising 2/s
    }
    CHECK(fabsf(out + 0.02f) < 2e-3f, "D on a 2 rad/s^2 ramp: %f (want -0.02)", (double)out);
}

void test_ctrl(void)
{
    test_mixer_signs();
    test_mixer_saturation();
    test_pid();
}

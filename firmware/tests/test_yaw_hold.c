// Host tests for App/app/yaw_hold.c.
#include <math.h>

#include "app/yaw_hold.h"
#include "lib/mathx.h"
#include "test_common.h"

void test_yaw_hold(void)
{
    yaw_hold_t h;
    yaw_hold_reset(&h);
    const float kp = 2.5f;
    // Stick moved: plain rate command.
    CHECK(yaw_hold_step(&h, 1.0f, false, 0.3f, 1.0f, kp) == 1.0f && !h.holding, "stick: not a rate command");
    // Released while turning at 40 deg/s: brake, no capture yet.
    CHECK(yaw_hold_step(&h, 0.0f, true, 0.5f, 40.0f * MATHX_DEG2RAD, kp) == 0.0f && !h.holding, "captured while turning");
    // Slowed down: capture, zero command at the target.
    float sp = yaw_hold_step(&h, 0.0f, true, 0.6f, 0.05f, kp);
    CHECK(h.holding && fabsf(h.target - 0.6f) < 1e-6f && fabsf(sp) < 1e-6f, "capture: target %.3f sp %.3f",
          (double)h.target, (double)sp);
    // Pushed 10 deg clockwise: turns back (negative rate), proportional.
    sp = yaw_hold_step(&h, 0.0f, true, 0.6f + 10.0f * MATHX_DEG2RAD, 0.0f, kp);
    CHECK(fabsf(sp + kp * 10.0f * MATHX_DEG2RAD) < 1e-4f, "return rate %.3f", (double)sp);
    // Large error: limited.
    sp = yaw_hold_step(&h, 0.0f, true, 0.6f + 1.5f, 0.0f, kp);
    CHECK(fabsf(sp + YAW_HOLD_MAX_RATE) < 1e-6f, "not limited: %.3f", (double)sp);
    // Across +-180 deg: held 179, now -179 (= 181, 2 deg past): back 2 deg the short way, not 358 the long way.
    yaw_hold_reset(&h);
    (void)yaw_hold_step(&h, 0.0f, true, 179.0f * MATHX_DEG2RAD, 0.0f, kp);
    sp = yaw_hold_step(&h, 0.0f, true, -179.0f * MATHX_DEG2RAD, 0.0f, kp);
    CHECK(fabsf(sp + kp * 2.0f * MATHX_DEG2RAD) < 1e-3f, "wrap: %.3f", (double)sp);
    // kp 0: hold off.
    CHECK(yaw_hold_step(&h, 0.0f, true, 1.0f, 0.0f, 0.0f) == 0.0f && !h.holding, "kp 0 still holding");
}

// Heading hold (user, 2026-11-10: yaw drifted with rate-only yaw). Yaw stick centred: hold the heading reached
// once the turn has stopped (gyro-integrated yaw, drifts ~3-4 deg/min with the boot gyro calibration); stick
// moved: yaw rate from the stick as before. Idea as ArduPilot Stabilize / Betaflight, not copied. Pure C.
#pragma once

#include <stdbool.h>

#define YAW_HOLD_CAPTURE_RATE 0.1745329f  // rad/s (10 deg/s): stick released, brake until slower, then hold
#define YAW_HOLD_MAX_RATE     1.0471976f  // rad/s (60 deg/s): return-to-heading speed limit

typedef struct {
    bool holding;
    float target;  // rad
} yaw_hold_t;

void yaw_hold_reset(yaw_hold_t *h);
// Returns the yaw rate setpoint (rad/s). stick_rate: yaw stick * max_yaw_rate; centred: stick inside its
// dead band (exactly 0 from rc.c); yaw, yaw_rate: estimate (rad, rad/s); kp <= 0 turns the hold off.
float yaw_hold_step(yaw_hold_t *h, float stick_rate, bool centred, float yaw, float yaw_rate, float kp);

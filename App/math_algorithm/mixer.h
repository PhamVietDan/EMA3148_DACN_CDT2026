// Quad X mixer (PLAN §7.6, PX4/ArduPilot motor order) with desaturation that keeps roll/pitch over throttle.
#pragma once

#include <stdbool.h>

// Per axis (roll, pitch, yaw): the command was cut back because motors hit their limits.
// Sign tells which direction was limited, for rate-loop anti-windup.
typedef struct {
    bool pos[3];
    bool neg[3];
} mixer_sat_t;

// thr in [0,1]; torque roll/pitch/yaw in [-1,1] (normalized). Output motor[0..3] = M1..M4 in [idle, 1].
void mixer_quadx(float thr, const float torque[3], float idle, float motor[4], mixer_sat_t *sat);

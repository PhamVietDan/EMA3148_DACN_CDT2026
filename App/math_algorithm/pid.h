// Rate PID (PLAN §7.5; structure as PX4 rate_control): D on measurement through a PT1, integrator clamped
// and frozen in the direction the mixer is saturated.
#pragma once

#include <stdbool.h>

#include "math_algorithm/filter.h"

typedef struct {
    float kp, ki, kd, kff;  // output = normalized torque per rad/s (kff per rad/s of setpoint)
    float i_limit;          // |I| <= i_limit
} pid_gains_t;

typedef struct {
    float integ;
    float prev_meas_filt;
    bool primed;
    pt1_t dterm_lpf;
} pid_ctrl_t;

void pid_init(pid_ctrl_t *p, float dterm_cutoff_hz, float loop_hz);
void pid_reset(pid_ctrl_t *p);
// sat_pos / sat_neg: the mixer limited this axis last cycle in that direction.
float pid_update(pid_ctrl_t *p, const pid_gains_t *g, float sp, float meas, float dt, bool integrate,
                 bool sat_pos, bool sat_neg);

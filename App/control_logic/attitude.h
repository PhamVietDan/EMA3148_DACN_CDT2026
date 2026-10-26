// Attitude estimate: one Kalman 1D per axis (PLAN §7.3). Roll/pitch corrected by accel; yaw gyro-only until M11.
#pragma once

#include "control_logic/shared_state.h"

// Accel-only angles in body FRD (at rest a = (0, 0, -g)).
void attitude_acc_angles(const float acc[3], float *roll, float *pitch);
// Vertical (up) specific force minus g, using the given angles (PLAN §7.4).
float attitude_acc_up(const float acc[3], float roll, float pitch);

void attitude_init(const float acc[3]);
void attitude_set_sigmas(float sigma_gyro_rad_s, float sigma_acc_rad);  // task_ctrl, between updates
// rate: bias-corrected body rates (rad/s); acc: body accel (m/s^2); dt in s.
void attitude_update(attitude_t *out, const float rate[3], const float acc[3], float dt);

// 1-state Kalman filter for one attitude angle (Carbon Aeronautics form, PLAN §7.3).
#pragma once

typedef struct {
    float angle;  // rad
    float P;      // rad^2
} kalman1d_t;

void kalman1d_init(kalman1d_t *kf, float angle0, float P0);
// rate in rad/s; sigma_rate in rad/s. Q = (dt * sigma_rate)^2.
void kalman1d_predict(kalman1d_t *kf, float rate, float dt, float sigma_rate);
// R = sigma_meas^2 (not scaled by dt^2: that was a misprint in the source document).
void kalman1d_update(kalman1d_t *kf, float meas, float sigma_meas);
// Same as update, but the innovation is wrapped to [-pi, pi] (yaw).
void kalman1d_update_wrapped(kalman1d_t *kf, float meas, float sigma_meas);

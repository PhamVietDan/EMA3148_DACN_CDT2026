// 2-state altitude Kalman filter (PLAN §7.4, Carbon Aeronautics form): x = [z, vz] (up positive, m, m/s),
// input u = vertical acceleration (gravity and bias removed), measurement z (ToF + terrain offset).
#pragma once

#include <stdbool.h>

typedef struct {
    float z, vz;
    float P[2][2];
} alt_kf_t;

void alt_kf_init(alt_kf_t *k, float z0, float p0_z, float p0_vz);
void alt_kf_predict(alt_kf_t *k, float a_up, float dt, float sigma_a);
// Outlier gate: rejected (returns false, state unchanged) when (z_meas - z)^2 / S > gate_sq.
bool alt_kf_update(alt_kf_t *k, float z_meas, float sigma_meas, float gate_sq);

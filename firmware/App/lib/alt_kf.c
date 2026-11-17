// 2-state altitude Kalman filter (PLAN §7.4).
#include "lib/alt_kf.h"

void alt_kf_init(alt_kf_t *k, float z0, float p0_z, float p0_vz)
{
    k->z = z0;
    k->vz = 0.0f;
    k->P[0][0] = p0_z;
    k->P[0][1] = k->P[1][0] = 0.0f;
    k->P[1][1] = p0_vz;
}

// x = F x + G u, P = F P F^T + Q with F = [[1, dt], [0, 1]], G = [dt^2/2, dt], Q = G G^T sigma_a^2.
void alt_kf_predict(alt_kf_t *k, float a_up, float dt, float sigma_a)
{
    float g0 = 0.5f * dt * dt, g1 = dt;
    k->z += k->vz * dt + g0 * a_up;
    k->vz += g1 * a_up;
    float p00 = k->P[0][0], p01 = k->P[0][1], p10 = k->P[1][0], p11 = k->P[1][1];
    float q = sigma_a * sigma_a;
    k->P[0][0] = p00 + dt * (p10 + p01) + dt * dt * p11 + g0 * g0 * q;
    k->P[0][1] = p01 + dt * p11 + g0 * g1 * q;
    k->P[1][0] = p10 + dt * p11 + g0 * g1 * q;
    k->P[1][1] = p11 + g1 * g1 * q;
}

bool alt_kf_update(alt_kf_t *k, float z_meas, float sigma_meas, float gate_sq)
{
    float s = k->P[0][0] + sigma_meas * sigma_meas;
    float innov = z_meas - k->z;
    if (innov * innov > gate_sq * s) {
        return false;
    }
    float k0 = k->P[0][0] / s, k1 = k->P[1][0] / s;
    k->z += k0 * innov;
    k->vz += k1 * innov;
    // P = (I - K H) P, then symmetrised.
    float p00 = k->P[0][0], p01 = k->P[0][1], p10 = k->P[1][0], p11 = k->P[1][1];
    k->P[0][0] = (1.0f - k0) * p00;
    k->P[0][1] = (1.0f - k0) * p01;
    k->P[1][0] = p10 - k1 * p00;
    k->P[1][1] = p11 - k1 * p01;
    float sym = 0.5f * (k->P[0][1] + k->P[1][0]);
    k->P[0][1] = k->P[1][0] = sym;
    return true;
}

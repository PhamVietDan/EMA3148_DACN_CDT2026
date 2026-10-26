// Start-up calibration (PLAN §7.1): averages gyro/accel while still; restarts if motion is seen.
#include "control_logic/calib.h"

#include <math.h>

void calib_start(calib_acc_t *c, uint32_t n_target, float gyro_range_max, float acc_range_max)
{
    *c = (calib_acc_t){0};
    c->n_target = n_target;
    c->gyro_range_max = gyro_range_max;
    c->acc_range_max = acc_range_max;
}

calib_step_t calib_add(calib_acc_t *c, const float gyro[3], const float acc[3])
{
    float acc_norm = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    if (c->n == 0u) {
        for (int i = 0; i < 3; i++) {
            c->gyro_min[i] = c->gyro_max[i] = gyro[i];
        }
        c->acc_norm_min = c->acc_norm_max = acc_norm;
    }
    for (int i = 0; i < 3; i++) {
        c->gyro_sum[i] += gyro[i];
        c->acc_sum[i] += acc[i];
        c->gyro_min[i] = fminf(c->gyro_min[i], gyro[i]);
        c->gyro_max[i] = fmaxf(c->gyro_max[i], gyro[i]);
        if (c->gyro_max[i] - c->gyro_min[i] > c->gyro_range_max) {
            return CALIB_STEP_MOTION;
        }
    }
    c->acc_norm_min = fminf(c->acc_norm_min, acc_norm);
    c->acc_norm_max = fmaxf(c->acc_norm_max, acc_norm);
    if (c->acc_norm_max - c->acc_norm_min > c->acc_range_max) {
        return CALIB_STEP_MOTION;
    }
    return (++c->n >= c->n_target) ? CALIB_STEP_DONE : CALIB_STEP_BUSY;
}

void calib_result(const calib_acc_t *c, float gyro_bias[3], float acc_mean[3])
{
    float inv_n = 1.0f / (float)c->n;
    for (int i = 0; i < 3; i++) {
        gyro_bias[i] = c->gyro_sum[i] * inv_n;
        acc_mean[i] = c->acc_sum[i] * inv_n;
    }
}

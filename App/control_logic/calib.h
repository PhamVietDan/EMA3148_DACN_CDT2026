// Start-up calibration (PLAN §7.1): averages gyro/accel while still; restarts if motion is seen.
#pragma once

#include <stdint.h>

typedef enum { CALIB_STEP_BUSY, CALIB_STEP_DONE, CALIB_STEP_MOTION } calib_step_t;

typedef struct {
    uint32_t n, n_target;
    float gyro_sum[3], acc_sum[3];
    float gyro_min[3], gyro_max[3];
    float acc_norm_min, acc_norm_max;
    float gyro_range_max, acc_range_max;
} calib_acc_t;

void calib_start(calib_acc_t *c, uint32_t n_target, float gyro_range_max, float acc_range_max);
calib_step_t calib_add(calib_acc_t *c, const float gyro[3], const float acc[3]);
void calib_result(const calib_acc_t *c, float gyro_bias[3], float acc_mean[3]);

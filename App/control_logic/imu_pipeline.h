// Per-sample IMU processing in the DMA-complete ISR (3200 Hz); wakes task_ctrl every 3rd sample.
#pragma once

#define IMU_SAMPLES_PER_CTRL 3u

void imu_pipeline_start(float gyro_lpf_hz, float acc_lpf_hz);
void imu_pipeline_set_filters(float gyro_lpf_hz, float acc_lpf_hz);

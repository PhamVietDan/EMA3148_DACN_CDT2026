// The three RTOS tasks (PLAN §4.2): task_ctrl (Realtime), task_slow (High), task_telem (BelowNormal).
#pragma once

void task_ctrl(void const *argument);   // woken by the IMU ISR every 3 samples: calibration, attitude KF
void task_slow(void const *argument);   // ~100 Hz: altitude, ToF, baro, mag, RC (later milestones)
void task_telem(void const *argument);  // 50 Hz: telemetry, commands, logs; the only USB transmitter

void task_telem_capture_reset_cause(void);  // call once at boot, before anything clears RCC->CSR
void params_boot_load(void);  // app_init, before the scheduler: defaults + flash image

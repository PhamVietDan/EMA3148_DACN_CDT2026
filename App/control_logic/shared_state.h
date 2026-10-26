// Data shared between tasks/ISRs. One writer per object; whole-struct copies inside a critical section (PLAN §3.4).
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "cmsis_os.h"

#include "control_logic/params.h"

#define SIG_IMU    0x0001
#define SIG_TOF    0x0002
#define SIG_USB_RX 0x0004

extern osThreadId ctrlTaskHandle;
extern osThreadId slowTaskHandle;

// t_cyc: DWT cycles at the data-ready edge (diff-only, wraps ~25.5 s).
// Body frame FRD, low-passed (gyro biquad, accel PT1), gyro bias NOT yet removed.
typedef struct { uint32_t t_cyc; float gyro[3]; float acc[3]; } imu_sample_t;  // rad/s, m/s^2

// rate: bias-corrected body rates p, q, r. P: KF variance per axis (rad^2).
typedef struct { uint32_t t_cyc; float roll, pitch, yaw; float P[3]; float rate[3]; } attitude_t;

// Sticks normalized: roll/pitch/yaw in [-1, 1], thr in [0, 1]. Switches only meaningful while valid.
typedef struct {
    uint32_t t_ms;           // tick of the last decoded frame
    uint32_t age_ms;         // since the last good (non-failsafe) frame; UINT32_MAX if none yet
    float roll, pitch, yaw, thr;
    bool arm_sw;             // CH5 in the ARM position (up)
    uint8_t mode_sw;         // CH6: 0 = up (Angle), 1 = middle (Alt Hold), 2 = down (reserved)
    uint16_t raw[16];        // SBUS 0..2047
    uint32_t frames_lost;    // receiver "frame lost" flags seen
    bool failsafe_flag;      // receiver-reported failsafe
    bool valid;
} rc_t;

// Arm/motor status published by task_ctrl every cycle.
typedef struct {
    uint8_t armed;
    uint8_t motor_state;     // motor_state_t (control_logic/arming.h)
    uint8_t fs;              // failsafe_stage_t
    uint8_t mode;            // rc mode_sw at the time
    uint32_t block_mask;     // ARM_BLK_* currently failing
    float thr_cmd;           // [0,1]
    float motor[4];          // M1..M4 command [0,1]; ESC pulse = 900 + 1100 * m us (0 when stopped)
    float roll_sp, pitch_sp; // rad, angle setpoints of the attitude loop
    float yawr_sp;           // rad/s
    uint8_t esc_wd_tripped;
} arm_status_t;

typedef enum { CALIB_WAITING_IMU, CALIB_RUNNING, CALIB_DONE } calib_state_t;

typedef struct {
    uint8_t state;           // calib_state_t
    uint16_t restarts;       // motion detected during calibration
    float gyro_bias[3];      // rad/s, body
    float acc_mean[3];       // m/s^2, body
    float acc_up_bias0;      // m/s^2, vertical accel offset at rest (altitude KF, M9)
} calib_info_t;

typedef struct {
    uint32_t samples;
    uint32_t overruns;
    uint32_t spi_errors;
} imu_stats_t;

typedef struct {
    int32_t init_status;  // bmi323_status_t; 1 = not run yet
    uint16_t chip_id;
    uint16_t err_reg;
    uint16_t acc_conf;
    uint16_t gyr_conf;
} imu_info_t;

typedef struct {
    uint32_t cycles;
    uint32_t imu_timeouts;
    uint32_t exec_us_max;    // longest task_ctrl body since warm-up
    uint32_t jitter_us_max;  // max |period - nominal| since warm-up
    uint8_t imu_ok;          // 0 when IMU data stopped (~20 ms) or init failed; arming must require 1
} ctrl_stats_t;

typedef struct {
    uint32_t cycles;
} slow_stats_t;

// The IMU ISR (priority 5) is masked by taskENTER_CRITICAL, so task-side copies never see a torn write.
extern volatile imu_sample_t g_imu_sample;  // writer: IMU DMA ISR
extern volatile imu_stats_t g_imu_stats;    // writer: IMU EXTI/DMA ISR
extern volatile imu_info_t g_imu_info;      // writer: task_ctrl
extern volatile ctrl_stats_t g_ctrl_stats;  // writer: task_ctrl
extern volatile attitude_t g_attitude;      // writer: task_ctrl
extern volatile calib_info_t g_calib;       // writer: task_ctrl
extern volatile slow_stats_t g_slow_stats;  // writer: task_slow
extern volatile rc_t g_rc;                  // writer: task_slow
extern volatile arm_status_t g_arm;         // writer: task_ctrl
extern volatile uint32_t g_gcs_disarm_req;  // writer: task_telem (incremented per DISARM command)

// GCS motor test request: task_ctrl runs it once per new seq, only while disarmed.
typedef struct { uint32_t seq; uint8_t esc_ch; } motor_test_req_t;  // esc_ch 0..3 = TIM3 CH1..CH4
extern volatile motor_test_req_t g_motor_test_req;  // writer: task_telem

// Active parameter set. Single writer: task_telem (SET), or app_init before the scheduler. The struct is
// written before the sequence number, so a reader that sees a new seq copies the new values.
extern volatile params_t g_params;
extern volatile uint32_t g_params_seq;

// Task context only. Copies are a few dozen bytes, well under 1 µs with interrupts masked.
void shared_read(void *dst, const volatile void *src, size_t len);
void shared_write(volatile void *dst, const void *src, size_t len);

// Data shared between tasks/ISRs. One writer per object; whole-struct copies inside a critical section (PLAN §3.4).
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "cmsis_os.h"

#include "app/alt_est.h"
#include "app/params.h"
#include "drivers/ubx.h"

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
    uint8_t mode_sw;         // CH6: 0 = up (Alt Hold), 1 = middle, 2 = down (both Angle)
    uint16_t raw[16];        // SBUS 0..2047
    uint32_t frames_lost;    // receiver "frame lost" flags seen
    bool failsafe_flag;      // receiver-reported failsafe
    bool valid;
} rc_t;

// Arm/motor status published by task_ctrl every cycle.
#define ARM_MODE_ANGLE    0u
#define ARM_MODE_ALT_HOLD 1u
#define ARM_MODE_POS_HOLD 2u  // Alt Hold + position hold
#define ARM_MODE_LAND     3u  // failsafe or commanded (LAND) landing
typedef struct {
    uint8_t armed;
    uint8_t motor_state;     // motor_state_t (app/arming.h)
    uint8_t fs;              // failsafe_stage_t
    uint8_t mode;            // ARM_MODE_*: the mode flying now (CH6 up alone is not Alt Hold: it may be refused)
    uint8_t acro;            // always 0 (Acro removed 2026-10-29; field kept for the T line format)
    uint32_t block_mask;     // ARM_BLK_* currently failing
    float thr_cmd;           // [0,1], to the mixer (Alt Hold output while engaged)
    float z_sp;              // m, Alt Hold setpoint (0 when not engaged)
    float motor[4];          // M1..M4 command [0,1]; ESC pulse = 900 + 1100 * m us (0 when stopped)
    float roll_sp, pitch_sp; // rad, angle setpoints of the attitude loop
    float yawr_sp;           // rad/s
    float p_sp, q_sp;        // rad/s, roll/pitch rate setpoints fed to the rate PID
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
    uint32_t acc_clips, gyro_clips;  // samples with some axis near full scale (±16 g / ±2000 °/s)
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

// MTF-01P (PLAN §6.3). distance_m < 0 when the sensor reports out of range. valid: in range and the last
// rangefinder frame is < 50 ms old (the tilt < 30 deg condition is applied by the altitude estimator, M9).
typedef struct {
    uint32_t t_ms;           // tick of the last rangefinder frame
    float distance_m;
    uint8_t quality;         // 0..255, logged to learn what it means in practice
    bool valid;
    uint32_t flow_t_ms;      // tick of the last optical flow frame (logged only this phase)
    int32_t flow_x, flow_y;  // raw sensor units
    uint8_t flow_quality;
    uint32_t frames, crc_errors;
} tof_t;

// Vertical acceleration for the altitude filter (PLAN §7.4): up positive, gravity and the at-rest bias
// (calib acc_up_bias0) removed, mean over the last ACC_UP_BLOCK control cycles (~9.4 ms). seq counts blocks.
#define ACC_UP_BLOCK 10u
// a_h: horizontal acceleration of the heading frame (forward, right; attitude_acc_horizontal), same block mean.
typedef struct { uint32_t seq; float a_up; float a_h[2]; } acc_up_t;

// Horizontal velocity / position from optical flow (app/flow_est.h). Observation only (position hold step 2).
typedef struct {
    float v[2];              // m/s, heading frame: forward, right
    float meas[2];           // m/s, last flow velocity before filtering
    float pos[2];            // m, room frame (gyro yaw at boot), zeroed by POSZERO
    float acc_bias[2];       // m/s^2, accelerometer bias learnt by the flow filter (fwd, right)
    uint8_t quality;         // last MTF-01P flow quality
    bool valid;
} flow_out_t;

// IST8310 (PLAN §6.2), body FRD, hard-iron offsets (parameters mag_off_*) already removed. uT.
typedef struct {
    uint32_t t_ms;
    float field[3];
    bool valid;              // initialised and a sample within the last 200 ms
    int8_t init_status;      // ist8310_status_t
    uint32_t bus_errors;
} mag_t;

// DPS310 (PLAN §6.2b). alt_rel: relative to the mean pressure of the first ~2 s after start, PT1 ~2 Hz.
typedef struct {
    uint32_t t_ms;
    float pressure_pa, temp_c;
    float alt_rel_m;
    bool valid;              // initialised, reference taken, a sample within the last 200 ms
    int8_t init_status;      // dps310_status_t
    uint32_t bus_errors;
} baro_t;

// GEP-M10DI GPS, UBX NAV-PVT on USART3 (G1, 2026-11-14: observation only, nothing flies on it yet).
typedef struct {
    uint32_t t_ms;           // last NAV-PVT
    ubx_pvt_t pvt;
    bool fresh;              // a NAV-PVT within the last 500 ms
    uint32_t bytes, pvt_count, ck_errors, nmea_lines, acks, naks, cfg_sent;
} gps_t;

// The IMU ISR (priority 5) is masked by taskENTER_CRITICAL, so task-side copies never see a torn write.
extern volatile imu_sample_t g_imu_sample;  // writer: IMU DMA ISR
extern volatile imu_stats_t g_imu_stats;    // writer: IMU EXTI/DMA ISR
extern volatile imu_info_t g_imu_info;      // writer: task_ctrl
extern volatile ctrl_stats_t g_ctrl_stats;  // writer: task_ctrl
extern volatile attitude_t g_attitude;      // writer: task_ctrl
extern volatile calib_info_t g_calib;       // writer: task_ctrl
extern volatile slow_stats_t g_slow_stats;  // writer: task_slow
extern volatile rc_t g_rc;                  // writer: task_slow
extern volatile tof_t g_tof;                // writer: task_slow
extern volatile acc_up_t g_acc_up;          // writer: task_ctrl
extern volatile alt_out_t g_alt;            // writer: task_slow
extern volatile flow_out_t g_flow;          // writer: task_slow
extern volatile mag_t g_mag;                // writer: task_slow
extern volatile baro_t g_baro;              // writer: task_slow
extern volatile gps_t g_gps;                // writer: task_slow
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

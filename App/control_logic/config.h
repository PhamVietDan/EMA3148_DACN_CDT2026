// Compile-time constants. Tunable values (gains, filters, idle, deadband, KF sigmas) are in control_logic/params.c.
#pragma once

#include "math_algorithm/mathx.h"

// Sensor -> body FRD axis map, measured 2026-10-13 (PROGRESS.md): body[i] = sign[i] * sensor[src[i]].
#define CFG_IMU_AXIS_SRC  {0, 1, 2}
#define CFG_IMU_AXIS_SIGN {1.0f, -1.0f, -1.0f}

#define CFG_IMU_SAMPLE_HZ   3200.0f  // nominal ODR; the measured 3179 Hz shifts cutoffs by < 1 %

// RC (SBUS), MicroZone 6C-MINI. Channel indices are 0-based (CH1 = 0). Raw min/mid/max measured 2026-10-13
// with D/R at 100 % (legacy values were taken at D/R 50 %). Stick sign: roll right +, pitch back +, yaw right +.
#define CFG_RC_CH_ROLL   0
#define CFG_RC_CH_PITCH  1
#define CFG_RC_CH_THR    2
#define CFG_RC_CH_YAW    3
#define CFG_RC_ROLL_RAW  {200, 1000, 1800}
#define CFG_RC_PITCH_RAW {200, 1000, 1800}  // stick forward = 200 -> pitch -1 (checked on GCS)
#define CFG_RC_YAW_RAW   {200, 1000, 1800}
#define CFG_RC_THR_RAW   {168, 1768}
// Stick direction (+1 / -1), checked on GCS 2026-10-13 with transmitter mixing off: CH1 raw falls when moved right.
#define CFG_RC_ROLL_DIR  (-1.0f)
#define CFG_RC_PITCH_DIR (1.0f)
#define CFG_RC_YAW_DIR   (1.0f)
#define CFG_RC_TIMEOUT_MS 200u    // PLAN §6.4
// This receiver never sets the SBUS failsafe flag: on link loss it keeps sending frames with
// CH3 = 40 (~900 us) and other channels = 1000. Normal throttle never goes below ~200.
#define CFG_RC_FS_THR_RAW 100u  // failsafe 40, stick minimum 168
// 3-position switches: up = 200, middle = 1000, down = 1800 (user choice 2026-10-13).
#define CFG_RC_CH_ARM      4     // CH5: up = ARMED; middle/down = DISARMED (receiver failsafe -> middle)
#define CFG_RC_CH_MODE     5     // CH6: up = Angle, middle = Alt Hold (M10), down = reserved
#define CFG_RC_SW_LOW_MAX  600u  // raw below this = switch up
#define CFG_RC_SW_HIGH_MIN 1400u // raw above this = switch down

// Arming (PLAN §7.7; ArduPilot: 200 ms switch debounce, ARMING checks). Arm only by CH5 + throttle minimum.
#define CFG_ARM_SW_DEBOUNCE_MS  200u
#define CFG_ARM_STICK_HOLD_MS   3000u
#define CFG_ARM_STICK_YAW       0.9f    // yaw stick beyond -this, held CFG_ARM_STICK_HOLD_MS = stick disarm
#define CFG_ARM_THR_MAX         0.05f   // "throttle at minimum": below 5 % (arming and stick disarm)
#define CFG_ARM_STICK_CENTRE    0.2f    // roll/pitch sticks must be centred to arm
#define CFG_ARM_ANGLE_MAX       (15.0f * MATHX_DEG2RAD)
#define CFG_FLIP_ANGLE          (70.0f * MATHX_DEG2RAD)
#define CFG_FLIP_MS             300u
#define CFG_AUTO_DISARM_MS      15000u  // armed but throttle at minimum this long -> disarm
// RC-loss failsafe (PLAN §7.7; Betaflight-style 0.5 s of good signal to recover).
#define CFG_FS_STAGE1_MS        200u
#define CFG_FS_STAGE2_MS        1000u
#define CFG_FS_RECOVER_MS       500u
// Motor spool (ArduPilot GROUND_IDLE -> SPOOLING_UP -> THROTTLE_UNLIMITED, simplified).
#define CFG_THR_IDLE_ENTER      0.03f   // hysteresis between IDLE and spinning up
#define CFG_THR_IDLE_EXIT       0.05f
#define CFG_SPOOL_UP_MS         500u
// Motor command m in [0,1] -> ESC pulse 900 + m * 1100 us (idle: parameter motor_idle_us).
// Measured start-from-standstill threshold was ~1070 us; below it an armed motor may not start on its own.
#define CFG_CTRL_LOOP_HZ        1060.0f  // measured task_ctrl rate (3179 Hz IMU / 3), for filter design

// Motor test from the GCS (disarmed only, props off): one ESC channel at this pulse for this long.
// Above the ~1070 us start threshold so the motor starts from standstill.
#define CFG_MOTOR_TEST_US       1120u
#define CFG_MOTOR_TEST_MS       2000u
// Motor M1..M4 (PLAN §7.6 numbering) -> TIM3 channel index (0 = CH1/PA6/ESC1 ... 3 = CH4/PB1/ESC4).
// Motor test 2026-10-15: ESC1 rear-right CW, ESC2 front-right CCW, ESC3 rear-left CCW, ESC4 front-left CW
// => M1 = ESC2, M2 = ESC3, M3 = ESC4, M4 = ESC1. All spin directions match §7.6.
#define CFG_MOTOR_TO_TIM_CH     {1, 2, 3, 0}

// Start-up calibration (PLAN §7.1): vehicle still and level.
#define CFG_CALIB_TIME_S      2.5f
#define CFG_CALIB_GYRO_RANGE  0.10f  // rad/s peak-to-peak per axis; above this -> restart
#define CFG_CALIB_ACC_RANGE   1.0f   // m/s^2 peak-to-peak of |a|; above this -> restart

// Attitude Kalman 1D (PLAN §7.3); sigmas are parameters kf_sg / kf_sacc.
#define CFG_KF_P0          ((2.0f * MATHX_DEG2RAD) * (2.0f * MATHX_DEG2RAD))
#define CFG_KF_ACC_MIN_G   0.9f   // accel update only when 0.9 g < |a| < 1.1 g
#define CFG_KF_ACC_MAX_G   1.1f
#define CFG_KF_EULER_RATES 0      // 1 = convert body rates to Euler rates before integrating
#define CFG_KF_EVERY_CTRL  2u     // attitude KF runs every 2nd task_ctrl cycle (~530 Hz)

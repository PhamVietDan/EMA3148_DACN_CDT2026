// Arm/disarm state machine, RC-loss failsafe and motor spool state (PLAN §7.7; ideas from ArduPilot
// AP_Arming / RC_Channels::rudder_arm_disarm_check / AP_MotorsMulticopter::output_logic, not copied).
// Pure C: no HAL/RTOS, unit-tested on the PC. Runs in task_ctrl.
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t now_ms;
    bool rc_valid;
    float rc_roll, rc_pitch, rc_yaw, rc_thr;  // [-1,1], thr [0,1]
    bool arm_sw;                               // CH5 in the ARM position (only used while rc_valid)
    bool imu_ok;
    bool calib_done;
    float roll, pitch;                         // rad, estimated attitude
    bool gcs_disarm;                           // one-shot DISARM command from the ground station
    bool esc_wd_tripped;
    // Loss-of-control check (idea from ArduPilot crash_check): max(|roll - roll_sp|, |pitch - pitch_sp|)
    // in rad, evaluated only while the motors are spinning up or running.
    float att_err;
    float lc_angle;                            // rad
    uint32_t lc_ms;
} arming_in_t;

typedef enum { MOTOR_STOP, MOTOR_IDLE, MOTOR_SPOOL_UP, MOTOR_RUN } motor_state_t;
typedef enum { FS_NONE, FS_STAGE1, FS_STAGE2 } failsafe_stage_t;

typedef enum {
    ARM_METHOD_SWITCH = 1,
} arm_method_t;

typedef enum {
    DISARM_SWITCH = 1,
    DISARM_STICK,
    DISARM_GCS,
    DISARM_FAILSAFE,
    DISARM_FLIP,
    DISARM_IMU,
    DISARM_AUTO,     // left at idle throttle too long
    DISARM_ESC_WD,   // motor output watchdog fired (task_ctrl stalled)
    DISARM_LOST_CONTROL,
} disarm_reason_t;

// Arm checks; any set bit blocks arming.
enum {
    ARM_BLK_SWITCH       = 1u << 0,  // switch not in ARM (status only: arming happens on the edge into ARM)
    ARM_BLK_SWITCH_CYCLE = 1u << 1,  // switch must go to DISARM once after a failsafe/fault disarm
    ARM_BLK_RC           = 1u << 2,
    ARM_BLK_THROTTLE     = 1u << 3,
    ARM_BLK_STICKS       = 1u << 4,  // roll/pitch sticks not centred
    ARM_BLK_IMU          = 1u << 5,
    ARM_BLK_CALIB        = 1u << 6,
    ARM_BLK_ANGLE        = 1u << 7,
    ARM_BLK_FAILSAFE     = 1u << 8,
    ARM_BLK_ESC_WD       = 1u << 9,
};

typedef enum {
    ARM_EV_NONE,
    ARM_EV_ARMED,        // arg: arm_method_t
    ARM_EV_DISARMED,     // arg: disarm_reason_t
    ARM_EV_BLOCKED,      // arm attempt rejected; see block_mask
    ARM_EV_FS_STAGE1,
    ARM_EV_FS_RECOVERED,
} arming_event_t;

typedef struct {
    bool armed;
    motor_state_t motor_state;
    failsafe_stage_t fs;
    float roll_cmd, pitch_cmd, yaw_cmd;  // stick commands after failsafe override, [-1,1]
    float thr_cmd;                       // [0,1]
    float thr_ceiling;                   // spool-up limit on thr_cmd, [0,1]
    uint32_t block_mask;
    arming_event_t event;
    uint8_t event_arg;
} arming_out_t;

typedef struct {
    bool armed;
    bool need_switch_cycle;
    // Debounced switch (ArduPilot acts on settled position changes only; boot position is not an edge).
    bool sw_known, sw_stable, sw_candidate;
    uint32_t sw_candidate_ms;
    // Stick disarm gesture
    uint32_t stick_disarm_ms;
    bool stick_disarm_timing;
    // Failsafe
    failsafe_stage_t fs;
    bool rc_lost;
    uint32_t rc_lost_ms, rc_good_ms;
    float last_roll, last_pitch, last_yaw, last_thr;
    // Flip, auto-disarm, spool
    bool flip_timing;
    uint32_t flip_ms;
    bool lc_timing;
    uint32_t lc_start_ms;
    motor_state_t motor_state;
    uint32_t idle_since_ms, spool_start_ms;
} arming_t;

void arming_init(arming_t *a);
void arming_update(arming_t *a, const arming_in_t *in, arming_out_t *out);

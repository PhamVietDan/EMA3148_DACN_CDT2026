// Arm/disarm state machine, RC-loss failsafe and motor spool state (PLAN §7.7; ideas from ArduPilot
// AP_Arming / RC_Channels::rudder_arm_disarm_check / AP_MotorsMulticopter::output_logic, not copied).
// Pure C: no HAL/RTOS, unit-tested on the PC. Runs in task_ctrl.
//
// Arming (user decision): only on the CH5 edge into ARM, with the throttle stick at minimum and all
// arm checks passing. Leaving ARM always disarms. Disarm by stick (throttle minimum + yaw left 3 s) is kept.
#include "control_logic/arming.h"

#include <math.h>

#include "control_logic/config.h"

void arming_init(arming_t *a)
{
    *a = (arming_t){0};
    a->motor_state = MOTOR_STOP;
}

static void set_event(arming_out_t *out, arming_event_t ev, uint8_t arg)
{
    out->event = ev;
    out->event_arg = arg;
}

// An RC-loss episode starts at the first invalid sample and ends after CFG_FS_RECOVER_MS of continuous
// valid RC; stages count from the start, so a flickering link cannot postpone stage 2.
static void update_failsafe(arming_t *a, const arming_in_t *in, arming_out_t *out)
{
    uint32_t now = in->now_ms;
    if (!in->rc_valid) {
        if (!a->rc_lost) {
            a->rc_lost = true;
            a->rc_lost_ms = now;
        }
        a->rc_good_ms = now;
    } else if (a->rc_lost && now - a->rc_good_ms >= CFG_FS_RECOVER_MS) {
        a->rc_lost = false;
        if (a->fs != FS_NONE && a->armed) {
            set_event(out, ARM_EV_FS_RECOVERED, 0);
        }
        a->fs = FS_NONE;
    }
    if (a->rc_lost) {
        uint32_t lost_for = now - a->rc_lost_ms;
        failsafe_stage_t stage = (lost_for >= CFG_FS_STAGE2_MS) ? FS_STAGE2
                               : (lost_for >= CFG_FS_STAGE1_MS) ? FS_STAGE1 : FS_NONE;
        if (stage == FS_STAGE1 && a->fs == FS_NONE && a->armed) {
            set_event(out, ARM_EV_FS_STAGE1, 0);
        }
        if (stage > a->fs) {
            a->fs = stage;
        }
    }
}

static void update_switch(arming_t *a, const arming_in_t *in, bool *edge_on, bool *edge_off)
{
    *edge_on = false;
    *edge_off = false;
    if (!in->rc_valid) {
        a->sw_candidate = a->sw_stable;  // receiver failsafe values (CH5 = middle) must not act as a switch move
        return;
    }
    uint32_t now = in->now_ms;
    if (!a->sw_known) {
        // Position at power-up / first valid frame is recorded, not acted on.
        a->sw_known = true;
        a->sw_stable = a->sw_candidate = in->arm_sw;
        a->sw_candidate_ms = now;
    } else if (in->arm_sw != a->sw_candidate) {
        a->sw_candidate = in->arm_sw;
        a->sw_candidate_ms = now;
    } else if (a->sw_candidate != a->sw_stable && now - a->sw_candidate_ms >= CFG_ARM_SW_DEBOUNCE_MS) {
        a->sw_stable = a->sw_candidate;
        *edge_on = a->sw_stable;
        *edge_off = !a->sw_stable;
    }
    if (!a->sw_stable) {
        a->need_switch_cycle = false;
    }
}

static void update_sticks(arming_t *a, const arming_in_t *in, bool *disarm_req)
{
    *disarm_req = false;
    if (!in->rc_valid) {
        a->stick_disarm_timing = false;
        return;
    }
    uint32_t now = in->now_ms;
    bool thr_low = in->rc_thr < CFG_ARM_THR_MAX;

    // Stick disarm only at minimum throttle, so it cannot cut motors in flight (ArduPilot rule).
    if (a->armed && thr_low && in->rc_yaw < -CFG_ARM_STICK_YAW) {
        if (!a->stick_disarm_timing) {
            a->stick_disarm_timing = true;
            a->stick_disarm_ms = now;
        } else if (now - a->stick_disarm_ms >= CFG_ARM_STICK_HOLD_MS) {
            *disarm_req = true;
            a->stick_disarm_timing = false;
        }
    } else {
        a->stick_disarm_timing = false;
    }
}

static uint32_t arm_checks(const arming_t *a, const arming_in_t *in)
{
    uint32_t m = 0;
    if (a->need_switch_cycle) {
        m |= ARM_BLK_SWITCH_CYCLE;
    }
    if (!in->rc_valid) {
        m |= ARM_BLK_RC;
    } else {
        if (!a->sw_known || !a->sw_stable) {
            m |= ARM_BLK_SWITCH;
        }
        if (in->rc_thr >= CFG_ARM_THR_MAX) {
            m |= ARM_BLK_THROTTLE;
        }
        if (fabsf(in->rc_roll) > CFG_ARM_STICK_CENTRE || fabsf(in->rc_pitch) > CFG_ARM_STICK_CENTRE) {
            m |= ARM_BLK_STICKS;
        }
    }
    if (!in->imu_ok) {
        m |= ARM_BLK_IMU;
    }
    if (!in->calib_done) {
        m |= ARM_BLK_CALIB;
    } else if (fabsf(in->roll) > CFG_ARM_ANGLE_MAX || fabsf(in->pitch) > CFG_ARM_ANGLE_MAX) {
        m |= ARM_BLK_ANGLE;
    }
    if (a->fs != FS_NONE || a->rc_lost) {
        m |= ARM_BLK_FAILSAFE;
    }
    if (in->esc_wd_tripped) {
        m |= ARM_BLK_ESC_WD;
    }
    return m;
}

static void do_arm(arming_t *a, uint32_t now, arming_out_t *out, arm_method_t method)
{
    a->armed = true;
    a->motor_state = MOTOR_IDLE;
    a->idle_since_ms = now;
    a->flip_timing = false;
    a->lc_timing = false;
    a->stick_disarm_timing = false;
    set_event(out, ARM_EV_ARMED, (uint8_t)method);
}

static void do_disarm(arming_t *a, arming_out_t *out, disarm_reason_t reason)
{
    a->armed = false;
    a->motor_state = MOTOR_STOP;
    // Plan §7.7: after an automatic disarm the pilot must cycle the switch before arming again.
    if (reason != DISARM_SWITCH && reason != DISARM_STICK && reason != DISARM_AUTO) {
        a->need_switch_cycle = true;
    }
    set_event(out, ARM_EV_DISARMED, (uint8_t)reason);
}

static bool flip_detected(arming_t *a, const arming_in_t *in)
{
    bool tilted = in->calib_done && (fabsf(in->roll) > CFG_FLIP_ANGLE || fabsf(in->pitch) > CFG_FLIP_ANGLE);
    if (!tilted) {
        a->flip_timing = false;
        return false;
    }
    if (!a->flip_timing) {
        a->flip_timing = true;
        a->flip_ms = in->now_ms;
    }
    return in->now_ms - a->flip_ms >= CFG_FLIP_MS;
}

// Attitude far from its setpoint for lc_ms while flying: the controller cannot pull the vehicle back
// (wrong gains or signs, motor failure). Brief gusts / overshoots shorter than lc_ms do not trip it.
static bool lost_control(arming_t *a, const arming_in_t *in)
{
    bool flying = a->motor_state == MOTOR_SPOOL_UP || a->motor_state == MOTOR_RUN;
    if (!flying || !in->calib_done || in->att_err <= in->lc_angle) {
        a->lc_timing = false;
        return false;
    }
    if (!a->lc_timing) {
        a->lc_timing = true;
        a->lc_start_ms = in->now_ms;
    }
    return in->now_ms - a->lc_start_ms >= in->lc_ms;
}

static void update_commands(arming_t *a, const arming_in_t *in, arming_out_t *out)
{
    // A glitch shorter than stage 1 only freezes the sticks while it lasts.
    if (in->rc_valid && a->fs == FS_NONE) {
        a->last_roll = in->rc_roll;
        a->last_pitch = in->rc_pitch;
        a->last_yaw = in->rc_yaw;
        a->last_thr = in->rc_thr;
    }
    if (a->fs == FS_NONE) {
        // Valid RC, or a gap shorter than stage 1: last valid sticks.
        out->roll_cmd = a->last_roll;
        out->pitch_cmd = a->last_pitch;
        out->yaw_cmd = a->last_yaw;
    } else {
        // Stage 1 (and while recovering): level, no yaw, hold the last valid throttle.
        out->roll_cmd = 0.0f;
        out->pitch_cmd = 0.0f;
        out->yaw_cmd = 0.0f;
    }
    out->thr_cmd = a->last_thr;
}

static void update_spool(arming_t *a, uint32_t now, arming_out_t *out)
{
    switch (a->motor_state) {
    case MOTOR_STOP:
        out->thr_ceiling = 0.0f;
        break;
    case MOTOR_IDLE:
        out->thr_ceiling = 0.0f;
        if (out->thr_cmd > CFG_THR_IDLE_EXIT) {
            a->motor_state = MOTOR_SPOOL_UP;
            a->spool_start_ms = now;
        }
        break;
    case MOTOR_SPOOL_UP: {
        float ramp = (float)(now - a->spool_start_ms) / (float)CFG_SPOOL_UP_MS;
        out->thr_ceiling = (ramp >= 1.0f) ? 1.0f : ramp;
        if (ramp >= 1.0f) {
            a->motor_state = MOTOR_RUN;
        }
        break;
    }
    case MOTOR_RUN:
        out->thr_ceiling = 1.0f;
        break;
    }
    if ((a->motor_state == MOTOR_SPOOL_UP || a->motor_state == MOTOR_RUN) && out->thr_cmd < CFG_THR_IDLE_ENTER) {
        a->motor_state = MOTOR_IDLE;
        a->idle_since_ms = now;
        out->thr_ceiling = 0.0f;
    }
}

void arming_update(arming_t *a, const arming_in_t *in, arming_out_t *out)
{
    uint32_t now = in->now_ms;
    set_event(out, ARM_EV_NONE, 0);

    update_failsafe(a, in, out);
    bool sw_on, sw_off, stick_disarm;
    update_switch(a, in, &sw_on, &sw_off);
    update_sticks(a, in, &stick_disarm);
    bool flipped = a->armed && flip_detected(a, in);
    bool lost = a->armed && lost_control(a, in);

    if (a->armed) {
        disarm_reason_t reason = 0;
        if (!in->imu_ok) {
            reason = DISARM_IMU;  // user rule: no IMU, no motors
        } else if (in->esc_wd_tripped) {
            reason = DISARM_ESC_WD;
        } else if (in->gcs_disarm) {
            reason = DISARM_GCS;
        } else if (a->fs == FS_STAGE2) {
            reason = DISARM_FAILSAFE;  // no altitude estimate yet: stage 2 = disarm (PLAN §7.7)
        } else if (sw_off) {
            reason = DISARM_SWITCH;
        } else if (stick_disarm) {
            reason = DISARM_STICK;
        } else if (flipped) {
            reason = DISARM_FLIP;
        } else if (lost) {
            reason = DISARM_LOST_CONTROL;
        } else if (a->motor_state == MOTOR_IDLE && now - a->idle_since_ms >= CFG_AUTO_DISARM_MS) {
            reason = DISARM_AUTO;
        }
        if (reason != 0) {
            do_disarm(a, out, reason);
        }
    } else if (sw_on) {
        uint32_t blocked = arm_checks(a, in);
        if (blocked == 0u) {
            do_arm(a, now, out, ARM_METHOD_SWITCH);
        } else {
            set_event(out, ARM_EV_BLOCKED, ARM_METHOD_SWITCH);
        }
    }

    update_commands(a, in, out);
    update_spool(a, now, out);
    out->armed = a->armed;
    out->motor_state = a->motor_state;
    out->fs = a->fs;
    out->block_mask = arm_checks(a, in);
}

// All task bodies. Sensor ISR path: bsp/spi_imu.c -> app/imu_pipeline.c -> SIG_IMU -> task_ctrl.
// Estimation helpers live in app/calib.c, app/attitude.c; telemetry text in app/telem_proto.c.
#include "app/tasks.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app/alt_hold.h"
#include "app/arming.h"
#include "app/flow_est.h"
#include "app/pos_hold.h"
#include "app/attitude.h"
#include "app/blackbox.h"
#include "app/calib.h"
#include "app/capture.h"
#include "app/config.h"
#include "app/i2c_sensors.h"
#include "app/imu_pipeline.h"
#include "app/levelcal.h"
#include "app/link.h"
#include "app/log.h"
#include "app/rc.h"
#include "app/shared_state.h"
#include "app/telem_proto.h"
#include "app/yaw_hold.h"
#include "bsp/dbg_gpio.h"
#include "bsp/flash_params.h"
#include "bsp/flow_uart.h"
#include "bsp/gps_uart.h"
#include "bsp/pwm_esc.h"
#include "bsp/rc_uart.h"
#include "bsp/time_us.h"
#include "bsp/trace.h"
#include "bsp/usb_cdc_port.h"
#include "drivers/bmi323.h"
#include "drivers/mtf01p.h"
#include "lib/mixer.h"
#include "lib/numfmt.h"
#include "lib/pid.h"
#include "i2c.h"
#include "main.h"

extern osThreadId telemTaskHandle;  // created by CubeMX in freertos.c

/* ---- task_ctrl: IMU init, start-up calibration, attitude Kalman, arming, motor output ---- */

// 3 gyro samples at 3200 Hz. The BMI323 clock may differ from the MCU by up to ~1.7 %.
#define CTRL_NOMINAL_PERIOD_US 937u
#define CTRL_WARMUP_CYCLES     1000u
#define CTRL_IMU_TIMEOUT_MS    5u
// No MCU watchdog (user decision): IMU silence clears imu_ok, which disarms and blocks arming.
#define CTRL_IMU_LOST_TIMEOUTS 4u  // ~20 ms without data
#define CTRL_PRINT_PERIOD_MS   1000u

// Diagnostic switch: 0 = init the IMU over SPI but never start the EXTI/DMA stream (isolates hangs).
#define APP_IMU_STREAM 1

typedef enum { EST_MSG_NONE, EST_MSG_CALIB_MOTION } est_msg_t;

static calib_acc_t calib_acc;
static calib_info_t calib_info;
static uint32_t kf_divider;
static uint32_t kf_last_t_cyc;

// Calibration first (vehicle still), then bias-corrected rates feed the attitude KF every 2nd cycle.
static est_msg_t estimator_step(const imu_sample_t *s)
{
    if (calib_info.state != CALIB_DONE) {
        if (calib_info.state == CALIB_WAITING_IMU) {
            calib_start(&calib_acc, (uint32_t)(CFG_CALIB_TIME_S * CFG_IMU_SAMPLE_HZ / (float)IMU_SAMPLES_PER_CTRL),
                        CFG_CALIB_GYRO_RANGE, CFG_CALIB_ACC_RANGE);
            calib_info.state = CALIB_RUNNING;
        }
        calib_step_t step = calib_add(&calib_acc, s->gyro, s->acc);
        if (step == CALIB_STEP_MOTION) {
            calib_info.restarts++;
            calib_info.state = CALIB_WAITING_IMU;
            shared_write(&g_calib, &calib_info, sizeof calib_info);
            return EST_MSG_CALIB_MOTION;
        }
        if (step == CALIB_STEP_BUSY) {
            return EST_MSG_NONE;
        }
        calib_result(&calib_acc, calib_info.gyro_bias, calib_info.acc_mean);
        float roll0, pitch0;
        attitude_acc_angles(calib_info.acc_mean, &roll0, &pitch0);
        calib_info.acc_up_bias0 = attitude_acc_up(calib_info.acc_mean, roll0, pitch0);
        calib_info.state = CALIB_DONE;
        shared_write(&g_calib, &calib_info, sizeof calib_info);  // task_telem reports "CALIB done"
        attitude_init(calib_info.acc_mean);
        kf_last_t_cyc = s->t_cyc;
        kf_divider = 0;
        return EST_MSG_NONE;
    }

    if (++kf_divider < CFG_KF_EVERY_CTRL) {
        return EST_MSG_NONE;
    }
    kf_divider = 0;
    float dt = (float)(s->t_cyc - kf_last_t_cyc) / (float)(TIME_CYCLES_PER_US * 1000000u);
    kf_last_t_cyc = s->t_cyc;
    float rate[3];
    for (int i = 0; i < 3; i++) {
        rate[i] = s->gyro[i] - calib_info.gyro_bias[i];
    }
    attitude_t att;
    att.t_cyc = s->t_cyc;
    attitude_update(&att, rate, s->acc, dt);
    shared_write(&g_attitude, &att, sizeof att);
    return EST_MSG_NONE;
}

// Vertical acceleration block mean for the altitude filter (task_slow), every control cycle after calibration.
static void acc_up_step(const imu_sample_t *s)
{
    static float sum, sum_h[2];
    static uint32_t n;
    static acc_up_t out;
    attitude_t att;
    shared_read(&att, &g_attitude, sizeof att);
    sum += attitude_acc_up(s->acc, att.roll, att.pitch) - calib_info.acc_up_bias0;
    float ah[2];
    attitude_acc_horizontal(s->acc, att.roll, att.pitch, ah);
    sum_h[0] += ah[0];
    sum_h[1] += ah[1];
    if (++n >= ACC_UP_BLOCK) {
        out.a_up = sum / (float)n;
        out.a_h[0] = sum_h[0] / (float)n;
        out.a_h[1] = sum_h[1] / (float)n;
        out.seq++;
        shared_write(&g_acc_up, &out, sizeof out);
        sum = sum_h[0] = sum_h[1] = 0.0f;
        n = 0;
    }
}

static arming_t arming;
static uint32_t gcs_disarm_seen;

// GCS motor test (ArduPilot-style "motor test"): one ESC channel spins for CFG_MOTOR_TEST_MS while disarmed.
typedef enum { MTEST_MSG_NONE, MTEST_MSG_START, MTEST_MSG_DONE, MTEST_MSG_CANCELLED, MTEST_MSG_REFUSED } mtest_msg_t;
static uint32_t mtest_seq_seen;
static bool mtest_active;
static uint8_t mtest_ch;
static uint32_t mtest_end_ms;
static mtest_msg_t mtest_msg;
static const uint8_t motor_to_tim_ch[4] = CFG_MOTOR_TO_TIM_CH;

// Cascaded attitude control (PLAN §7.5): angle P -> rate PID (per axis) -> quad X mixer.
static pid_ctrl_t rate_pid[3];
static pid_gains_t rate_gains[3];
static mixer_sat_t mixer_sat;
static float motor_cmd[4];  // last M1..M4 command, held on cycles without a new IMU sample
static float sp_roll, sp_pitch, sp_yawr;
static float sp_p, sp_q;  // roll/pitch rate setpoints of the last cycle

// task_ctrl's copy of the parameters, refreshed at the start of a cycle when g_params_seq changes
// (PLAN §3.4). Angles/rates converted to rad once here.
static params_t prm;
static uint32_t prm_seq;
static float ang_p, max_angle, max_rate_rp, max_yaw_rate, motor_idle, lc_angle, trim_roll, trim_pitch, yaw_hold_p;
static yaw_hold_t yhold;
static uint32_t lc_ms;
static alt_hold_t ahold;
static alt_hold_cfg_t ah_cfg;
// Failsafe stage 2 / LAND command landing (app/alt_hold.c): touchdown seen this cycle, disarm on the next.
static bool land_touchdown;
static bool gcs_land;
static uint32_t gcs_land_seen;
static uint8_t gcs_land_mode_sw;
volatile uint32_t g_gcs_land_req;  // writer: task_telem (LAND)
enum { LAND_MSG_NONE, LAND_MSG_START, LAND_MSG_REFUSED, LAND_MSG_CANCELLED };
static uint8_t land_msg;
static pos_hold_t phold;
static pos_hold_cfg_t ph_cfg;
static pos_hold_out_t ph_out;
static uint8_t ph_event;  // pos_hold_event_t, cleared once printed
static float ph_event_pos[2];
static uint8_t ph_event_why;  // PH_EV_OFF: 0 Alt Hold off, 1 flow lost, 2 low or switched off, 3 touching down
static uint8_t ah_event;  // alt_hold_event_t, cleared once printed
static float ah_event_val;

// Gyro notch follows the mean motor command (prop vibration frequency rises with motor speed). Updated
// every NOTCH_UPDATE_CYCLES control cycles (~100 Hz) or when the parameters change.
#define NOTCH_UPDATE_CYCLES 10u
static uint32_t notch_div;
static float notch_hz_applied = -1.0f;

static void notch_track(void)
{
    if (++notch_div < NOTCH_UPDATE_CYCLES && notch_hz_applied >= 0.0f) {
        return;
    }
    notch_div = 0;
    float hz = 0.0f;
    if (prm.notch_lo_hz > 0.0f) {
        float m = 0.25f * (motor_cmd[0] + motor_cmd[1] + motor_cmd[2] + motor_cmd[3]);
        hz = prm.notch_lo_hz + (prm.notch_hi_hz - prm.notch_lo_hz) * clampf(m, 0.0f, 1.0f);
        hz = clampf(hz, 10.0f, 0.45f * CFG_IMU_SAMPLE_HZ);
    }
    if (fabsf(hz - notch_hz_applied) >= 0.5f) {
        imu_pipeline_set_notch(hz, prm.notch_q);
        notch_hz_applied = hz;
    }
}

static void ctrl_apply_params(bool first)
{
    uint32_t seq = g_params_seq;
    if (!first && seq == prm_seq) {
        return;
    }
    params_t old = prm;
    shared_read(&prm, &g_params, sizeof prm);
    prm_seq = seq;
    rate_gains[0] = (pid_gains_t){prm.rate_p_rp, prm.rate_i_rp, prm.rate_d_rp, 0.0f, prm.rate_ilim};
    rate_gains[1] = rate_gains[0];
    rate_gains[2] = (pid_gains_t){prm.rate_p_y, prm.rate_i_y, 0.0f, 0.0f, prm.rate_ilim};
    ang_p = prm.ang_p_rp;
    max_angle = prm.max_angle * MATHX_DEG2RAD;
    max_rate_rp = prm.max_rate_rp * MATHX_DEG2RAD;
    max_yaw_rate = prm.max_yaw_rate * MATHX_DEG2RAD;
    motor_idle = (prm.motor_idle_us - (float)ESC_STOP_US) / (float)(ESC_MAX_US - ESC_STOP_US);
    lc_angle = prm.lc_angle * MATHX_DEG2RAD;
    lc_ms = (uint32_t)prm.lc_time_ms;
    trim_roll = prm.trim_roll * MATHX_DEG2RAD;
    trim_pitch = prm.trim_pitch * MATHX_DEG2RAD;
    yaw_hold_p = prm.yaw_hold_p;
    ah_cfg = (alt_hold_cfg_t){prm.alt_p, prm.vz_p, prm.vz_i, prm.vz_max, prm.alt_thr_rng, prm.hover_thr,
                              prm.takeoff_alt};
    ph_cfg = (pos_hold_cfg_t){prm.pos_p, prm.vel_p * MATHX_DEG2RAD, prm.vel_i * MATHX_DEG2RAD, prm.vel_max,
                              prm.pos_ang_max * MATHX_DEG2RAD};
    notch_hz_applied = -1.0f;  // re-apply with the new notch parameters on the next cycle
    attitude_set_sigmas(prm.kf_sg * MATHX_DEG2RAD, prm.kf_sacc * MATHX_DEG2RAD);
    // Filter parameters are DISARMED_ONLY (refused by SET while armed), so a restart here is safe.
    if (first || prm.dterm_lpf_hz != old.dterm_lpf_hz) {
        for (int i = 0; i < 3; i++) {
            pid_init(&rate_pid[i], prm.dterm_lpf_hz, CFG_CTRL_LOOP_HZ);
        }
    }
    if (!first && (prm.gyro_lpf_hz != old.gyro_lpf_hz || prm.acc_lpf_hz != old.acc_lpf_hz)) {
        imu_pipeline_set_filters(prm.gyro_lpf_hz, prm.acc_lpf_hz);
    }
}

// rate: bias-corrected body rates (rad/s) of this cycle; att: latest attitude estimate; ph: position hold angles
// and their weight against the stick angles (0 outside position hold).
static void attitude_rate_control(const arming_out_t *o, const attitude_t *att, const float rate[3], float dt,
                                  float thr, const pos_hold_out_t *ph)
{
    // Stick centred (exactly 0 inside the dead band, and in failsafe): hold the heading.
    sp_yawr = yaw_hold_step(&yhold, o->yaw_cmd * max_yaw_rate, o->yaw_cmd == 0.0f, att->yaw, rate[2], yaw_hold_p);
    float w = ph->weight;
    sp_roll = (1.0f - w) * o->roll_cmd * max_angle + w * ph->roll_sp + ph->add_roll + trim_roll;
    sp_pitch = (1.0f - w) * o->pitch_cmd * max_angle + w * ph->pitch_sp + ph->add_pitch + trim_pitch;
    sp_p = clampf(ang_p * (sp_roll - att->roll), -max_rate_rp, max_rate_rp);
    sp_q = clampf(ang_p * (sp_pitch - att->pitch), -max_rate_rp, max_rate_rp);
    float rate_sp[3] = {sp_p, sp_q, sp_yawr};
    // I only in normal flight; spool-up / idle on the ground must not wind it up (PLAN §7.5, PX4 "landed").
    bool integrate = o->motor_state == MOTOR_RUN;
    float torque[3];
    for (int i = 0; i < 3; i++) {
        torque[i] = pid_update(&rate_pid[i], &rate_gains[i], rate_sp[i], rate[i], dt, integrate,
                               mixer_sat.pos[i], mixer_sat.neg[i]);
    }
    mixer_quadx(fminf(thr, o->thr_ceiling), torque, motor_idle, motor_cmd, &mixer_sat);
}

// This cycle's rates, setpoints, PID terms and motors into the high-rate capture (app/capture.h).
static void capture_sample(const float rate[3], const arm_status_t *st, float dt)
{
    cap_in_t in = {
        .rate = {rate[0], rate[1], rate[2]},
        .sp = {sp_p, sp_q, sp_yawr},
        .term = {rate_pid[0].out_p, rate_pid[0].integ, rate_pid[0].out_d,
                 rate_pid[1].out_p, rate_pid[1].integ, rate_pid[1].out_d,
                 rate_pid[2].out_p, rate_pid[2].integ},
        .thr = st->thr_cmd,
    };
    for (int i = 0; i < 4; i++) {
        in.m_us[i] = (float)ESC_STOP_US + st->motor[i] * (float)(ESC_MAX_US - ESC_STOP_US);
    }
    capture_add(&in, (uint32_t)(dt * 1.0e6f));
}

// Arming + motor output. Runs every cycle, including IMU timeouts, so the ESC output watchdog is fed
// and an IMU loss disarms at once. M4: all motors get the same throttle (mixer + PID come in M5).
// rate == NULL on cycles without a fresh IMU sample: the last motor command is held.
static arming_out_t motors_step(uint8_t imu_ok, uint32_t now_ms, const float *rate, float dt)
{
    rc_t rc;
    attitude_t att;
    shared_read(&rc, &g_rc, sizeof rc);
    shared_read(&att, &g_attitude, sizeof att);
    uint32_t disarm_req = g_gcs_disarm_req;
    bool esc_wd = pwm_esc_watchdog_tripped();

    arming_in_t in = {
        .now_ms = now_ms,
        .rc_valid = rc.valid,
        .rc_roll = rc.roll,
        .rc_pitch = rc.pitch,
        .rc_yaw = rc.yaw,
        .rc_thr = rc.thr,
        .arm_sw = rc.arm_sw,
        .imu_ok = imu_ok != 0u,
        .calib_done = calib_info.state == CALIB_DONE,
        .roll = att.roll,
        .pitch = att.pitch,
        .gcs_disarm = disarm_req != gcs_disarm_seen,
        .esc_wd_tripped = esc_wd,
        // Setpoints of the previous control cycle (one cycle old: ~1 ms, irrelevant against lc_ms).
        .att_err = fmaxf(fabsf(sp_roll - att.roll), fabsf(sp_pitch - att.pitch)),
        .lc_angle = lc_angle,
        .lc_ms = lc_ms,
    };
    gcs_disarm_seen = disarm_req;
    alt_out_t alt;
    shared_read(&alt, &g_alt, sizeof alt);
    // Failsafe stage 2 lands rather than disarms while z is known and the landing has not had to quit.
    in.land_ok = alt.valid && !ahold.land_failed;
    in.land_done = land_touchdown;   // a forced landing reached the floor on the previous cycle
    land_touchdown = false;
    if (ahold.landed) {
        in.rc_thr = 0.0f;  // landed in Alt Hold: motors to idle (auto-disarm timer runs) until the stick is lowered
    } else if (ahold.active) {
        // Alt Hold: a low stick means "descend", not "stop the motors" (no IDLE, no idle auto-disarm, no stick
        // disarm gesture; the ARM switch still disarms).
        in.rc_thr = fmaxf(in.rc_thr, CFG_THR_IDLE_EXIT + 0.01f);
    }
    arming_out_t out;
    arming_update(&arming, &in, &out);

    motor_test_req_t treq;
    shared_read(&treq, &g_motor_test_req, sizeof treq);
    mtest_msg = MTEST_MSG_NONE;
    if (treq.seq != mtest_seq_seen) {
        mtest_seq_seen = treq.seq;
        if (out.armed || treq.esc_ch > 3u) {
            mtest_msg = MTEST_MSG_REFUSED;
        } else {
            mtest_active = true;
            mtest_ch = treq.esc_ch;
            mtest_end_ms = now_ms + CFG_MOTOR_TEST_MS;
            mtest_msg = MTEST_MSG_START;
        }
    }
    if (mtest_active && (out.armed || in.gcs_disarm)) {
        mtest_active = false;  // arming or any DISARM command ends the test at once
        mtest_msg = MTEST_MSG_CANCELLED;
    } else if (mtest_active && (int32_t)(now_ms - mtest_end_ms) >= 0) {
        mtest_active = false;
        mtest_msg = MTEST_MSG_DONE;
    }

    // Switch values are stale without a link: in failsafe an engaged Alt Hold keeps holding (PLAN §7.7 stage 1).
    bool rc_live = rc.valid && out.fs == FS_NONE;
    bool flying = out.armed && out.motor_state != MOTOR_IDLE;

    // LAND command (GCS): until touchdown, disarm, or the pilot flips CH6 (takes over).
    uint32_t land_req = g_gcs_land_req;
    if (land_req != gcs_land_seen) {
        gcs_land_seen = land_req;
        if (out.armed && out.motor_state == MOTOR_RUN && alt.valid) {
            gcs_land = true;
            gcs_land_mode_sw = rc.mode_sw;
            land_msg = LAND_MSG_START;
        } else {
            land_msg = LAND_MSG_REFUSED;
        }
    }
    if (gcs_land && (!out.armed || (rc_live && rc.mode_sw != gcs_land_mode_sw) || ahold.land_failed)) {
        gcs_land = false;
        land_msg = out.armed ? LAND_MSG_CANCELLED : LAND_MSG_NONE;
    }
    bool land = out.armed && (out.fs == FS_STAGE2 || gcs_land);

    // Throttle stick curve (thr_mid / thr_expo) for Angle; Alt Hold gets the curved stick too, so its engagement
    // and the blend back to the stick after an exit match what Angle would fly.
    float thr = thr_curve(out.thr_cmd, prm.thr_mid, prm.thr_expo);
    if (rate != NULL) {
        alt_hold_in_t ah_in = {
            .dt = dt,
            .request = land || (rc_live ? rc.mode_sw == CFG_RC_MODE_ALT : ahold.active),
            .land = land,
            .flying = out.armed && out.motor_state == MOTOR_RUN,
            .alt_valid = alt.valid,
            .z = alt.z,
            .vz = alt.vz,
            // The real stick (out.thr_cmd is altered above while engaged or landed); failsafe: hold the height.
            .stick_thr = rc_live ? thr_curve(rc.thr, prm.thr_mid, prm.thr_expo) : 0.5f,
            .tilt_cos = cosf(att.roll) * cosf(att.pitch),
            .z_floor = alt.z_floor,
        };
        uint8_t ev;
        float ev_val;
        bool was_landing = ahold.landing;
        thr = alt_hold_step(&ahold, &ah_cfg, &ah_in, &ev, &ev_val);
        if (ev != AH_EV_NONE) {
            ah_event = ev;  // printed by the task loop, outside the timed section
            ah_event_val = ev_val;
        }
        if (ev == AH_EV_LANDED && was_landing) {
            land_touchdown = true;  // arming disarms on the next cycle (DISARM_LANDED)
            gcs_land = false;
        }

        // Position hold on top of an engaged Alt Hold, only with a valid flow estimate and above POS_HOLD_MIN_Z
        // (flow unreliable near the floor; also lets a stick landing end on plain levelling).
        flow_out_t fl;
        shared_read(&fl, &g_flow, sizeof fl);
        pos_hold_in_t ph_in = {
            .dt = dt,
            // Touching down (maybe landed): no horizontal control at all, legs on the floor could tip it over.
            .request = ahold.active && !ahold.maybe_landed && ph_cfg.pos_p > 0.0f && fl.valid &&
                       alt.z >= POS_HOLD_MIN_Z,
            .freeze = ahold.active && !ahold.maybe_landed && ph_cfg.pos_p > 0.0f,
            .stick = {out.roll_cmd, out.pitch_cmd},
            .v = {fl.v[0], fl.v[1]},
            .pos = {fl.pos[0], fl.pos[1]},
            .yaw = att.yaw,
        };
        pos_hold_event_t pev = pos_hold_step(&phold, &ph_cfg, &ph_in, &ph_out);
        if (pev != PH_EV_NONE) {
            ph_event = (uint8_t)pev;
            ph_event_pos[0] = fl.pos[0];
            ph_event_pos[1] = fl.pos[1];
            ph_event_why = !ahold.active ? 0u : ahold.maybe_landed ? 3u : !fl.valid ? 1u : 2u;
        }
    }
    if (!flying) {
        // Disarmed or idle at minimum throttle: no attitude control, I terms cleared.
        for (int i = 0; i < 3; i++) {
            pid_reset(&rate_pid[i]);
            motor_cmd[i] = out.armed ? motor_idle : 0.0f;
        }
        motor_cmd[3] = out.armed ? motor_idle : 0.0f;
        mixer_sat = (mixer_sat_t){0};
        sp_roll = sp_pitch = sp_yawr = sp_p = sp_q = 0.0f;
        yaw_hold_reset(&yhold);  // heading taken again once flying (turning on the ground is not held against)
    } else if (rate != NULL) {
        attitude_rate_control(&out, &att, rate, dt, thr, &ph_out);
    }

    if (out.armed) {
        uint16_t us[4];
        for (int i = 0; i < 4; i++) {
            us[motor_to_tim_ch[i]] =
                (uint16_t)((float)ESC_STOP_US + motor_cmd[i] * (float)(ESC_MAX_US - ESC_STOP_US) + 0.5f);
        }
        pwm_esc_write_us(us);
    } else if (mtest_active) {
        uint16_t us[4] = {ESC_STOP_US, ESC_STOP_US, ESC_STOP_US, ESC_STOP_US};
        us[mtest_ch] = CFG_MOTOR_TEST_US;
        pwm_esc_write_us(us);  // fed like normal output, so the ESC watchdog still covers a stall
    } else {
        pwm_esc_stop();
        pwm_esc_watchdog_clear();  // the disarm already latched "cycle the switch" (arming.c)
    }

    arm_status_t st = {
        .armed = out.armed,
        .motor_state = (uint8_t)out.motor_state,
        .fs = (uint8_t)out.fs,
        .mode = ahold.landing ? ARM_MODE_LAND : phold.active ? ARM_MODE_POS_HOLD
                : ahold.active ? ARM_MODE_ALT_HOLD : ARM_MODE_ANGLE,
        .block_mask = out.block_mask,
        .thr_cmd = flying ? thr : out.thr_cmd,
        .z_sp = ahold.active ? ahold.z_sp : 0.0f,
        .esc_wd_tripped = esc_wd,
        .roll_sp = sp_roll,
        .pitch_sp = sp_pitch,
        .yawr_sp = sp_yawr,
        .p_sp = sp_p,
        .q_sp = sp_q,
    };
    for (int i = 0; i < 4; i++) {
        st.motor[i] = out.armed ? motor_cmd[i] : 0.0f;
    }
    if (!out.armed && mtest_active) {
        // During a motor test the T line shows the ESC channel (CH1..CH4) order, not M1..M4.
        st.motor[mtest_ch] = (float)(CFG_MOTOR_TEST_US - ESC_STOP_US) / (float)(ESC_MAX_US - ESC_STOP_US);
    }
    shared_write(&g_arm, &st, sizeof st);
    if (rate != NULL && capture_state() == CAP_RECORDING) {
        capture_sample(rate, &st, dt);
    }
    return out;
}

static const char *const disarm_text[] = {
    [DISARM_SWITCH] = "switch", [DISARM_STICK] = "stick (yaw left)", [DISARM_GCS] = "GCS command",
    [DISARM_FAILSAFE] = "RC failsafe stage 2", [DISARM_FLIP] = "flip > 70 deg", [DISARM_IMU] = "IMU lost",
    [DISARM_AUTO] = "idle throttle timeout", [DISARM_ESC_WD] = "motor output watchdog",
    [DISARM_LOST_CONTROL] = "lost control (attitude far from setpoint)",
    [DISARM_LANDED] = "landed (failsafe / LAND command)",
};

static void format_block_reasons(uint32_t m, char *buf, size_t size)
{
    static const struct { uint32_t bit; const char *text; } names[] = {
        {ARM_BLK_SWITCH, " switch-not-ARM"}, {ARM_BLK_SWITCH_CYCLE, " cycle-switch-first"},
        {ARM_BLK_RC, " no-RC"}, {ARM_BLK_THROTTLE, " throttle-not-low"}, {ARM_BLK_STICKS, " sticks-not-centred"},
        {ARM_BLK_IMU, " IMU"}, {ARM_BLK_CALIB, " not-calibrated"}, {ARM_BLK_ANGLE, " tilted>15deg"},
        {ARM_BLK_FAILSAFE, " RC-recovering"}, {ARM_BLK_ESC_WD, " ESC-watchdog"},
    };
    size_t n = 0;
    buf[0] = '\0';
    for (size_t i = 0; i < sizeof names / sizeof names[0]; i++) {
        size_t len = strlen(names[i].text);
        if ((m & names[i].bit) && n + len + 1u < size) {
            memcpy(buf + n, names[i].text, len + 1u);
            n += len;
        }
    }
}

static void print_arm_event(const arming_out_t *o)
{
    char reasons[100];
    switch (o->event) {
    case ARM_EV_ARMED:
        mprintf("MSG,ARMED by switch\r\n");
        break;
    case ARM_EV_DISARMED:
        mprintf("MSG,DISARMED: %s\r\n",
                (o->event_arg < sizeof disarm_text / sizeof disarm_text[0] && disarm_text[o->event_arg])
                    ? disarm_text[o->event_arg] : "?");
        break;
    case ARM_EV_BLOCKED:
        format_block_reasons(o->block_mask, reasons, sizeof reasons);
        mprintf("MSG,ARM refused:%s\r\n", reasons);
        break;
    case ARM_EV_FS_STAGE1:
        mprintf("MSG,FAILSAFE stage 1: RC lost, levelling and holding throttle\r\n");
        break;
    case ARM_EV_FS_RECOVERED:
        mprintf("MSG,FAILSAFE cleared: RC back\r\n");
        break;
    default:
        break;
    }
}

static void print_motor_test_event(void)
{
    switch (mtest_msg) {
    case MTEST_MSG_START:
        mprintf("MSG,MOTOR_TEST ESC%u (TIM3 CH%u) at %u us for %u ms\r\n", (unsigned)mtest_ch + 1u,
                (unsigned)mtest_ch + 1u, (unsigned)CFG_MOTOR_TEST_US, (unsigned)CFG_MOTOR_TEST_MS);
        break;
    case MTEST_MSG_DONE:
        mprintf("MSG,MOTOR_TEST done\r\n");
        break;
    case MTEST_MSG_CANCELLED:
        mprintf("MSG,MOTOR_TEST cancelled\r\n");
        break;
    case MTEST_MSG_REFUSED:
        mprintf("ERR,MOTOR_TEST,armed\r\n");
        break;
    default:
        break;
    }
}

static void print_alt_hold_event(void)
{
    uint8_t ev = ah_event;
    ah_event = AH_EV_NONE;
    long mm = (long)(ah_event_val * 1000.0f);
    bool left = ev == AH_EV_EXIT_SWITCH || ev == AH_EV_EXIT_INVALID || ev == AH_EV_EXIT_ERROR || ev == AH_EV_LANDED;
    if (left && fabsf(ahold.hover - prm.hover_thr) >= 0.01f) {
        long h = (long)(ahold.hover * 1000.0f + 0.5f);
        mprintf("MSG,ALTHOLD hover throttle learnt %ld/1000 (hover_thr %ld): SET hover_thr 0.%03ld then SAVE to keep\r\n",
                h, (long)(prm.hover_thr * 1000.0f + 0.5f), h);
    }
    switch (ev) {
    case AH_EV_ENGAGED:
        mprintf("MSG,ALTHOLD on: holding z %ld mm, base throttle %ld/1000\r\n", mm, (long)(ahold.base_thr * 1000.0f));
        break;
    case AH_EV_EXIT_SWITCH:
        mprintf("MSG,ALTHOLD off (switch): throttle back to the stick over 0.5 s\r\n");
        break;
    case AH_EV_EXIT_INVALID:
        mprintf("MSG,ALTHOLD off: altitude estimate lost, throttle back to the stick (CH6 out and in to retry)\r\n");
        break;
    case AH_EV_EXIT_ERROR:
        mprintf("MSG,ALTHOLD off: z %ld mm from the setpoint, throttle back to the stick\r\n", mm);
        break;
    case AH_EV_REFUSED_INVALID:
        mprintf("MSG,ALTHOLD refused: altitude estimate not valid (CH6 out and in to retry)\r\n");
        break;
    case AH_EV_LAND:
        mprintf("MSG,LAND: descending from %ld mm at 0.3 m/s, disarm at touchdown\r\n", mm);
        break;
    case AH_EV_TAKEOFF:
        mprintf("MSG,ALTHOLD take-off: climbing to %ld mm at vz_max, base throttle %ld/1000 (stick below the hold "
                "band cancels)\r\n", mm, (long)(ahold.base_thr * 1000.0f));
        break;
    case AH_EV_LANDED:
        mprintf("MSG,ALTHOLD landed (z %ld mm): motors to idle, lower the throttle stick, then disarm\r\n", mm);
        break;
    case AH_EV_REFUSED_LOW:
        mprintf("MSG,ALTHOLD refused: z %ld mm below 300 mm (CH6 out and in to retry)\r\n", mm);
        break;
    default:
        break;
    }
}

static void print_land_msg(void)
{
    uint8_t m = land_msg;
    land_msg = LAND_MSG_NONE;
    if (m == LAND_MSG_START) {
        mprintf("OK,LAND,landing (flip CH6 to take over, DISARM to cut)\r\n");
    } else if (m == LAND_MSG_REFUSED) {
        mprintf("ERR,LAND,not flying or no altitude estimate\r\n");
    } else if (m == LAND_MSG_CANCELLED) {
        mprintf("MSG,LAND cancelled (CH6 moved, or the altitude estimate was lost): the pilot has it\r\n");
    }
}

static void print_pos_hold_event(void)
{
    uint8_t ev = ph_event;
    ph_event = PH_EV_NONE;
    long x = (long)(ph_event_pos[0] * 100.0f), y = (long)(ph_event_pos[1] * 100.0f);
    static const char *const why[] = {"Alt Hold off", "flow lost (quality / ToF / tilt)", "below 0.2 m or pos_p 0",
                                      "touching down"};
    switch (ev) {
    case PH_EV_ON:
        mprintf("MSG,POSHOLD on: braking, then anchoring where it stops\r\n");
        break;
    case PH_EV_ANCHOR:
        mprintf("MSG,POSHOLD anchored at x %ld y %ld cm\r\n", x, y);
        break;
    case PH_EV_OFF:
        mprintf("MSG,POSHOLD off (%s): roll/pitch back to the sticks over 0.5 s\r\n",
                why[ph_event_why < 4u ? ph_event_why : 0u]);
        if (phold.steady_t >= POS_HOLD_TRIM_MIN_S) {
            // In-flight trim: the integral averaged while hovering in place, added to the current trim.
            long r = lroundf((prm.trim_roll + phold.i_avg[1] * MATHX_RAD2DEG) * 100.0f);
            long p = lroundf((prm.trim_pitch - phold.i_avg[0] * MATHX_RAD2DEG) * 100.0f);
            mprintf("MSG,POSHOLD trim learnt over %lu s of hover: SET trim_roll %s%ld.%02ld / SET trim_pitch "
                    "%s%ld.%02ld (then SAVE)\r\n", (unsigned long)phold.steady_t, r < 0 ? "-" : "", labs(r) / 100,
                    labs(r) % 100, p < 0 ? "-" : "", labs(p) / 100, labs(p) % 100);
        }
        {
            // Accel bias learnt by the flow filter, as degrees of attitude error (bias / g): a few tenths is
            // normal; several degrees means the IMU reads a tilt the drone does not have (mounting, vibration).
            flow_out_t fl;
            shared_read(&fl, &g_flow, sizeof fl);
            long bf = lroundf(fl.acc_bias[0] / MATHX_G * MATHX_RAD2DEG * 10.0f);
            long br = lroundf(fl.acc_bias[1] / MATHX_G * MATHX_RAD2DEG * 10.0f);
            mprintf("MSG,FLOW accel bias learnt: fwd %s%ld.%ld deg, right %s%ld.%ld deg (as attitude error)\r\n",
                    bf < 0 ? "-" : "", labs(bf) / 10, labs(bf) % 10, br < 0 ? "-" : "", labs(br) / 10, labs(br) % 10);
        }
        break;
    default:
        break;
    }
}

static void delay_ms_rtos(uint32_t ms)
{
    osDelay(ms + 1u);  // osDelay(n) may return after n-1 full ticks
}

static bmi323_status_t init_imu(void)
{
    bmi323_info_t chip;
    trace_boot("BMI323 init (SPI polling) is running");
    bmi323_status_t rc = bmi323_init(delay_ms_rtos, &chip);
    imu_info_t info = {
        .init_status = rc,
        .chip_id = chip.chip_id,
        .err_reg = chip.err_reg,
        .acc_conf = chip.acc_conf,
        .gyr_conf = chip.gyr_conf,
    };
    shared_write(&g_imu_info, &info, sizeof info);
    trace_boot(rc == BMI323_OK ? "BMI323 init OK" : "BMI323 init FAILED (imu_ok stays 0, motors stay off)");
    return rc;
}

// Printed outside the timed section; costs ~tens of µs once per second (shows up in jitter_max).
static void print_ctrl_status(const ctrl_stats_t *st, uint32_t elapsed_ms)
{
    static uint32_t prev_cycles;
    uint32_t hz = (st->cycles - prev_cycles) * 1000u / elapsed_ms;
    prev_cycles = st->cycles;
    mprintf("MSG,task_ctrl is running: %lu Hz exec_max %lu us jitter_max %lu us imu_timeouts %lu imu_ok %u\r\n",
            (unsigned long)hz, (unsigned long)st->exec_us_max, (unsigned long)st->jitter_us_max,
            (unsigned long)st->imu_timeouts, (unsigned)st->imu_ok);
}

void task_ctrl(void const *argument)
{
    (void)argument;
    trace_boot("task_ctrl is running (osPriorityRealtime)");

    if (init_imu() != BMI323_OK) {
        for (;;) {
            mprintf("MSG,task_ctrl is running: IMU init failed, motors disabled\r\n");
            osDelay(CTRL_PRINT_PERIOD_MS);
        }
    }
    ctrl_apply_params(true);
#if APP_IMU_STREAM
    imu_pipeline_start(prm.gyro_lpf_hz, prm.acc_lpf_hz);
    trace_boot("IMU stream is running (INT1 EXTI -> SPI DMA -> ISR, 3200 Hz)");
#else
    trace_boot("IMU stream DISABLED (APP_IMU_STREAM 0): task_ctrl only times out");
#endif
    trace_boot("task_ctrl loop is running (woken by SIG_IMU every 3 samples)");

    ctrl_stats_t st = {0};
    uint32_t prev_start = time_cycles();
    uint32_t consecutive_timeouts = 0;
    uint32_t last_print_ms = osKernelSysTick();
    arming_init(&arming);
    float body_rate[3] = {0.0f, 0.0f, 0.0f};
    float rate_dt = 1.0f / CFG_CTRL_LOOP_HZ;
    uint32_t rate_last_t_cyc = time_cycles();

    for (;;) {
        osEvent ev = osSignalWait(SIG_IMU, CTRL_IMU_TIMEOUT_MS);
        uint32_t now_ms = osKernelSysTick();
        ctrl_apply_params(false);
        if (LOG_PERIODIC_STATS && LOG_RAW_MODE == 0 && now_ms - last_print_ms >= CTRL_PRINT_PERIOD_MS) {
            print_ctrl_status(&st, now_ms - last_print_ms);
            last_print_ms = now_ms;
        }

        bool fresh = ev.status == osEventSignal;
        if (fresh) {
            consecutive_timeouts = 0;
            st.imu_ok = 1;
        } else {
            st.imu_timeouts++;
            if (++consecutive_timeouts >= CTRL_IMU_LOST_TIMEOUTS) {
                st.imu_ok = 0;
            }
            trace_mark(TR_CTRL_TIMEOUT, 0);
        }

        uint32_t start = time_cycles();
        DBG1_HIGH();
        uint32_t period_us = time_cycles_to_us(start - prev_start);
        est_msg_t msg = EST_MSG_NONE;
        if (fresh) {
            prev_start = start;
            imu_sample_t sample;
            shared_read(&sample, &g_imu_sample, sizeof sample);
            msg = estimator_step(&sample);
            if (calib_info.state == CALIB_DONE) {
                acc_up_step(&sample);
            }
            for (int i = 0; i < 3; i++) {
                body_rate[i] = sample.gyro[i] - calib_info.gyro_bias[i];
            }
            rate_dt = (float)(sample.t_cyc - rate_last_t_cyc) / (float)(TIME_CYCLES_PER_US * 1000000u);
            rate_last_t_cyc = sample.t_cyc;
        }
        arming_out_t arm_out = motors_step(st.imu_ok, now_ms, fresh ? body_rate : NULL, rate_dt);
        if (fresh) {
            notch_track();
        }
        DBG1_LOW();
        trace_mark(TR_CTRL_DONE, 0);
        uint32_t exec_us = time_cycles_to_us(time_cycles() - start);

        // Text output only after the timed section.
        if (msg == EST_MSG_CALIB_MOTION) {
            // calib_acc still holds the aborted window until the next cycle restarts it.
            float gyro_pp = 0.0f;
            for (int i = 0; i < 3; i++) {
                gyro_pp = fmaxf(gyro_pp, calib_acc.gyro_max[i] - calib_acc.gyro_min[i]);
            }
            mprintf("MSG,CALIB motion detected (gyro p-p %ld mrad/s, limit %ld | |a| p-p %ld mm/s2, limit %ld),"
                    " restarting (restarts %u)\r\n",
                    (long)(gyro_pp * 1000.0f), (long)(CFG_CALIB_GYRO_RANGE * 1000.0f),
                    (long)((calib_acc.acc_norm_max - calib_acc.acc_norm_min) * 1000.0f),
                    (long)(CFG_CALIB_ACC_RANGE * 1000.0f), (unsigned)calib_info.restarts);
        }
        print_arm_event(&arm_out);
        print_motor_test_event();
        print_alt_hold_event();
        print_pos_hold_event();
        print_land_msg();

        if (fresh) {
            st.cycles++;
            if (st.cycles > CTRL_WARMUP_CYCLES) {
                uint32_t jitter_us = (period_us > CTRL_NOMINAL_PERIOD_US) ? period_us - CTRL_NOMINAL_PERIOD_US
                                                                          : CTRL_NOMINAL_PERIOD_US - period_us;
                if (jitter_us > st.jitter_us_max) {
                    st.jitter_us_max = jitter_us;
                }
                if (exec_us > st.exec_us_max) {
                    st.exec_us_max = exec_us;
                }
            }
        }
        shared_write(&g_ctrl_stats, &st, sizeof st);
    }
}

/* ---- task_slow: ~100 Hz: RC input (M4); ToF/baro/mag/altitude in later milestones ---- */

// No ToF packets yet, so the timeout paces the loop at ~100 Hz. Becomes 15 ms once ToF wakes it (M8).
#define SLOW_TOF_TIMEOUT_MS  10u
#define SLOW_PRINT_PERIOD_MS 1000u

// M8 step 1 (PLAN §6.2, §6.3): confirm the chips and the MTF-01P protocol before writing drivers.
// Requests come from task_telem commands; only task_slow touches I2C1 and USART2.
volatile uint32_t g_i2c_scan_req;  // writer: task_telem (I2CSCAN)
volatile uint32_t g_flow_raw_req;  // writer: task_telem (FLOWRAW)
// MAGCAL: struct written before seq is incremented (single writer task_telem), like g_params.
static volatile struct { uint32_t seq, duration_ms; } g_magcal_req;
volatile uint32_t g_levelcal_req;  // writer: task_telem (LEVELCAL)

// Signed centi-units as "-1.23" (mprintf has no %f).
static void fmt_2dp(char *out, size_t size, float v)
{
    long t = (long)(v * 100.0f + (v >= 0.0f ? 0.5f : -0.5f));
    unsigned long a = (unsigned long)(t < 0 ? -t : t);
    snprintf(out, size, "%s%lu.%02lu", t < 0 ? "-" : "", a / 100u, a % 100u);
}

// LEVELCAL (app/levelcal.h): ~2 s of filtered accel, then the trim for a level prop plane as SET lines,
// like MAGCAL (parameters have a single writer, task_telem).
static void levelcal_step(levelcal_t *c, bool *active)
{
    imu_sample_t s;
    shared_read(&s, &g_imu_sample, sizeof s);
    if (!levelcal_add(c, s.acc)) {
        return;
    }
    *active = false;
    float roll, pitch;
    bool inverted;
    levelcal_result_t r = levelcal_finish(c, &roll, &pitch, &inverted);
    char rs[16], ps[16];
    fmt_2dp(rs, sizeof rs, roll * MATHX_RAD2DEG);
    fmt_2dp(ps, sizeof ps, pitch * MATHX_RAD2DEG);
    if (r == LEVELCAL_MOVED) {
        mprintf("ERR,LEVELCAL,moved during the 2 s: keep the drone still and retry\r\n");
    } else if (r == LEVELCAL_TILTED) {
        mprintf("ERR,LEVELCAL,prop plane %s %s deg from level (> 10): not on a level surface?\r\n", rs, ps);
    } else {
        mprintf("MSG,LEVELCAL %s: IMU reads roll %s pitch %s deg with the prop plane level\r\n",
                inverted ? "upside down on the motor caps" : "upright", rs, ps);
        mprintf("MSG,LEVELCAL SET trim_roll %s\r\n", rs);
        mprintf("MSG,LEVELCAL SET trim_pitch %s\r\n", ps);
        mprintf("MSG,LEVELCAL send the 2 SET lines above, then SAVE\r\n");
    }
}

// Every 7-bit address answering an address-only probe. Takes ~0.1 s; the RC is read again right after.
static void i2c_scan(void)
{
    char line[160];
    int n = snprintf(line, sizeof line, "MSG,I2CSCAN I2C1 (PB6 SCL, PB7 SDA) found:");
    unsigned found = 0;
    for (uint16_t a = 0x08u; a <= 0x77u; a++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 1, 2) == HAL_OK) {
            found++;
            if (n > 0 && n < (int)sizeof line - 8) {
                n += snprintf(line + n, sizeof line - (size_t)n, " 0x%02X", (unsigned)a);
            }
        }
    }
    mprintf("%s%s (bus error code 0x%lX)\r\n", line, found ? "" : " none",
            (unsigned long)HAL_I2C_GetError(&hi2c1));

    // Identity registers of the usual candidates at these addresses. Raw values only: which value means
    // which chip is checked against the datasheets before a driver is written (PLAN §6.2).
    static const struct { uint8_t addr, reg; const char *what; } ids[] = {
        {0x0Eu, 0x00u, "0x0E reg 0x00 (IST8310 WAI)"}, {0x0Du, 0x0Du, "0x0D reg 0x0D (QMC5883L ID)"},
        {0x76u, 0x0Du, "0x76 reg 0x0D (DPS310/368 ID)"}, {0x76u, 0xD0u, "0x76 reg 0xD0 (BMP280/BME280 ID)"},
        {0x76u, 0x00u, "0x76 reg 0x00 (BMP388/390 ID)"}, {0x77u, 0x0Du, "0x77 reg 0x0D (DPS310/368 ID)"},
    };
    for (size_t i = 0; i < sizeof ids / sizeof ids[0]; i++) {
        uint8_t v = 0;
        if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(ids[i].addr << 1), ids[i].reg, I2C_MEMADD_SIZE_8BIT, &v, 1, 5) ==
            HAL_OK) {
            mprintf("MSG,I2CID %s = 0x%02X\r\n", ids[i].what, v);
        }
    }
}

// MTF-01P decoding (task_slow). The rangefinder is valid while in range and fresher than TOF_TIMEOUT_MS.
#define TOF_TIMEOUT_MS 50u
static mspv2_parser_t tof_parser;
static tof_t tof;

// FLOWLOG 1: one "F" line per optical flow frame (~100 Hz) with the gyro and ToF of that moment, for
// tools/flow_cal.py (axis map, sign and scale of motion_x/y: units are not documented, PLAN §6.3).
volatile uint32_t g_flowlog_on;  // writer: task_telem (FLOWLOG)

static void flowlog_print(const mtf01p_flow_t *f, uint32_t now_ms)
{
    attitude_t att;
    shared_read(&att, &g_attitude, sizeof att);
    mprintf_to(g_flowlog_on, "F,%lu,%u,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n", (unsigned long)now_ms, (unsigned)f->quality,
            (long)f->motion_x, (long)f->motion_y, (long)(att.rate[0] * 1000.0f), (long)(att.rate[1] * 1000.0f),
            (long)(att.rate[2] * 1000.0f), (long)(att.roll * MATHX_RAD2DEG * 100.0f),
            (long)(att.pitch * MATHX_RAD2DEG * 100.0f), (long)(tof.distance_m * 1000.0f));
}

// Horizontal velocity from the flow (app/flow_est.h, position hold step 2): updated per flow frame here, predicted
// and published every task_slow cycle by flow_step().
static flow_est_t flow_est;
static flow_cfg_t flow_cfg;
static uint32_t flow_prev_cyc;
volatile uint32_t g_poszero_req;  // writer: task_telem (POSZERO)

static void flow_frame(const mtf01p_flow_t *m, uint32_t now_ms)
{
    attitude_t att;
    shared_read(&att, &g_attitude, sizeof att);
    bool range_ok = tof.distance_m >= 0.0f && now_ms - tof.t_ms < TOF_TIMEOUT_MS;
    flow_frame_t f = {
        .mx = m->motion_x,
        .my = m->motion_y,
        .quality = m->quality,
        .p = att.rate[0],
        .q = att.rate[1],
        .h = range_ok ? tof.distance_m : -1.0f,
        .tilt_cos = cosf(att.roll) * cosf(att.pitch),
    };
    (void)flow_est_frame(&flow_est, &flow_cfg, &f, now_ms);
}

static void flow_step(uint32_t now_ms, const params_t *p, uint32_t *poszero_seen)
{
    uint32_t cyc = time_cycles();
    float dt = (float)(cyc - flow_prev_cyc) / (float)(TIME_CYCLES_PER_US * 1000000u);
    flow_prev_cyc = cyc;
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.01f;
    }
    flow_cfg = (flow_cfg_t){p->flow_k, p->flow_scale, p->flow_qmin, p->flow_sigma,
                            (uint8_t)((unsigned long)lroundf(p->flow_rot / 90.0f) & 3u)};
    if (g_poszero_req != *poszero_seen) {
        *poszero_seen = g_poszero_req;
        flow_est_zero_position(&flow_est);
    }
    acc_up_t acc;
    attitude_t att;
    shared_read(&acc, &g_acc_up, sizeof acc);
    shared_read(&att, &g_attitude, sizeof att);
    flow_est_predict(&flow_est, acc.a_h, att.rate[2], att.yaw, dt);
    flow_out_t out = {
        .v = {flow_est.v[0], flow_est.v[1]},
        .meas = {flow_est.meas[0], flow_est.meas[1]},
        .pos = {flow_est.pos[0], flow_est.pos[1]},
        .acc_bias = {flow_est.b[0], flow_est.b[1]},
        .quality = tof.flow_quality,
        .valid = flow_est_valid(&flow_est, now_ms),
    };
    shared_write(&g_flow, &out, sizeof out);
}

static void tof_feed(const uint8_t *b, size_t n, uint32_t now_ms)
{
    for (size_t i = 0; i < n; i++) {
        if (!mspv2_feed(&tof_parser, b[i])) {
            continue;
        }
        mtf01p_range_t r;
        mtf01p_flow_t f;
        if (mtf01p_parse_range(&tof_parser.frame, &r)) {
            tof.t_ms = now_ms;
            tof.quality = r.quality;
            tof.distance_m = (r.distance_mm < 0) ? -1.0f : (float)r.distance_mm * 0.001f;
        } else if (mtf01p_parse_flow(&tof_parser.frame, &f)) {
            tof.flow_t_ms = now_ms;
            tof.flow_quality = f.quality;
            tof.flow_x = f.motion_x;
            tof.flow_y = f.motion_y;
            if (g_flowlog_on) {
                flowlog_print(&f, now_ms);
            }
            flow_frame(&f, now_ms);
        }
    }
}

static void tof_publish(uint32_t now_ms)
{
    tof.frames = tof_parser.frames;
    tof.crc_errors = tof_parser.crc_errors;
    tof.valid = tof.frames > 0u && tof.distance_m >= 0.0f && now_ms - tof.t_ms < TOF_TIMEOUT_MS;
    shared_write(&g_tof, &tof, sizeof tof);
}

// Altitude estimator (M9, observation only). Runs once the IMU calibration has fixed the accel bias.
// ALTRAW 1 adds an "A" line per step (inputs and outputs) for tools/replay_kf.py.
volatile uint32_t g_altraw_on;  // writer: task_telem (ALTRAW)
static alt_est_t alt_est;
static uint32_t alt_tof_seen_ms;
static uint32_t alt_prev_cyc;

static long to_milli(float v)
{
    return (long)(v * 1000.0f + (v >= 0.0f ? 0.5f : -0.5f));
}

static void alt_step(uint32_t now_ms, const params_t *p)
{
    calib_info_t cal;
    shared_read(&cal, &g_calib, sizeof cal);
    uint32_t cyc = time_cycles();
    if (cal.state != CALIB_DONE) {
        alt_prev_cyc = cyc;
        return;
    }
    float dt = (float)(cyc - alt_prev_cyc) / (float)(TIME_CYCLES_PER_US * 1000000u);
    alt_prev_cyc = cyc;
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.01f;  // first call or a stall: one nominal step
    }
    acc_up_t acc;
    attitude_t att;
    arm_status_t arm;
    shared_read(&acc, &g_acc_up, sizeof acc);
    shared_read(&att, &g_attitude, sizeof att);
    shared_read(&arm, &g_arm, sizeof arm);
    alt_in_t in = {
        .now_ms = now_ms,
        .dt = dt,
        .a_up = acc.a_up,
        .tof_new = tof.frames > 0u && tof.t_ms != alt_tof_seen_ms,
        .tof_in_range = tof.distance_m >= 0.0f,
        .tof_dist = tof.distance_m,
        .roll = att.roll,
        .pitch = att.pitch,
        .armed = arm.armed != 0u,
    };
    alt_tof_seen_ms = tof.t_ms;
    baro_t baro;
    shared_read(&baro, &g_baro, sizeof baro);
    static uint32_t baro_seen_ms;
    in.baro_new = baro.valid && baro.t_ms != baro_seen_ms;
    in.baro_alt = baro.alt_rel_m;
    baro_seen_ms = baro.t_ms;
    // A failsafe / commanded landing may take longer than the PLAN's 3 s baro bridge (2 m at 0.3 m/s ~ 7 s).
    uint32_t baro_max = (arm.mode == ARM_MODE_LAND) ? ALT_BARO_MAX_LAND_MS : ALT_BARO_MAX_MS;
    const alt_cfg_t cfg = {p->alt_sigma_a, p->alt_sigma_tof, p->alt_step_th, p->alt_sigma_baro, baro_max};
    alt_out_t out;
    alt_est_step(&alt_est, &cfg, &in, &out);
    shared_write(&g_alt, &out, sizeof out);

    switch ((alt_event_t)out.event) {
    case ALT_EV_INIT:
        mprintf("MSG,ALT init from ToF: z %ld mm\r\n", to_milli(out.z));
        break;
    case ALT_EV_STEP:
        mprintf("MSG,ALT step under the drone: h_terrain %ld mm, z %ld mm\r\n", to_milli(out.event_val),
                to_milli(out.z));
        break;
    case ALT_EV_RESYNC:
        mprintf("MSG,ALT ToF disagreed 0.1 s: z restarted from ToF (z error %ld mm; accel drift?)\r\n",
                to_milli(out.event_val));
        break;
    case ALT_EV_GROUND:
        mprintf("MSG,ALT on the floor again: h_terrain %ld -> 0 mm, z %ld mm\r\n", to_milli(out.event_val),
                to_milli(out.z));
        break;
    case ALT_EV_ARM:
        mprintf("MSG,ALT armed: z re-referenced to this floor, z %ld mm (h_terrain was %ld mm)\r\n",
                to_milli(out.z), to_milli(out.event_val));
        break;
    default:
        break;
    }
    if (g_altraw_on) {
        mprintf_to(g_altraw_on, "A,%lu,%lu,%ld,%u,%u,%ld,%ld,%ld,%ld,%ld,%ld,%u,%u,%u,%u\r\n", (unsigned long)now_ms,
                (unsigned long)(dt * 1.0e6f), to_milli(in.a_up), (unsigned)in.tof_new, (unsigned)in.tof_in_range,
                to_milli(in.tof_dist), (long)(in.roll * MATHX_RAD2DEG * 100.0f),
                (long)(in.pitch * MATHX_RAD2DEG * 100.0f), to_milli(out.z), to_milli(out.vz),
                to_milli(out.h_terrain), (unsigned)out.src, (unsigned)out.step_state, (unsigned)out.valid,
                (unsigned)in.armed);
    }
}

// Byte count per second, "$X" (MSPv2) and "$M" (MSPv1) header counts, and the first bytes seen, to tell
// whether the MTF-01P is set to MSP and at the right baud rate.
typedef struct {
    uint32_t start_ms, bytes, msp2, msp1;
    uint8_t first[24];
    uint8_t n_first;
    uint8_t prev;
    bool active;
} flow_raw_t;

static void flow_raw_feed(flow_raw_t *r, const uint8_t *b, size_t n, uint32_t now_ms)
{
    for (size_t i = 0; i < n; i++) {
        if (r->prev == '$' && b[i] == 'X') {
            r->msp2++;
        } else if (r->prev == '$' && b[i] == 'M') {
            r->msp1++;
        }
        r->prev = b[i];
        if (r->active && r->n_first < sizeof r->first) {
            r->first[r->n_first++] = b[i];
        }
    }
    if (!r->active) {
        return;
    }
    r->bytes += (uint32_t)n;
    if (now_ms - r->start_ms >= 1000u) {
        char hex[3 * sizeof r->first + 1];
        size_t h = 0;
        for (uint8_t i = 0; i < r->n_first; i++) {
            h += (size_t)snprintf(hex + h, sizeof hex - h, "%02X ", r->first[i]);
        }
        hex[h] = '\0';
        mprintf("MSG,FLOWRAW USART2 115200: %lu B/s, $X %lu, $M %lu, uart restarts %lu | %s\r\n",
                (unsigned long)r->bytes, (unsigned long)r->msp2, (unsigned long)r->msp1,
                (unsigned long)flow_uart_restarts(), r->n_first ? hex : "(no bytes)");
        mprintf("MSG,TOF %ld mm q %u valid %u | flow x %ld y %ld q %u | frames %lu crc_err %lu oversize %lu\r\n",
                (long)(tof.distance_m * 1000.0f), (unsigned)tof.quality, (unsigned)tof.valid, (long)tof.flow_x,
                (long)tof.flow_y, (unsigned)tof.flow_quality, (unsigned long)tof_parser.frames,
                (unsigned long)tof_parser.crc_errors, (unsigned long)tof_parser.oversize);
        r->active = false;
    }
}

// ---- GPS (G1, 2026-11-14): GEP-M10DI on USART3, UBX NAV-PVT at 10 Hz, observation only ----
#define GPS_CFG_FIRST_MS  1000u  // after task_slow starts (the module boots with the board)
#define GPS_CFG_RETRY_MS  2000u  // resend while no NAV-PVT arrives (module powered later, config lost)
#define GPS_FRESH_MS      500u
#define GPS_NODATA_MS     5000u
#define GPS_REPORT_MS     10000u // repeat the "no fix" line this often

static ubx_parser_t gps_parser;
static gps_t gps;

// RAM layer only: UBX out on UART1 with NAV-PVT every solution, 10 Hz, NMEA off (the bandwidth it frees and
// one protocol to parse). Each VALSET is applied whole or not at all (ACK-NAK).
static void gps_configure(void)
{
    static const ubx_kv_t kv[] = {
        {UBX_CFG_KEY_UART1OUTPROT_UBX, 1u},
        {UBX_CFG_KEY_MSGOUT_NAV_PVT_UART1, 1u},
        {UBX_CFG_KEY_RATE_MEAS, 100u},
        {UBX_CFG_KEY_UART1OUTPROT_NMEA, 0u},
    };
    uint8_t buf[64];
    size_t n = ubx_build_valset(buf, sizeof buf, kv, sizeof kv / sizeof kv[0]);
    if (n > 0u) {
        gps_uart_write(buf, n);  // ~3 ms blocking, only while disarmed
        gps.cfg_sent++;
    }
}

// 0 no bytes, 1 bytes but no NAV-PVT, 2 NAV-PVT without a usable fix, 3 3D fix (gnssFixOK)
static unsigned gps_state(void)
{
    if (gps.bytes == 0u) {
        return 0u;
    }
    if (!gps.fresh) {
        return 1u;
    }
    return (gps.pvt.fix_ok && gps.pvt.fix_type >= 3u) ? 3u : 2u;
}

static void gps_report(unsigned s)
{
    char acc[16];
    switch (s) {
    case 0u:
        mprintf("MSG,GPS no bytes on USART3 for 5 s: check power and GPS TX -> PC11, GPS RX -> PC10\r\n");
        break;
    case 1u:
        mprintf("MSG,GPS bytes %lu but no NAV-PVT (NMEA lines %lu, UBX frames %lu, checksum errors %lu, config sent "
                "%lu ack %lu nak %lu): baud not 115200 or config refused\r\n",
                (unsigned long)gps.bytes, (unsigned long)gps.nmea_lines, (unsigned long)gps_parser.frames,
                (unsigned long)gps.ck_errors, (unsigned long)gps.cfg_sent, (unsigned long)gps.acks,
                (unsigned long)gps.naks);
        break;
    case 2u:
        mprintf("MSG,GPS NAV-PVT ok, no 3D fix yet: fix type %u, %u satellites (outdoors, open sky; first fix can "
                "take a few minutes)\r\n", (unsigned)gps.pvt.fix_type, (unsigned)gps.pvt.num_sv);
        break;
    default:
        (void)numfmt_float(acc, sizeof acc, (float)gps.pvt.h_acc_mm * 0.001f, 2u);
        mprintf("MSG,GPS 3D fix: %u satellites, horizontal accuracy %s m\r\n", (unsigned)gps.pvt.num_sv, acc);
        break;
    }
}

static void gps_step(uint32_t now_ms, uint32_t start_ms)
{
    static uint32_t last_cfg_ms, last_report_ms;
    static unsigned reported = 99u;
    static uint8_t prev_byte;
    uint8_t b[64];
    size_t n;
    while ((n = gps_uart_read(b, sizeof b)) > 0u) {
        gps.bytes += (uint32_t)n;
        for (size_t i = 0; i < n; i++) {
            if (prev_byte == '$' && b[i] == 'G') {
                gps.nmea_lines++;  // "$GNGGA" etc.: the module talks NMEA (default) at this baud
            }
            prev_byte = b[i];
            if (!ubx_parse_byte(&gps_parser, b[i])) {
                continue;
            }
            if (gps_parser.cls == UBX_CLASS_NAV && gps_parser.id == UBX_ID_NAV_PVT) {
                if (ubx_decode_nav_pvt(gps_parser.payload, gps_parser.len, &gps.pvt)) {
                    gps.t_ms = now_ms;
                    gps.pvt_count++;
                }
            } else if (gps_parser.cls == UBX_CLASS_ACK && gps_parser.len >= 2u &&
                       gps_parser.payload[0] == UBX_CLASS_CFG && gps_parser.payload[1] == UBX_ID_CFG_VALSET) {
                if (gps_parser.id == UBX_ID_ACK_ACK) {
                    gps.acks++;
                } else {
                    gps.naks++;
                }
            }
        }
    }
    gps.ck_errors = gps_parser.ck_errors;
    gps.fresh = gps.pvt_count > 0u && now_ms - gps.t_ms < GPS_FRESH_MS;
    arm_status_t arm;
    shared_read(&arm, &g_arm, sizeof arm);
    if (!gps.fresh && !arm.armed && now_ms - start_ms >= GPS_CFG_FIRST_MS &&
        (gps.cfg_sent == 0u || now_ms - last_cfg_ms >= GPS_CFG_RETRY_MS)) {
        gps_configure();
        last_cfg_ms = now_ms;
    }
    shared_write(&g_gps, &gps, sizeof gps);

    unsigned s = gps_state();
    if (s == 0u && now_ms - start_ms < GPS_NODATA_MS) {
        return;
    }
    if (s != reported || (s < 3u && now_ms - last_report_ms >= GPS_REPORT_MS)) {
        if (s != reported || s != 0u) {  // "no bytes" once; the others repeat while not fixed
            gps_report(s);
        }
        reported = s;
        last_report_ms = now_ms;
    }
}

void task_slow(void const *argument)
{
    (void)argument;
    slow_stats_t st = {0};
    trace_boot("task_slow is running (osPriorityHigh, ~100 Hz)");
    uint32_t last_print_ms = osKernelSysTick();
    uint32_t prev_cycles = 0;
    rc_start();
    trace_boot("RC SBUS is running (USART1 DMA + IDLE)");
    flow_uart_start();
    gps_uart_start();
    ubx_parser_init(&gps_parser);
    const uint32_t slow_start_ms = osKernelSysTick();
    i2c_sensors_init();
    alt_est_init(&alt_est);
    alt_prev_cyc = time_cycles();
    uint32_t prm_seq_slow = g_params_seq - 1u;  // forces the first copy
    params_t slow_prm;
    shared_read(&slow_prm, &g_params, sizeof slow_prm);
    uint32_t magcal_seen = g_magcal_req.seq;
    uint32_t i2c_scan_seen = g_i2c_scan_req;
    uint32_t flow_raw_seen = g_flow_raw_req;
    flow_raw_t flow_raw = {0};
    uint32_t levelcal_seen = g_levelcal_req;
    uint32_t poszero_seen = g_poszero_req;
    flow_est_init(&flow_est);
    flow_prev_cyc = time_cycles();
    levelcal_t levelcal;
    bool levelcal_active = false;

    for (;;) {
        (void)osSignalWait(SIG_TOF, SLOW_TOF_TIMEOUT_MS);
        DBG2_TOGGLE();
        if (g_params_seq != prm_seq_slow) {
            prm_seq_slow = g_params_seq;
            shared_read(&slow_prm, &g_params, sizeof slow_prm);
            rc_set_deadband(slow_prm.rc_deadband);
        }
        rc_update(osKernelSysTick());
        st.cycles++;
        shared_write(&g_slow_stats, &st, sizeof st);

        uint32_t now_ms = osKernelSysTick();
        flow_step(now_ms, &slow_prm, &poszero_seen);  // predict before this cycle's flow frames update it
        uint8_t fb[64];
        size_t fn;
        if (g_flow_raw_req != flow_raw_seen) {
            flow_raw_seen = g_flow_raw_req;
            flow_raw = (flow_raw_t){.start_ms = now_ms, .active = true};
        }
        while ((fn = flow_uart_read(fb, sizeof fb)) > 0u) {
            flow_raw_feed(&flow_raw, fb, fn, now_ms);
            tof_feed(fb, fn, now_ms);
        }
        flow_raw_feed(&flow_raw, fb, 0u, now_ms);  // closes the 1 s window even with no bytes
        tof_publish(now_ms);
        if (g_magcal_req.seq != magcal_seen) {
            magcal_seen = g_magcal_req.seq;
            i2c_sensors_magcal_start(now_ms, g_magcal_req.duration_ms);
        }
        i2c_sensors_step(now_ms, &slow_prm);
        gps_step(now_ms, slow_start_ms);
        alt_step(now_ms, &slow_prm);
        if (g_levelcal_req != levelcal_seen) {
            levelcal_seen = g_levelcal_req;
            levelcal_start(&levelcal);
            levelcal_active = true;
        }
        if (levelcal_active) {
            levelcal_step(&levelcal, &levelcal_active);
        }
        if (g_i2c_scan_req != i2c_scan_seen) {
            i2c_scan_seen = g_i2c_scan_req;
            arm_status_t arm;
            shared_read(&arm, &g_arm, sizeof arm);
            if (arm.armed) {
                mprintf("ERR,I2CSCAN,disarm first\r\n");  // the scan stalls task_slow (RC) for ~0.1 s
            } else {
                i2c_scan();
            }
        }
        if (LOG_PERIODIC_STATS && LOG_RAW_MODE == 0 && now_ms - last_print_ms >= SLOW_PRINT_PERIOD_MS) {
            mprintf("MSG,task_slow is running: %lu Hz\r\n",
                    (unsigned long)((st.cycles - prev_cycles) * 1000u / (now_ms - last_print_ms)));
            prev_cycles = st.cycles;
            last_print_ms = now_ms;
        }
    }
}

/* ---- task_telem: telemetry, status/event messages, commands, trace dump ---- */

#define TELEM_PERIOD_MS      20u
#define STATS_EVERY_TICKS    50u    // 1 s
#define TRACE_LINES_PER_TICK 6u     // keeps output under one 512-byte USB chunk per 20 ms
#define CMD_LINE_MAX         48u
// DUMP / CAPTURE pace, ~75 B lines: USB 6 per tick (~22 KB/s, 1280 capture lines in ~4.3 s); the 115200-baud
// HC-05 2 per tick (~7.5 KB/s of its ~10 KB/s, ~13 s).
#define DUMP_LINES_PER_TICK       6u
#define DUMP_LINES_PER_TICK_RADIO 2u
// Radio share of the 50 Hz stream: T every 3rd tick (~17 Hz, ~3 KB/s), R every 5th RC line (2 Hz).
#define TELEM_RADIO_DIV      3u
#define RC_RADIO_DIV         5u
#define BB_TAIL_MS           2000u  // keep recording this long after disarm (landing / crash)
// Event lines must not be starved by telemetry when the host is slow or the ring is filling.
#define TELEM_MIN_TX_FREE    1024u

static uint32_t reset_flags;
static param_load_t params_load_result;
static unsigned params_loaded;

// Before the scheduler starts (single writer rule holds trivially): defaults, then whatever valid
// entries the flash image has.
void params_boot_load(void)
{
    params_t p;
    params_defaults(&p);
    size_t len;
    const uint8_t *img = flash_params_image(&len);
    params_load_result = params_image_load(&p, img, len, &params_loaded);
    if (params_load_result != PARAM_LOAD_OK) {
        params_defaults(&p);  // a corrupt image is not applied even partly
    }
    shared_write(&g_params, &p, sizeof p);
    g_params_seq = g_params_seq + 1u;
}

void task_telem_capture_reset_cause(void)
{
    reset_flags = RCC->CSR;
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

static void log_boot_banner(void)
{
    mprintf("MSG,== Drone FW (M6 params) == SYSCLK %lu Hz, reset:%s%s%s%s%s\r\n",
            (unsigned long)SystemCoreClock,
            (reset_flags & RCC_CSR_PORRSTF) ? " POR" : "",
            (reset_flags & RCC_CSR_PINRSTF) ? " PIN" : "",
            (reset_flags & RCC_CSR_SFTRSTF) ? " SW" : "",
            (reset_flags & RCC_CSR_IWDGRSTF) ? " IWDG" : "",
            (reset_flags & RCC_CSR_WWDGRSTF) ? " WWDG" : "");
    if (params_load_result == PARAM_LOAD_OK) {
        mprintf("MSG,PARAMS %u of %u loaded from flash\r\n", params_loaded, (unsigned)params_count());
    } else {
        mprintf("MSG,PARAMS defaults (flash %s)\r\n", (params_load_result == PARAM_LOAD_EMPTY) ? "empty" : "CORRUPT");
    }
}

static void log_imu(uint32_t elapsed_ms)
{
    static imu_stats_t prev;
    imu_info_t info;
    imu_stats_t st;
    imu_sample_t s;
    shared_read(&info, &g_imu_info, sizeof info);
    shared_read(&st, &g_imu_stats, sizeof st);
    shared_read(&s, &g_imu_sample, sizeof s);

    uint32_t imu_hz = (st.samples - prev.samples) * 1000u / elapsed_ms;
    prev = st;
    float acc_norm = sqrtf(s.acc[0] * s.acc[0] + s.acc[1] * s.acc[1] + s.acc[2] * s.acc[2]);

    // Integer output only (mprintf has no %f): mrad/s and mm/s^2.
    mprintf("MSG,imu init %ld id 0x%04X err 0x%04X acc_conf 0x%04X gyr_conf 0x%04X | %lu Hz ovr %lu spierr %lu\r\n",
            (long)info.init_status, info.chip_id, info.err_reg, info.acc_conf, info.gyr_conf,
            (unsigned long)imu_hz, (unsigned long)st.overruns, (unsigned long)st.spi_errors);
    mprintf("MSG,imu gyro mrad/s %ld %ld %ld | acc mm/s2 %ld %ld %ld |a| %ld\r\n",
            (long)(s.gyro[0] * 1000.0f), (long)(s.gyro[1] * 1000.0f), (long)(s.gyro[2] * 1000.0f),
            (long)(s.acc[0] * 1000.0f), (long)(s.acc[1] * 1000.0f), (long)(s.acc[2] * 1000.0f),
            (long)(acc_norm * 1000.0f));
}

// Once a second, only when the IMU saturated meanwhile (vibration on a hard-mounted board biases a clipped
// accelerometer, which the altitude filter integrates).
static void report_imu_clips(void)
{
    static uint32_t prev_acc, prev_gyro, prev_samples;
    imu_stats_t st;
    shared_read(&st, &g_imu_stats, sizeof st);
    uint32_t acc = st.acc_clips - prev_acc, gyro = st.gyro_clips - prev_gyro, n = st.samples - prev_samples;
    prev_acc = st.acc_clips;
    prev_gyro = st.gyro_clips;
    prev_samples = st.samples;
    if (acc != 0u || gyro != 0u) {
        mprintf("MSG,IMU CLIP last 1 s: acc %lu gyro %lu of %lu samples (total acc %lu gyro %lu)\r\n",
                (unsigned long)acc, (unsigned long)gyro, (unsigned long)n, (unsigned long)st.acc_clips,
                (unsigned long)st.gyro_clips);
    }
}

static void log_stats(uint32_t elapsed_ms, uint32_t telem_runs)
{
    mprintf("MSG,task_telem is running: %lu Hz | stack free (words) ctrl %lu slow %lu telem %lu\r\n",
            (unsigned long)(telem_runs * 1000u / elapsed_ms),
            (unsigned long)uxTaskGetStackHighWaterMark(ctrlTaskHandle),
            (unsigned long)uxTaskGetStackHighWaterMark(slowTaskHandle),
            (unsigned long)uxTaskGetStackHighWaterMark(NULL));
    log_imu(elapsed_ms);
}

static const char *const trace_event_text[TR_COUNT] = {
    [TR_IMU_EXTI]     = "ISR  IMU data-ready (EXTI9_5) is running",
    [TR_IMU_DMA_DONE] = "ISR  IMU SPI DMA done is running (decode; every 3rd -> SIG_IMU)",
    [TR_IMU_OVERRUN]  = "ISR  IMU overrun: previous burst still busy",
    [TR_IMU_SPI_ERR]  = "ISR  IMU SPI error",
    [TR_TICK_ISR]     = "ISR  TIM7 HAL tick is running",
    [TR_USB_ISR]      = "ISR  USB OTG_FS is running",
    [TR_TASK_IN]      = "",
    [TR_CTRL_DONE]    = "     task_ctrl cycle finished",
    [TR_CTRL_TIMEOUT] = "     task_ctrl timed out waiting for SIG_IMU",
};

static const char *const trace_task_text[] = {
    [TRACE_TASK_OTHER] = "TASK idle is running",
    [TRACE_TASK_CTRL]  = "TASK task_ctrl is running",
    [TRACE_TASK_SLOW]  = "TASK task_slow is running",
    [TRACE_TASK_TELEM] = "TASK task_telem is running",
};

static uint32_t boot_printed;
static bool trace_printing;
static uint32_t trace_print_idx;

static void print_new_boot_lines(void)
{
    uint32_t n = trace_boot_count();
    while (boot_printed < n) {
        mprintf("MSG,BOOT %s\r\n", trace_boot_text(boot_printed));
        boot_printed++;
    }
}

static void trace_start(void)
{
    if (trace_printing || trace_capture_count() != 0u) {
        mprintf("ERR,trace,busy\r\n");
        return;
    }
    trace_capture_arm();
    mprintf("OK,trace capturing %u events\r\n", (unsigned)TRACE_CAPTURE_LEN);
}

// Prints a few lines per tick so a 512-event capture never overflows the USB TX ring.
static void print_trace_chunk(void)
{
    if (!trace_printing) {
        if (!trace_capture_done()) {
            return;
        }
        trace_printing = true;
        trace_print_idx = 0;
        const trace_rec_t *r = trace_capture_buf();
        mprintf("TRC,begin %u events over %lu us (t = since first event, +dt = since previous)\r\n",
                (unsigned)TRACE_CAPTURE_LEN,
                (unsigned long)time_cycles_to_us(r[TRACE_CAPTURE_LEN - 1u].t_cyc - r[0].t_cyc));
    }

    const trace_rec_t *r = trace_capture_buf();
    for (uint32_t k = 0; k < TRACE_LINES_PER_TICK && trace_print_idx < TRACE_CAPTURE_LEN; k++) {
        const trace_rec_t *e = &r[trace_print_idx];
        uint32_t t_us = time_cycles_to_us(e->t_cyc - r[0].t_cyc);
        uint32_t dt_us = (trace_print_idx == 0u) ? 0u : time_cycles_to_us(e->t_cyc - r[trace_print_idx - 1u].t_cyc);
        const char *text = "?";
        if (e->event == TR_TASK_IN && e->arg <= TRACE_TASK_TELEM) {
            text = trace_task_text[e->arg];
        } else if (e->event < TR_COUNT) {
            text = trace_event_text[e->event];
        }
        mprintf("TRC,%6lu us +%4lu  %s\r\n", (unsigned long)t_us, (unsigned long)dt_us, text);
        trace_print_idx++;
    }

    if (trace_print_idx >= TRACE_CAPTURE_LEN) {
        mprintf("TRC,end (type 'trace' + Enter to capture again)\r\n");
        trace_printing = false;
        trace_capture_release();
    }
}

static void print_param(size_t i, const params_t *p)
{
    const param_desc_t *d = params_desc(i);
    char v[16], lo[16], hi[16];
    numfmt_float(v, sizeof v, params_get(p, i), 5);
    numfmt_float(lo, sizeof lo, d->min, 5);
    numfmt_float(hi, sizeof hi, d->max, 5);
    mprintf("P,%s,%s,%s,%s\r\n", d->name, v, lo, hi);
}

static void cmd_get(const char *name)
{
    int i = params_find(name);
    if (i < 0) {
        mprintf("ERR,GET,unknown param %s\r\n", name);
        return;
    }
    params_t p;
    shared_read(&p, &g_params, sizeof p);
    print_param((size_t)i, &p);
}

static void cmd_list(void)
{
    params_t p;
    shared_read(&p, &g_params, sizeof p);
    for (size_t i = 0; i < params_count(); i++) {
        print_param(i, &p);
    }
    mprintf("OK,LIST\r\n");
}

static void cmd_set(char *args)
{
    char *name = args;
    char *val = strchr(args, ' ');
    if (val == NULL) {
        mprintf("ERR,SET,usage: SET <name> <value>\r\n");
        return;
    }
    *val++ = '\0';
    int i = params_find(name);
    float v;
    if (i < 0) {
        mprintf("ERR,SET,unknown param %s\r\n", name);
        return;
    }
    if (!numfmt_parse_float(val, &v)) {
        mprintf("ERR,SET,bad number %s\r\n", val);
        return;
    }
    arm_status_t arm;
    shared_read(&arm, &g_arm, sizeof arm);
    if (arm.armed && (params_desc((size_t)i)->flags & PARAM_DISARMED_ONLY)) {
        mprintf("ERR,SET,%s can only change while disarmed\r\n", name);
        return;
    }
    params_t p;
    shared_read(&p, &g_params, sizeof p);
    if (params_set(&p, (size_t)i, v) != PARAM_OK) {
        char lo[16], hi[16];
        numfmt_float(lo, sizeof lo, params_desc((size_t)i)->min, 5);
        numfmt_float(hi, sizeof hi, params_desc((size_t)i)->max, 5);
        mprintf("ERR,SET,%s out of range [%s..%s]\r\n", name, lo, hi);
        return;
    }
    shared_write(&g_params, &p, sizeof p);  // values first, then the sequence number
    g_params_seq = g_params_seq + 1u;
    print_param((size_t)i, &p);
    mprintf("OK,SET\r\n");
}

static void cmd_save(void)
{
    arm_status_t arm;
    shared_read(&arm, &g_arm, sizeof arm);
    if (arm.armed || arm.motor[0] + arm.motor[1] + arm.motor[2] + arm.motor[3] > 0.0f) {
        mprintf("ERR,SAVE,disarm first (and no motor test running)\r\n");
        return;
    }
    params_t p;
    shared_read(&p, &g_params, sizeof p);
    uint8_t img[PARAMS_IMAGE_MAX];
    size_t len = params_image_build(&p, img, sizeof img);
    // The whole MCU stalls during the sector erase (~1-2 s): ESC keeps its 900 us stop pulse in hardware.
    bool ok = len > 0u && flash_params_write(img, len);
    mprintf(ok ? "OK,SAVE\r\n" : "ERR,SAVE,flash write failed\r\n");
}

static void print_calib_status(const calib_info_t *c);

// SENSORS: one snapshot of the magnetometer and barometer (integers: mprintf has no %f).
static void print_i2c_sensors(void)
{
    mag_t m;
    baro_t b;
    shared_read(&m, &g_mag, sizeof m);
    shared_read(&b, &g_baro, sizeof b);
    float norm = sqrtf(m.field[0] * m.field[0] + m.field[1] * m.field[1] + m.field[2] * m.field[2]);
    // Heading only meaningful with the drone level: FRD body, magnetic north = atan2(-y, x).
    float hdg = atan2f(-m.field[1], m.field[0]) * MATHX_RAD2DEG;
    if (hdg < 0.0f) {
        hdg += 360.0f;
    }
    mprintf("MSG,MAG valid %u init %d err %lu | body FRD x %ld y %ld z %ld |B| %ld (0.1 uT) | heading %ld deg "
            "(level only)\r\n",
            (unsigned)m.valid, m.init_status, (unsigned long)m.bus_errors, (long)(m.field[0] * 10.0f),
            (long)(m.field[1] * 10.0f), (long)(m.field[2] * 10.0f), (long)(norm * 10.0f), (long)hdg);
    mprintf("MSG,BARO valid %u init %d err %lu | %ld Pa | %ld.%ld C | alt_rel %ld cm\r\n", (unsigned)b.valid,
            b.init_status, (unsigned long)b.bus_errors, (long)b.pressure_pa, (long)b.temp_c,
            (long)(fabsf(b.temp_c) * 10.0f) % 10, (long)(b.alt_rel_m * 100.0f));
}

static bool dump_active;
static size_t dump_idx;
// Port of the command being handled (set by poll_commands): DUMP, CAPTURE, ALTRAW and FLOWLOG output goes
// only there, so a USB dump does not flood the 115200-baud radio and vice versa.
static unsigned cmd_to = LINK_TO_ALL;
static unsigned dump_to = LINK_TO_ALL, capture_to = LINK_TO_ALL;
static bool bb_recording;

static void cmd_dump(void)
{
    arm_status_t arm;
    shared_read(&arm, &g_arm, sizeof arm);
    if (arm.armed || bb_recording) {
        mprintf("ERR,DUMP,disarm and wait 2 s first\r\n");
    } else if (blackbox_count() == 0u) {
        mprintf("ERR,DUMP,empty (records start when armed)\r\n");
    } else {
        dump_to = cmd_to;
        mprintf_to(dump_to, "MSG,DUMP %u records\r\n", (unsigned)blackbox_count());
        link_write_to(dump_to, blackbox_header(), strlen(blackbox_header()));
        dump_active = true;
        dump_idx = 0;
    }
}

static void handle_command(const char *line_in)
{
    char line[CMD_LINE_MAX];
    strncpy(line, line_in, sizeof line - 1u);
    line[sizeof line - 1u] = '\0';
    if (line[0] == '\0') {
        return;
    }
    if (strncmp(line, "GET ", 4) == 0) {
        cmd_get(&line[4]);
    } else if (strncmp(line, "SET ", 4) == 0) {
        cmd_set(&line[4]);
    } else if (strcmp(line, "LIST") == 0) {
        cmd_list();
    } else if (strcmp(line, "SAVE") == 0) {
        cmd_save();
    } else if (strcmp(line, "DUMP") == 0) {
        cmd_dump();
    } else if (strcmp(line, "trace") == 0) {
        trace_start();
    } else if (strncmp(line, "MOTOR_TEST ", 11) == 0) {
        // "MOTOR_TEST <1..4> CONFIRM": the confirm word (sent by the GCS after the props-off dialog) keeps a
        // stray terminal line from spinning a motor.
        arm_status_t arm;
        shared_read(&arm, &g_arm, sizeof arm);
        char ch = line[11];
        if (ch < '1' || ch > '4' || strcmp(&line[12], " CONFIRM") != 0) {
            mprintf("ERR,MOTOR_TEST,usage: MOTOR_TEST <1..4> CONFIRM\r\n");
        } else if (arm.armed) {
            mprintf("ERR,MOTOR_TEST,armed\r\n");
        } else {
            motor_test_req_t req;
            shared_read(&req, &g_motor_test_req, sizeof req);
            req.seq++;
            req.esc_ch = (uint8_t)(ch - '1');
            shared_write(&g_motor_test_req, &req, sizeof req);
            mprintf("OK,MOTOR_TEST\r\n");
        }
    } else if (strcmp(line, "DISARM") == 0) {
        g_gcs_disarm_req = g_gcs_disarm_req + 1u;  // single writer; task_ctrl acts on the change
        mprintf("OK,DISARM\r\n");
    } else if (strncmp(line, "CAPTURE", 7) == 0 && (line[7] == '\0' || line[7] == ' ')) {
        // "CAPTURE [n]": n control cycles averaged per record (1..8), default CAP_DECIM_DEFAULT.
        uint32_t d = CAP_DECIM_DEFAULT;
        if (line[7] == ' ') {
            d = (line[8] >= '0' && line[8] <= '9' && line[9] == '\0') ? (uint32_t)(line[8] - '0') : 0u;
        }
        if (d < CAP_DECIM_MIN || d > CAP_DECIM_MAX) {
            mprintf("ERR,CAPTURE,usage: CAPTURE [%u..%u]\r\n", (unsigned)CAP_DECIM_MIN, (unsigned)CAP_DECIM_MAX);
        } else if (dump_active || !capture_arm(d)) {
            mprintf("ERR,CAPTURE,busy (a capture or DUMP is still being sent)\r\n");
        } else {
            capture_to = cmd_to;
            float hz = CFG_CTRL_LOOP_HZ / (float)d;
            mprintf("OK,CAPTURE,started %u records at ~%u Hz (~%u ms)\r\n", (unsigned)CAP_RECORDS, (unsigned)hz,
                    (unsigned)((float)CAP_RECORDS * 1000.0f / hz));
        }
    } else if (strcmp(line, "I2CSCAN") == 0) {
        g_i2c_scan_req = g_i2c_scan_req + 1u;  // task_slow runs it and prints the result
    } else if (strncmp(line, "MAGCAL", 6) == 0 && (line[6] == '\0' || line[6] == ' ')) {
        // "MAGCAL [seconds]", default 30 s (PLAN §6.2).
        unsigned s = 30u;
        if (line[6] == ' ') {
            s = 0u;
            for (const char *c = &line[7]; *c >= '0' && *c <= '9' && s < 1000u; c++) {
                s = s * 10u + (unsigned)(*c - '0');
            }
        }
        if (s < 5u || s > 120u) {
            mprintf("ERR,MAGCAL,usage: MAGCAL [5..120 s]\r\n");
        } else {
            g_magcal_req.duration_ms = s * 1000u;
            g_magcal_req.seq = g_magcal_req.seq + 1u;
            mprintf("OK,MAGCAL,%u s: rotate the drone slowly through every orientation (nose up/down, rolled, "
                    "upside down)\r\n", s);
        }
    } else if (strcmp(line, "ALTRAW 1") == 0 || strcmp(line, "ALTRAW 0") == 0) {
        g_altraw_on = (line[7] == '1') ? cmd_to : 0u;  // ~7 KB/s of "A" lines, to the requesting port only
        mprintf("OK,ALTRAW,%c\r\n", line[7]);
    } else if (strcmp(line, "LEVELCAL") == 0) {
        arm_status_t arm;
        shared_read(&arm, &g_arm, sizeof arm);
        if (arm.armed) {
            mprintf("ERR,LEVELCAL,disarm first\r\n");
        } else {
            g_levelcal_req = g_levelcal_req + 1u;  // task_slow averages ~2 s and prints the result
            mprintf("OK,LEVELCAL,keep still 2 s (upright on a level surface, or upside down on the motor caps)\r\n");
        }
    } else if (strcmp(line, "LAND") == 0) {
        g_gcs_land_req = g_gcs_land_req + 1u;  // task_ctrl starts the landing (or refuses) and answers
    } else if (strcmp(line, "POSZERO") == 0) {
        g_poszero_req = g_poszero_req + 1u;  // task_slow zeroes the flow position on its next cycle
        mprintf("OK,POSZERO\r\n");
    } else if (strcmp(line, "FLOWLOG 1") == 0 || strcmp(line, "FLOWLOG 0") == 0) {
        g_flowlog_on = (line[8] == '1') ? cmd_to : 0u;  // ~6 KB/s of "F" lines, to the requesting port only
        mprintf("OK,FLOWLOG,%c\r\n", line[8]);
    } else if (strcmp(line, "SENSORS") == 0) {
        print_i2c_sensors();
    } else if (strcmp(line, "FLOWRAW") == 0) {
        g_flow_raw_req = g_flow_raw_req + 1u;  // task_slow prints 1 s of USART2 statistics
    } else if (strcmp(line, "HELLO") == 0) {
        // Sent by the GCS on connect: over the radio bridge there is no DTR edge to replay the banner on.
        calib_info_t c;
        shared_read(&c, &g_calib, sizeof c);
        log_boot_banner();
        print_calib_status(&c);
    } else {
        mprintf("ERR,%s,unknown (GET SET LIST SAVE DUMP CAPTURE DISARM MOTOR_TEST HELLO I2CSCAN FLOWRAW FLOWLOG POSZERO LAND SENSORS MAGCAL LEVELCAL ALTRAW trace)\r\n",
                line);
    }
}

// One line buffer per source, so bytes arriving on USB and radio in the same tick never mix.
static void poll_commands(void)
{
    static char line[LINK_SRC_COUNT][CMD_LINE_MAX];
    static uint32_t len[LINK_SRC_COUNT];
    static bool junk[LINK_SRC_COUNT];
    uint8_t rx[32];
    size_t n;
    for (int s = 0; s < LINK_SRC_COUNT; s++) {
        while ((n = link_read((link_src_t)s, rx, sizeof rx)) > 0u) {
            // Echo for terminals on USB only: if the radio RX ever hears our own TX (floating or coupled PC7), an
            // echo there would feed back forever and flood both links.
            if (s == LINK_SRC_USB) {
                link_write(rx, n);
            }
            for (size_t i = 0; i < n; i++) {
                char c = (char)rx[i];
                if (c == '\r' || c == '\n') {
                    if (c == '\r' && s == LINK_SRC_USB) {
                        mprintf("\n");
                    }
                    line[s][len[s]] = '\0';
                    // No command has a comma or a non-printable byte: such a line is our own output heard back
                    // or line noise, dropped without an ERR reply (that reply would loop too).
                    if (!junk[s]) {
                        cmd_to = LINK_TO_SRC(s);
                        handle_command(line[s]);
                        cmd_to = LINK_TO_ALL;
                    }
                    len[s] = 0;
                    junk[s] = false;
                } else if (c == ',' || c < 0x20 || c > 0x7E) {
                    junk[s] = true;
                } else if (len[s] < CMD_LINE_MAX - 1u) {
                    line[s][len[s]++] = c;
                }
            }
        }
    }
}

static void print_calib_status(const calib_info_t *c)
{
    if (c->state == CALIB_DONE) {
        mprintf("MSG,CALIB done: gyro_bias mrad/s %ld %ld %ld | acc_up_bias0 mm/s2 %ld | restarts %u\r\n",
                (long)(c->gyro_bias[0] * 1000.0f), (long)(c->gyro_bias[1] * 1000.0f),
                (long)(c->gyro_bias[2] * 1000.0f), (long)(c->acc_up_bias0 * 1000.0f), (unsigned)c->restarts);
    } else {
        mprintf("MSG,CALIB running: keep the drone still and level (restarts %u)\r\n", (unsigned)c->restarts);
    }
}

// Reports calibration completion, and replays banner + status whenever a host opens the port
// (anything printed while the port was closed may have been dropped).
static void report_status(void)
{
    static bool was_open;
    static uint8_t reported_state = 0xFFu;
    calib_info_t c;
    shared_read(&c, &g_calib, sizeof c);
    bool open = usb_cdc_port_host_open();
    if (open && !was_open) {
        log_boot_banner();
        print_calib_status(&c);
        reported_state = c.state;
    } else if (open && c.state != reported_state) {
        if (c.state == CALIB_DONE) {
            print_calib_status(&c);
        }
        reported_state = c.state;
    }
    was_open = open;
}

// Each port gets the line only with room left for event lines; the radio only every TELEM_RADIO_DIV ticks.
static unsigned telem_dest(uint32_t *radio_div, uint32_t every)
{
    unsigned to = 0u;
    if (link_tx_free_to(LINK_TO_USB) >= TELEM_MIN_TX_FREE) {
        to |= LINK_TO_USB;
    }
    if (++*radio_div >= every) {
        *radio_div = 0u;
        if (link_tx_free_to(LINK_TO_RADIO) >= TELEM_MIN_TX_FREE) {
            to |= LINK_TO_RADIO;
        }
    }
    return to;
}

static void send_telemetry(void)
{
    static uint32_t radio_div;
    unsigned to = telem_dest(&radio_div, TELEM_RADIO_DIV);
    if (to == 0u) {
        return;
    }
    static char line[TELEM_LINE_MAX];
    attitude_t att;
    ctrl_stats_t ctrl;
    arm_status_t arm;
    shared_read(&att, &g_attitude, sizeof att);
    shared_read(&ctrl, &g_ctrl_stats, sizeof ctrl);
    shared_read(&arm, &g_arm, sizeof arm);

    telem_frame_t f = {0};  // fields of later milestones stay 0
    f.t_ms = osKernelSysTick();
    f.arm = arm.armed;
    f.mode = arm.mode;
    f.fs = arm.fs;
    f.thr = arm.thr_cmd;
    f.roll_sp = arm.roll_sp;
    f.pitch_sp = arm.pitch_sp;
    f.yawr_sp = arm.yawr_sp;
    f.p_sp = arm.p_sp;
    f.q_sp = arm.q_sp;
    f.acro = arm.acro;
    for (int i = 0; i < 4; i++) {
        f.motor[i] = arm.motor[i];
    }
    f.roll = att.roll;
    f.pitch = att.pitch;
    f.yaw = att.yaw;
    f.p = att.rate[0];
    f.q = att.rate[1];
    f.r = att.rate[2];
    tof_t tf;
    shared_read(&tf, &g_tof, sizeof tf);
    f.tof = tf.valid ? tf.distance_m : -1.0f;  // -1: no fresh in-range reading
    baro_t bo;
    shared_read(&bo, &g_baro, sizeof bo);
    f.baro_rel = bo.valid ? bo.alt_rel_m : 0.0f;
    alt_out_t alt;
    shared_read(&alt, &g_alt, sizeof alt);
    f.z = alt.z;
    f.vz = alt.vz;
    f.z_sp = arm.z_sp;
    f.h_terrain = alt.h_terrain;
    f.alt_src = alt.src;
    f.alt_valid = alt.valid;
    flow_out_t fl;
    shared_read(&fl, &g_flow, sizeof fl);
    f.vel_fwd = fl.v[0];
    f.vel_right = fl.v[1];
    f.pos_x = fl.pos[0];
    f.pos_y = fl.pos[1];
    f.flow_q = fl.quality;
    f.flow_valid = fl.valid;
    f.ctrl_us_max = ctrl.exec_us_max;
    f.jitter_us_max = ctrl.jitter_us_max;
    size_t n = telem_proto_build_T(line, sizeof line, &f);
    link_write_to(to, line, n);
}

// M4 step 1 RC test line at 10 Hz: R,valid,failsafe,age_ms,frames/s,bad,uart_errs,lost,roll,pitch,yaw,thr,ch1..ch16
// (sticks x1000). Lets the user find the ARM/MODE switch channels on the GCS.
#define RC_TEST_EVERY_TICKS 5u

static void send_rc_test(uint32_t now_ms)
{
    static uint32_t prev_frames, prev_ms, radio_div;
    unsigned to = telem_dest(&radio_div, RC_RADIO_DIV);
    if (to == 0u) {
        return;
    }
    rc_t rc;
    rc_uart_stats_t us;
    shared_read(&rc, &g_rc, sizeof rc);
    rc_uart_get_stats(&us);
    uint32_t dt = now_ms - prev_ms;
    uint32_t fps = (dt > 0u) ? (us.frames - prev_frames) * 1000u / dt : 0u;
    prev_frames = us.frames;
    prev_ms = now_ms;

    char line[200];
    int n = snprintf(line, sizeof line, "R,%u,%u,%lu,%lu,%lu,%lu,%lu,%ld,%ld,%ld,%ld",
                     (unsigned)rc.valid, (unsigned)rc.failsafe_flag,
                     (unsigned long)((rc.age_ms > 99999u) ? 99999u : rc.age_ms), (unsigned long)fps,
                     (unsigned long)us.bad_len, (unsigned long)us.uart_errs, (unsigned long)rc.frames_lost,
                     (long)(rc.roll * 1000.0f), (long)(rc.pitch * 1000.0f), (long)(rc.yaw * 1000.0f),
                     (long)(rc.thr * 1000.0f));
    for (int i = 0; i < 16 && n > 0 && n < (int)sizeof line - 8; i++) {
        n += snprintf(line + n, sizeof line - (size_t)n, ",%u", (unsigned)rc.raw[i]);
    }
    if (n > 0 && n < (int)sizeof line - 3) {
        line[n++] = '\r';
        line[n++] = '\n';
        link_write_to(to, line, (size_t)n);
    }
}

// G1 GPS line at 5 Hz (radio 1 Hz): G,age_ms,fix_type,fix_ok,num_sv,lat_e7,lon_e7,hmsl_mm,h_acc_mm,v_acc_mm,
// vel_n,vel_e,vel_d (mm/s),s_acc_mm,pdop_x100,bytes,pvt_count,ck_errors,nmea_lines,cfg_sent,acks,naks
#define GPS_EVERY_TICKS 10u
#define GPS_RADIO_DIV   5u

static void send_gps(uint32_t now_ms)
{
    static uint32_t radio_div;
    unsigned to = telem_dest(&radio_div, GPS_RADIO_DIV);
    if (to == 0u) {
        return;
    }
    gps_t g;
    shared_read(&g, &g_gps, sizeof g);
    uint32_t age = g.pvt_count ? now_ms - g.t_ms : 99999u;
    char line[220];
    int n = snprintf(line, sizeof line,
                     "G,%lu,%u,%u,%u,%ld,%ld,%ld,%lu,%lu,%ld,%ld,%ld,%lu,%u,%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
                     (unsigned long)(age > 99999u ? 99999u : age), (unsigned)g.pvt.fix_type, (unsigned)g.pvt.fix_ok,
                     (unsigned)g.pvt.num_sv, (long)g.pvt.lat_e7, (long)g.pvt.lon_e7, (long)g.pvt.hmsl_mm,
                     (unsigned long)g.pvt.h_acc_mm, (unsigned long)g.pvt.v_acc_mm, (long)g.pvt.vel_n_mm,
                     (long)g.pvt.vel_e_mm, (long)g.pvt.vel_d_mm, (unsigned long)g.pvt.s_acc_mm, (unsigned)g.pvt.pdop,
                     (unsigned long)g.bytes, (unsigned long)g.pvt_count, (unsigned long)g.ck_errors,
                     (unsigned long)g.nmea_lines, (unsigned long)g.cfg_sent, (unsigned long)g.acks,
                     (unsigned long)g.naks);
    if (n > 0 && n < (int)sizeof line) {
        link_write_to(to, line, (size_t)n);
    }
}

// Fixed-point "-12.3" text (mprintf has no %f).
static void fmt_1dp(char *out, size_t size, float v)
{
    long t = (long)(v * 10.0f + (v >= 0.0f ? 0.5f : -0.5f));
    unsigned long a = (unsigned long)(t < 0 ? -t : t);
    snprintf(out, size, "%s%lu.%lu", (t < 0) ? "-" : "", a / 10u, a % 10u);
}

static void print_raw_imu(void)
{
    imu_sample_t s;
    shared_read(&s, &g_imu_sample, sizeof s);
    if (LOG_RAW_MODE == 1) {
        mprintf("%7ld %7ld %7ld\r\n",
                (long)(s.acc[0] * 1000.0f), (long)(s.acc[1] * 1000.0f), (long)(s.acc[2] * 1000.0f));
        return;
    }
    char gx[12], gy[12], gz[12];
    fmt_1dp(gx, sizeof gx, s.gyro[0] * MATHX_RAD2DEG);
    fmt_1dp(gy, sizeof gy, s.gyro[1] * MATHX_RAD2DEG);
    fmt_1dp(gz, sizeof gz, s.gyro[2] * MATHX_RAD2DEG);
    mprintf("%8s %8s %8s\r\n", gx, gy, gz);
}

static int16_t to_i16(float v)
{
    return (int16_t)clampf(v + (v >= 0.0f ? 0.5f : -0.5f), -32768.0f, 32767.0f);
}

// 50 Hz from task_telem. A new arm starts a new recording; it continues BB_TAIL_MS after disarm.
static void blackbox_tick(uint32_t now_ms)
{
    static bool was_armed;
    static uint32_t disarm_ms;
    arm_status_t arm;
    attitude_t att;
    rc_t rc;
    shared_read(&arm, &g_arm, sizeof arm);
    shared_read(&att, &g_attitude, sizeof att);
    shared_read(&rc, &g_rc, sizeof rc);

    if (arm.armed && !was_armed) {
        blackbox_reset();
        bb_recording = true;
        dump_active = false;
    } else if (!arm.armed && was_armed) {
        disarm_ms = now_ms;
    }
    was_armed = arm.armed;
    if (bb_recording && !arm.armed && now_ms - disarm_ms > BB_TAIL_MS) {
        bb_recording = false;
    }
    if (!bb_recording) {
        return;
    }
    bb_rec_t r = {
        .t_ms = now_ms,
        .roll = to_i16(att.roll * MATHX_RAD2DEG * 100.0f),
        .pitch = to_i16(att.pitch * MATHX_RAD2DEG * 100.0f),
        .yaw = to_i16(att.yaw * MATHX_RAD2DEG * 100.0f),
        .p = to_i16(att.rate[0] * MATHX_RAD2DEG * 10.0f),
        .q = to_i16(att.rate[1] * MATHX_RAD2DEG * 10.0f),
        .r = to_i16(att.rate[2] * MATHX_RAD2DEG * 10.0f),
        .roll_sp = to_i16(arm.roll_sp * MATHX_RAD2DEG * 100.0f),
        .pitch_sp = to_i16(arm.pitch_sp * MATHX_RAD2DEG * 100.0f),
        .yawr_sp = to_i16(arm.yawr_sp * MATHX_RAD2DEG * 10.0f),
        .thr = (uint16_t)to_i16(arm.thr_cmd * 1000.0f),
        .arm = arm.armed,
        .fs = arm.fs,
        .mode = arm.mode,
        .rc_valid = rc.valid,
    };
    for (int i = 0; i < 4; i++) {
        r.m_us[i] = (uint16_t)((float)ESC_STOP_US + arm.motor[i] * (float)(ESC_MAX_US - ESC_STOP_US) + 0.5f);
    }
    blackbox_add(&r);
}

static void dump_chunk(void)
{
    char line[120];
    uint32_t per_tick = (dump_to & LINK_TO_RADIO) ? DUMP_LINES_PER_TICK_RADIO : DUMP_LINES_PER_TICK;
    for (uint32_t k = 0; k < per_tick && dump_idx < blackbox_count(); k++) {
        if (link_tx_free_to(dump_to) < sizeof line) {
            return;  // resume next tick
        }
        size_t n = blackbox_format(blackbox_get(dump_idx), line, sizeof line);
        link_write_to(dump_to, line, n);
        dump_idx++;
    }
    if (dump_idx >= blackbox_count()) {
        dump_active = false;
        mprintf_to(dump_to, "OK,DUMP\r\n");
    }
}

// Sends a finished capture like a DUMP (telemetry paused), then frees the buffer for the next one.
static void capture_chunk(void)
{
    static bool sending;
    static size_t idx;
    if (!sending) {
        mprintf_to(capture_to, "MSG,CAPTURE sending %u records\r\n", (unsigned)capture_count());
        link_write_to(capture_to, capture_header(), strlen(capture_header()));
        sending = true;
        idx = 0;
    }
    char line[120];
    uint32_t per_tick = (capture_to & LINK_TO_RADIO) ? DUMP_LINES_PER_TICK_RADIO : DUMP_LINES_PER_TICK;
    for (uint32_t k = 0; k < per_tick && idx < capture_count(); k++) {
        if (link_tx_free_to(capture_to) < sizeof line) {
            return;  // resume next tick
        }
        size_t n = capture_format(idx, capture_get(idx), line, sizeof line);
        link_write_to(capture_to, line, n);
        idx++;
    }
    if (idx >= capture_count()) {
        sending = false;
        capture_release();
        mprintf_to(capture_to, "OK,CAPTURE,done\r\n");
    }
}

void task_telem(void const *argument)
{
    (void)argument;
    usb_cdc_port_set_rx_notify(osThreadGetId(), SIG_USB_RX);
    trace_set_task_handles(ctrlTaskHandle, slowTaskHandle, telemTaskHandle);
    trace_boot("task_telem is running (osPriorityBelowNormal, 50 Hz)");

    uint32_t last_wake = osKernelSysTick();
    uint32_t stats_start_ms = last_wake;
    uint32_t runs = 0;
    uint32_t raw_runs = 0;
    uint32_t rc_runs = 0;
    uint32_t gps_runs = 0;

    for (;;) {
        osDelayUntil(&last_wake, TELEM_PERIOD_MS);
        DBG3_TOGGLE();
        (void)osSignalWait(SIG_USB_RX, 0);  // clear the flag; the ring buffer is drained below anyway

        poll_commands();
        if (LOG_RAW_MODE != 0) {
            if (++raw_runs >= 10u) {  // 5 Hz
                raw_runs = 0;
                print_raw_imu();
            }
            link_flush();
            continue;
        }
        print_new_boot_lines();

        uint32_t now_ms = osKernelSysTick();
        blackbox_tick(now_ms);
        report_status();  // banner is sent here, once the host has the port open
        if (dump_active) {
            dump_chunk();  // telemetry paused so the dump finishes quickly
            link_flush();
            continue;
        }
        if (capture_state() == CAP_DONE) {
            capture_chunk();
            link_flush();
            continue;
        }
        print_trace_chunk();
        if (!trace_printing) {
            send_telemetry();
            if (++rc_runs >= RC_TEST_EVERY_TICKS) {
                rc_runs = 0;
                send_rc_test(now_ms);
            }
            if (++gps_runs >= GPS_EVERY_TICKS) {
                gps_runs = 0;
                send_gps(now_ms);
            }
        }

        if (++runs >= STATS_EVERY_TICKS) {
            if (LOG_PERIODIC_STATS && !trace_printing) {  // keep the timeline dump contiguous
                log_stats(now_ms - stats_start_ms, runs);
            }
            report_imu_clips();
            stats_start_ms = now_ms;
            runs = 0;
        }

        link_flush();
    }
}

// task_ctrl implementation: IMU init, calibration, attitude KF, arming, motor output (PLAN §4.1, §7).
#include "control_logic/ctrl_task.h"

#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "control_logic/arming.h"
#include "control_logic/attitude.h"
#include "control_logic/calib.h"
#include "control_logic/config.h"
#include "control_logic/imu_pipeline.h"
#include "control_logic/log.h"
#include "control_logic/shared_state.h"
#include "board_hardware_layer/dbg_gpio.h"
#include "board_hardware_layer/pwm_esc.h"
#include "board_hardware_layer/time_us.h"
#include "board_hardware_layer/trace.h"
#include "sensor_and_radio_driver/bmi323.h"
#include "math_algorithm/mixer.h"
#include "math_algorithm/pid.h"
#include "main.h"

#define CTRL_NOMINAL_PERIOD_US 937u
#define CTRL_WARMUP_CYCLES     1000u
#define CTRL_PRINT_PERIOD_MS   1000u
#define CTRL_IMU_LOST_TIMEOUTS 4u  // ~20 ms without data

// Diagnostic switch: 0 = init IMU over SPI but never start the EXTI/DMA stream (isolates hangs).
#define APP_IMU_STREAM 1

/* ---- Estimator (calibration + attitude KF) ---- */

typedef enum { EST_MSG_NONE, EST_MSG_CALIB_MOTION } est_msg_t;

static calib_acc_t calib_acc;
static calib_info_t calib_info;
static uint32_t kf_divider;
static uint32_t kf_last_t_cyc;

static est_msg_t estimator_step(const imu_sample_t *s)
{
    if (calib_info.state != CALIB_DONE) {
        if (calib_info.state == CALIB_WAITING_IMU) {
            calib_start(&calib_acc,
                        (uint32_t)(CFG_CALIB_TIME_S * CFG_IMU_SAMPLE_HZ / (float)IMU_SAMPLES_PER_CTRL),
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
        shared_write(&g_calib, &calib_info, sizeof calib_info);
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

/* ---- Parameters ---- */

static params_t prm;
static uint32_t prm_seq;
static float ang_p, max_angle, max_rate_rp, max_yaw_rate, motor_idle, lc_angle;
static uint32_t lc_ms;
static pid_ctrl_t rate_pid[3];
static pid_gains_t rate_gains[3];

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
    attitude_set_sigmas(prm.kf_sg * MATHX_DEG2RAD, prm.kf_sacc * MATHX_DEG2RAD);
    if (first || prm.dterm_lpf_hz != old.dterm_lpf_hz) {
        for (int i = 0; i < 3; i++) {
            pid_init(&rate_pid[i], prm.dterm_lpf_hz, CFG_CTRL_LOOP_HZ);
        }
    }
    if (!first && (prm.gyro_lpf_hz != old.gyro_lpf_hz || prm.acc_lpf_hz != old.acc_lpf_hz)) {
        imu_pipeline_set_filters(prm.gyro_lpf_hz, prm.acc_lpf_hz);
    }
}

/* ---- Attitude + rate control ---- */

static mixer_sat_t mixer_sat;
static float motor_cmd[4];
static float sp_roll, sp_pitch, sp_yawr;

static void attitude_rate_control(const arming_out_t *o, const attitude_t *att, const float rate[3], float dt)
{
    sp_roll = o->roll_cmd * max_angle;
    sp_pitch = o->pitch_cmd * max_angle;
    sp_yawr = o->yaw_cmd * max_yaw_rate;
    float rate_sp[3] = {
        clampf(ang_p * (sp_roll - att->roll), -max_rate_rp, max_rate_rp),
        clampf(ang_p * (sp_pitch - att->pitch), -max_rate_rp, max_rate_rp),
        sp_yawr,
    };
    bool integrate = o->motor_state == MOTOR_RUN;
    float torque[3];
    for (int i = 0; i < 3; i++) {
        torque[i] = pid_update(&rate_pid[i], &rate_gains[i], rate_sp[i], rate[i], dt, integrate,
                               mixer_sat.pos[i], mixer_sat.neg[i]);
    }
    mixer_quadx(fminf(o->thr_cmd, o->thr_ceiling), torque, motor_idle, motor_cmd, &mixer_sat);
}

/* ---- Arming + motor output ---- */

static arming_t arming;
static uint32_t gcs_disarm_seen;

typedef enum { MTEST_MSG_NONE, MTEST_MSG_START, MTEST_MSG_DONE, MTEST_MSG_CANCELLED, MTEST_MSG_REFUSED } mtest_msg_t;
static uint32_t mtest_seq_seen;
static bool mtest_active;
static uint8_t mtest_ch;
static uint32_t mtest_end_ms;
static mtest_msg_t mtest_msg;
static const uint8_t motor_to_tim_ch[4] = CFG_MOTOR_TO_TIM_CH;

static arming_out_t arming_and_motor_output(uint8_t imu_ok, uint32_t now_ms, const float *rate, float dt)
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
        .att_err = fmaxf(fabsf(sp_roll - att.roll), fabsf(sp_pitch - att.pitch)),
        .lc_angle = lc_angle,
        .lc_ms = lc_ms,
    };
    gcs_disarm_seen = disarm_req;
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
        mtest_active = false;
        mtest_msg = MTEST_MSG_CANCELLED;
    } else if (mtest_active && (int32_t)(now_ms - mtest_end_ms) >= 0) {
        mtest_active = false;
        mtest_msg = MTEST_MSG_DONE;
    }

    bool flying = out.armed && out.motor_state != MOTOR_IDLE;
    if (!flying) {
        for (int i = 0; i < 3; i++) {
            pid_reset(&rate_pid[i]);
            motor_cmd[i] = out.armed ? motor_idle : 0.0f;
        }
        motor_cmd[3] = out.armed ? motor_idle : 0.0f;
        mixer_sat = (mixer_sat_t){0};
        sp_roll = sp_pitch = sp_yawr = 0.0f;
    } else if (rate != NULL) {
        attitude_rate_control(&out, &att, rate, dt);
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
        pwm_esc_write_us(us);
    } else {
        pwm_esc_stop();
        pwm_esc_watchdog_clear();
    }

    arm_status_t st = {
        .armed = out.armed,
        .motor_state = (uint8_t)out.motor_state,
        .fs = (uint8_t)out.fs,
        .mode = rc.mode_sw,
        .block_mask = out.block_mask,
        .thr_cmd = out.thr_cmd,
        .esc_wd_tripped = esc_wd,
        .roll_sp = sp_roll,
        .pitch_sp = sp_pitch,
        .yawr_sp = sp_yawr,
    };
    for (int i = 0; i < 4; i++) {
        st.motor[i] = out.armed ? motor_cmd[i] : 0.0f;
    }
    if (!out.armed && mtest_active) {
        st.motor[mtest_ch] = (float)(CFG_MOTOR_TEST_US - ESC_STOP_US) / (float)(ESC_MAX_US - ESC_STOP_US);
    }
    shared_write(&g_arm, &st, sizeof st);
    return out;
}

/* ---- Logging helpers ---- */

static const char *const disarm_text[] = {
    [DISARM_SWITCH] = "switch", [DISARM_STICK] = "stick (yaw left)", [DISARM_GCS] = "GCS command",
    [DISARM_FAILSAFE] = "RC failsafe stage 2", [DISARM_FLIP] = "flip > 70 deg", [DISARM_IMU] = "IMU lost",
    [DISARM_AUTO] = "idle throttle timeout", [DISARM_ESC_WD] = "motor output watchdog",
    [DISARM_LOST_CONTROL] = "lost control (attitude far from setpoint)",
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

static void print_ctrl_status(const ctrl_stats_t *st, uint32_t elapsed_ms)
{
    static uint32_t prev_cycles;
    uint32_t hz = (st->cycles - prev_cycles) * 1000u / elapsed_ms;
    prev_cycles = st->cycles;
    mprintf("MSG,task_ctrl is running: %lu Hz exec_max %lu us jitter_max %lu us imu_timeouts %lu imu_ok %u\r\n",
            (unsigned long)hz, (unsigned long)st->exec_us_max, (unsigned long)st->jitter_us_max,
            (unsigned long)st->imu_timeouts, (unsigned)st->imu_ok);
}

/* ---- IMU init ---- */

static void delay_ms_rtos(uint32_t ms)
{
    osDelay(ms + 1u);
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

/* ---- Public API ---- */

// State shared between ctrl_init and ctrl_update via file-scope variables.
static ctrl_stats_t ctrl_st;
static uint32_t prev_start;
static uint32_t consecutive_timeouts;
static uint32_t last_print_ms;
static float body_rate[3];
static float rate_dt;
static uint32_t rate_last_t_cyc;

void ctrl_init(void)
{
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

    ctrl_st = (ctrl_stats_t){0};
    prev_start = time_cycles();
    consecutive_timeouts = 0;
    last_print_ms = osKernelSysTick();
    arming_init(&arming);
    body_rate[0] = body_rate[1] = body_rate[2] = 0.0f;
    rate_dt = 1.0f / CFG_CTRL_LOOP_HZ;
    rate_last_t_cyc = time_cycles();
}

void ctrl_update(bool fresh)
{
    uint32_t now_ms = osKernelSysTick();
    ctrl_apply_params(false);
    if (LOG_PERIODIC_STATS && LOG_RAW_MODE == 0 && now_ms - last_print_ms >= CTRL_PRINT_PERIOD_MS) {
        print_ctrl_status(&ctrl_st, now_ms - last_print_ms);
        last_print_ms = now_ms;
    }

    if (fresh) {
        consecutive_timeouts = 0;
        ctrl_st.imu_ok = 1;
    } else {
        ctrl_st.imu_timeouts++;
        if (++consecutive_timeouts >= CTRL_IMU_LOST_TIMEOUTS) {
            ctrl_st.imu_ok = 0;
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
        for (int i = 0; i < 3; i++) {
            body_rate[i] = sample.gyro[i] - calib_info.gyro_bias[i];
        }
        rate_dt = (float)(sample.t_cyc - rate_last_t_cyc) / (float)(TIME_CYCLES_PER_US * 1000000u);
        rate_last_t_cyc = sample.t_cyc;
    }
    arming_out_t arm_out = arming_and_motor_output(ctrl_st.imu_ok, now_ms, fresh ? body_rate : NULL, rate_dt);
    DBG1_LOW();
    trace_mark(TR_CTRL_DONE, 0);
    uint32_t exec_us = time_cycles_to_us(time_cycles() - start);

    if (msg == EST_MSG_CALIB_MOTION) {
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

    if (fresh) {
        ctrl_st.cycles++;
        if (ctrl_st.cycles > CTRL_WARMUP_CYCLES) {
            uint32_t jitter_us = (period_us > CTRL_NOMINAL_PERIOD_US) ? period_us - CTRL_NOMINAL_PERIOD_US
                                                                       : CTRL_NOMINAL_PERIOD_US - period_us;
            if (jitter_us > ctrl_st.jitter_us_max) ctrl_st.jitter_us_max = jitter_us;
            if (exec_us > ctrl_st.exec_us_max)     ctrl_st.exec_us_max = exec_us;
        }
    }
    shared_write(&g_ctrl_stats, &ctrl_st, sizeof ctrl_st);
}

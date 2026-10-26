// task_telem implementation: telemetry, commands, blackbox, trace dump (PLAN §9).
#include "control_logic/telem_task.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "control_logic/blackbox.h"
#include "control_logic/log.h"
#include "control_logic/params.h"
#include "control_logic/shared_state.h"
#include "control_logic/telem_proto.h"
#include "board_hardware_layer/flash_params.h"
#include "board_hardware_layer/pwm_esc.h"
#include "board_hardware_layer/time_us.h"
#include "board_hardware_layer/trace.h"
#include "board_hardware_layer/usb_cdc_port.h"
#include "board_hardware_layer/rc_uart.h"
#include "math_algorithm/mathx.h"
#include "math_algorithm/numfmt.h"
#include "main.h"

#define CMD_LINE_MAX         48u
#define DUMP_LINES_PER_TICK  40u
#define TRACE_LINES_PER_TICK 6u
#define TELEM_MIN_TX_FREE    1024u
#define BB_TAIL_MS           2000u
#define STATS_EVERY_TICKS    50u
#define RC_TEST_EVERY_TICKS  5u

static uint32_t reset_flags;
static param_load_t params_load_result;
static unsigned params_loaded;

/* ---- Boot / params ---- */

void task_telem_capture_reset_cause(void)
{
    reset_flags = RCC->CSR;
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

void params_boot_load(void)
{
    params_t p;
    params_defaults(&p);
    size_t len;
    const uint8_t *img = flash_params_image(&len);
    params_load_result = params_image_load(&p, img, len, &params_loaded);
    if (params_load_result != PARAM_LOAD_OK) {
        params_defaults(&p);
    }
    shared_write(&g_params, &p, sizeof p);
    g_params_seq = g_params_seq + 1u;
}

/* ---- Status / banner ---- */

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
        mprintf("MSG,PARAMS defaults (flash %s)\r\n",
                (params_load_result == PARAM_LOAD_EMPTY) ? "empty" : "CORRUPT");
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
    mprintf("MSG,imu init %ld id 0x%04X err 0x%04X acc_conf 0x%04X gyr_conf 0x%04X | %lu Hz ovr %lu spierr %lu\r\n",
            (long)info.init_status, info.chip_id, info.err_reg, info.acc_conf, info.gyr_conf,
            (unsigned long)imu_hz, (unsigned long)st.overruns, (unsigned long)st.spi_errors);
    mprintf("MSG,imu gyro mrad/s %ld %ld %ld | acc mm/s2 %ld %ld %ld |a| %ld\r\n",
            (long)(s.gyro[0] * 1000.0f), (long)(s.gyro[1] * 1000.0f), (long)(s.gyro[2] * 1000.0f),
            (long)(s.acc[0] * 1000.0f), (long)(s.acc[1] * 1000.0f), (long)(s.acc[2] * 1000.0f),
            (long)(acc_norm * 1000.0f));
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

/* ---- Telemetry ---- */

static void send_telemetry(void)
{
    if (!usb_cdc_port_host_open() || usb_cdc_port_tx_free() < TELEM_MIN_TX_FREE) {
        return;
    }
    static char line[TELEM_LINE_MAX];
    attitude_t att;
    ctrl_stats_t ctrl;
    arm_status_t arm;
    shared_read(&att, &g_attitude, sizeof att);
    shared_read(&ctrl, &g_ctrl_stats, sizeof ctrl);
    shared_read(&arm, &g_arm, sizeof arm);

    telem_frame_t f = {0};
    f.t_ms = osKernelSysTick();
    f.arm = arm.armed;
    f.mode = arm.mode;
    f.fs = arm.fs;
    f.thr = arm.thr_cmd;
    f.roll_sp = arm.roll_sp;
    f.pitch_sp = arm.pitch_sp;
    f.yawr_sp = arm.yawr_sp;
    for (int i = 0; i < 4; i++) {
        f.motor[i] = arm.motor[i];
    }
    f.roll = att.roll;
    f.pitch = att.pitch;
    f.yaw = att.yaw;
    f.p = att.rate[0];
    f.q = att.rate[1];
    f.r = att.rate[2];
    f.ctrl_us_max = ctrl.exec_us_max;
    f.jitter_us_max = ctrl.jitter_us_max;
    size_t n = telem_proto_build_T(line, sizeof line, &f);
    (void)usb_cdc_port_write(line, n);
}

static void send_rc_test(uint32_t now_ms)
{
    static uint32_t prev_frames, prev_ms;
    if (!usb_cdc_port_host_open() || usb_cdc_port_tx_free() < TELEM_MIN_TX_FREE) {
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
        (void)usb_cdc_port_write(line, (size_t)n);
    }
}

/* ---- Raw IMU log ---- */

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

/* ---- Blackbox ---- */

static bool dump_active;
static size_t dump_idx;
static bool bb_recording;

static int16_t to_i16(float v)
{
    return (int16_t)clampf(v + (v >= 0.0f ? 0.5f : -0.5f), -32768.0f, 32767.0f);
}

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
    for (uint32_t k = 0; k < DUMP_LINES_PER_TICK && dump_idx < blackbox_count(); k++) {
        if (usb_cdc_port_tx_free() < sizeof line) {
            return;
        }
        size_t n = blackbox_format(blackbox_get(dump_idx), line, sizeof line);
        (void)usb_cdc_port_write(line, n);
        dump_idx++;
    }
    if (dump_idx >= blackbox_count()) {
        dump_active = false;
        mprintf("OK,DUMP\r\n");
    }
}

/* ---- Trace ---- */

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
        uint32_t dt_us = (trace_print_idx == 0u) ? 0u
                       : time_cycles_to_us(e->t_cyc - r[trace_print_idx - 1u].t_cyc);
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

/* ---- Commands ---- */

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
    shared_write(&g_params, &p, sizeof p);
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
    bool ok = len > 0u && flash_params_write(img, len);
    mprintf(ok ? "OK,SAVE\r\n" : "ERR,SAVE,flash write failed\r\n");
}

static void cmd_dump(void)
{
    arm_status_t arm;
    shared_read(&arm, &g_arm, sizeof arm);
    if (arm.armed || bb_recording) {
        mprintf("ERR,DUMP,disarm and wait 2 s first\r\n");
    } else if (blackbox_count() == 0u) {
        mprintf("ERR,DUMP,empty (records start when armed)\r\n");
    } else {
        mprintf("MSG,DUMP %u records\r\n", (unsigned)blackbox_count());
        (void)usb_cdc_port_write(blackbox_header(), strlen(blackbox_header()));
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
        g_gcs_disarm_req = g_gcs_disarm_req + 1u;
        mprintf("OK,DISARM\r\n");
    } else {
        mprintf("ERR,%s,unknown (commands: GET, SET, LIST, SAVE, DUMP, DISARM, MOTOR_TEST, trace)\r\n", line);
    }
}

static void poll_commands(void)
{
    static char line[CMD_LINE_MAX];
    static uint32_t len;
    uint8_t rx[32];
    size_t n;
    while ((n = usb_cdc_port_read(rx, sizeof rx)) > 0u) {
        (void)usb_cdc_port_write(rx, n);
        for (size_t i = 0; i < n; i++) {
            char c = (char)rx[i];
            if (c == '\r' || c == '\n') {
                if (c == '\r') {
                    mprintf("\n");
                }
                line[len] = '\0';
                handle_command(line);
                len = 0;
            } else if (len < CMD_LINE_MAX - 1u) {
                line[len++] = c;
            }
        }
    }
}

/* ---- Public API ---- */

static uint32_t stats_start_ms;
static uint32_t runs;
static uint32_t raw_runs;
static uint32_t rc_runs;

extern osThreadId telemTaskHandle;  // created by CubeMX in freertos.c

void telem_init(void)
{
    usb_cdc_port_set_rx_notify(osThreadGetId(), SIG_USB_RX);
    trace_set_task_handles(ctrlTaskHandle, slowTaskHandle, telemTaskHandle);
    trace_boot("task_telem is running (osPriorityBelowNormal, 50 Hz)");
    stats_start_ms = osKernelSysTick();
    runs = raw_runs = rc_runs = 0;
}

void telem_update(void)
{
    (void)osSignalWait(SIG_USB_RX, 0);

    poll_commands();
    if (LOG_RAW_MODE != 0) {
        if (++raw_runs >= 10u) {
            raw_runs = 0;
            print_raw_imu();
        }
        usb_cdc_port_flush();
        return;
    }
    print_new_boot_lines();

    uint32_t now_ms = osKernelSysTick();
    blackbox_tick(now_ms);
    report_status();
    if (dump_active) {
        dump_chunk();
        usb_cdc_port_flush();
        return;
    }
    print_trace_chunk();
    if (!trace_printing) {
        send_telemetry();
        if (++rc_runs >= RC_TEST_EVERY_TICKS) {
            rc_runs = 0;
            send_rc_test(now_ms);
        }
    }
    if (++runs >= STATS_EVERY_TICKS) {
        if (LOG_PERIODIC_STATS && !trace_printing) {
            log_stats(now_ms - stats_start_ms, runs);
        }
        stats_start_ms = now_ms;
        runs = 0;
    }
    usb_cdc_port_flush();
}

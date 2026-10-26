// RC input: decodes SBUS frames from bsp/rc_uart into a normalized rc_t (PLAN §6.4). Runs in task_slow.
#include "control_logic/rc.h"

#include <math.h>

#include "control_logic/config.h"
#include "board_hardware_layer/rc_uart.h"
#include "sensor_and_radio_driver/sbus.h"
#include "math_algorithm/mathx.h"

static const uint16_t roll_raw[3] = CFG_RC_ROLL_RAW;
static const uint16_t pitch_raw[3] = CFG_RC_PITCH_RAW;
static const uint16_t yaw_raw[3] = CFG_RC_YAW_RAW;
static const uint16_t thr_raw[2] = CFG_RC_THR_RAW;

static uint32_t frame_seq;
static uint32_t last_good_ms;
static bool have_frame;
static rc_t rc;
static float deadband = 0.03f;  // overwritten from the parameter table

void rc_set_deadband(float fraction)
{
    deadband = fraction;
}

// Piecewise-linear min/mid/max -> [-1, 1] (the two halves of a stick travel are rarely symmetric).
static float stick(uint16_t raw, const uint16_t cal[3])
{
    float v = (raw < cal[1]) ? -(float)(cal[1] - raw) / (float)(cal[1] - cal[0])
                             : (float)(raw - cal[1]) / (float)(cal[2] - cal[1]);
    v = clampf(v, -1.0f, 1.0f);
    if (fabsf(v) < deadband) {
        return 0.0f;
    }
    // Rescale so output still spans [-1, 1] just outside the deadband.
    return (v > 0.0f) ? (v - deadband) / (1.0f - deadband) : (v + deadband) / (1.0f - deadband);
}

void rc_start(void)
{
    rc_uart_start();
}

void rc_update(uint32_t now_ms)
{
    uint8_t frame[SBUS_FRAME_LEN];
    sbus_frame_t f;
    if (rc_uart_get_frame(frame, &frame_seq) && sbus_decode(frame, &f)) {
        for (uint32_t i = 0; i < SBUS_NUM_CHANNELS; i++) {
            rc.raw[i] = f.ch[i];
        }
        rc.failsafe_flag = f.failsafe || f.ch[CFG_RC_CH_THR] < CFG_RC_FS_THR_RAW;
        if (f.frame_lost) {
            rc.frames_lost++;
        }
        rc.roll = CFG_RC_ROLL_DIR * stick(f.ch[CFG_RC_CH_ROLL], roll_raw);
        rc.pitch = CFG_RC_PITCH_DIR * stick(f.ch[CFG_RC_CH_PITCH], pitch_raw);
        rc.yaw = CFG_RC_YAW_DIR * stick(f.ch[CFG_RC_CH_YAW], yaw_raw);
        rc.arm_sw = f.ch[CFG_RC_CH_ARM] < CFG_RC_SW_LOW_MAX;
        uint16_t mode_raw = f.ch[CFG_RC_CH_MODE];
        rc.mode_sw = (mode_raw < CFG_RC_SW_LOW_MAX) ? 0u : (mode_raw > CFG_RC_SW_HIGH_MIN) ? 2u : 1u;
        rc.thr = clampf((float)(f.ch[CFG_RC_CH_THR] - thr_raw[0]) / (float)(thr_raw[1] - thr_raw[0]), 0.0f, 1.0f);
        if (!f.failsafe) {
            last_good_ms = now_ms;
            have_frame = true;
        }
        rc.t_ms = now_ms;
    }
    rc.age_ms = have_frame ? now_ms - last_good_ms : UINT32_MAX;
    // Invalid on timeout, SBUS failsafe flag, or the receiver's failsafe throttle value (sticks then hold
    // receiver defaults, so they must not be used).
    rc.valid = have_frame && rc.age_ms <= CFG_RC_TIMEOUT_MS && !rc.failsafe_flag;
    shared_write(&g_rc, &rc, sizeof rc);
}

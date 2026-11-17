// Per-sample IMU processing in the DMA-complete ISR (3200 Hz); wakes task_ctrl every 3rd sample.
#include "app/imu_pipeline.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app/config.h"
#include "app/shared_state.h"
#include "bsp/spi_imu.h"
#include "drivers/bmi323.h"
#include "lib/filter.h"

static uint8_t burst_rx[BMI323_BURST_LEN];
static uint32_t ctrl_divider;

static const int axis_src[3] = CFG_IMU_AXIS_SRC;
static const float axis_sign[3] = CFG_IMU_AXIS_SIGN;
static biquad_t gyro_lpf[3];
static pt1_t acc_lpf[3];
// Throttle-tracking notch ahead of the gyro low-pass (motor/prop vibration). Coefficients are written by
// task_ctrl with the ISR masked; the state is kept across updates.
static biquad_t gyro_notch[3];
static bool notch_on;

// ~2-3 µs per sample: decode, remap to body FRD, low-pass. Bias is removed in task_ctrl
// (the filters have unity DC gain, so subtracting after filtering is equivalent).
static void on_burst(uint32_t t_cyc, const uint8_t *rx, spi_imu_burst_status_t status)
{
    if (status == SPI_IMU_BURST_OVERRUN) {
        g_imu_stats.overruns++;
        return;
    }
    if (status != SPI_IMU_BURST_OK) {
        g_imu_stats.spi_errors++;
        return;
    }

    float gyro_s[3], acc_s[3];
    uint8_t clip = bmi323_decode_burst(rx, gyro_s, acc_s);
    if (clip & BMI323_CLIP_ACC) {
        g_imu_stats.acc_clips++;
    }
    if (clip & BMI323_CLIP_GYRO) {
        g_imu_stats.gyro_clips++;
    }
    g_imu_sample.t_cyc = t_cyc;
    for (int i = 0; i < 3; i++) {
        float g = axis_sign[i] * gyro_s[axis_src[i]];
        if (notch_on) {
            g = biquad_apply(&gyro_notch[i], g);
        }
        g_imu_sample.gyro[i] = biquad_apply(&gyro_lpf[i], g);
        g_imu_sample.acc[i] = pt1_apply(&acc_lpf[i], axis_sign[i] * acc_s[axis_src[i]]);
    }
    g_imu_stats.samples++;

    if (++ctrl_divider >= IMU_SAMPLES_PER_CTRL) {
        ctrl_divider = 0;
        osSignalSet(ctrlTaskHandle, SIG_IMU);
    }
}

static void init_filters(float gyro_lpf_hz, float acc_lpf_hz)
{
    for (int i = 0; i < 3; i++) {
        biquad_lpf_init(&gyro_lpf[i], gyro_lpf_hz, CFG_IMU_SAMPLE_HZ);
        pt1_init(&acc_lpf[i], acc_lpf_hz, CFG_IMU_SAMPLE_HZ);
    }
}

// Task context, disarmed only (filter state restarts). The IMU ISR (NVIC 5) is masked meanwhile.
void imu_pipeline_set_filters(float gyro_lpf_hz, float acc_lpf_hz)
{
    taskENTER_CRITICAL();
    init_filters(gyro_lpf_hz, acc_lpf_hz);
    taskEXIT_CRITICAL();
}

// Task context, any time. center_hz <= 0 turns the notch off. ~1 us with the IMU ISR masked.
void imu_pipeline_set_notch(float center_hz, float q)
{
    if (center_hz <= 0.0f) {
        notch_on = false;  // single byte store; the ISR sees it on its next sample
        return;
    }
    biquad_t c;
    biquad_notch_coeffs(&c, center_hz, q, CFG_IMU_SAMPLE_HZ);
    taskENTER_CRITICAL();
    for (int i = 0; i < 3; i++) {
        gyro_notch[i].b0 = c.b0;
        gyro_notch[i].b1 = c.b1;
        gyro_notch[i].b2 = c.b2;
        gyro_notch[i].a1 = c.a1;
        gyro_notch[i].a2 = c.a2;
        if (!notch_on) {
            gyro_notch[i].primed = false;  // turning on: start from the current gyro value
        }
    }
    notch_on = true;
    taskEXIT_CRITICAL();
}

void imu_pipeline_start(float gyro_lpf_hz, float acc_lpf_hz)
{
    init_filters(gyro_lpf_hz, acc_lpf_hz);
    ctrl_divider = 0;
    spi_imu_stream_start(bmi323_burst_tx(), burst_rx, BMI323_BURST_LEN, on_burst);
}

// Per-sample IMU processing in the DMA-complete ISR (3200 Hz); wakes task_ctrl every 3rd sample.
#include "control_logic/imu_pipeline.h"

#include "FreeRTOS.h"
#include "task.h"

#include "control_logic/config.h"
#include "control_logic/shared_state.h"
#include "board_hardware_layer/spi_imu.h"
#include "sensor_and_radio_driver/bmi323.h"
#include "math_algorithm/filter.h"

static uint8_t burst_rx[BMI323_BURST_LEN];
static uint32_t ctrl_divider;

static const int axis_src[3] = CFG_IMU_AXIS_SRC;
static const float axis_sign[3] = CFG_IMU_AXIS_SIGN;
static biquad_t gyro_lpf[3];
static pt1_t acc_lpf[3];

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
    bmi323_decode_burst(rx, gyro_s, acc_s);
    g_imu_sample.t_cyc = t_cyc;
    for (int i = 0; i < 3; i++) {
        g_imu_sample.gyro[i] = biquad_apply(&gyro_lpf[i], axis_sign[i] * gyro_s[axis_src[i]]);
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

void imu_pipeline_start(float gyro_lpf_hz, float acc_lpf_hz)
{
    init_filters(gyro_lpf_hz, acc_lpf_hz);
    ctrl_divider = 0;
    spi_imu_stream_start(bmi323_burst_tx(), burst_rx, BMI323_BURST_LEN, on_burst);
}

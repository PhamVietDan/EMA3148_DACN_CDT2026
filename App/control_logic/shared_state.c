// Data shared between tasks/ISRs. One writer per object; whole-struct copies inside a critical section (PLAN §3.4).
#include "control_logic/shared_state.h"

#include "FreeRTOS.h"
#include "task.h"

osThreadId ctrlTaskHandle;
osThreadId slowTaskHandle;

volatile imu_sample_t g_imu_sample;
volatile imu_stats_t g_imu_stats;
volatile imu_info_t g_imu_info = {.init_status = 1};
volatile ctrl_stats_t g_ctrl_stats;
volatile attitude_t g_attitude;
volatile calib_info_t g_calib;
volatile slow_stats_t g_slow_stats;
volatile rc_t g_rc;
volatile arm_status_t g_arm;
volatile uint32_t g_gcs_disarm_req;
volatile motor_test_req_t g_motor_test_req;
volatile params_t g_params;
volatile uint32_t g_params_seq;

void shared_read(void *dst, const volatile void *src, size_t len)
{
    uint8_t *d = dst;
    const volatile uint8_t *s = src;
    taskENTER_CRITICAL();
    for (size_t i = 0; i < len; i++) {
        d[i] = s[i];
    }
    taskEXIT_CRITICAL();
}

void shared_write(volatile void *dst, const void *src, size_t len)
{
    volatile uint8_t *d = dst;
    const uint8_t *s = src;
    taskENTER_CRITICAL();
    for (size_t i = 0; i < len; i++) {
        d[i] = s[i];
    }
    taskEXIT_CRITICAL();
}

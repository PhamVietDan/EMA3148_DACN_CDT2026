// ESC PWM on TIM3 CH1..CH4 (PA6, PA7, PB0, PB1), 1 us/tick. The only code that writes the CCRs.
#include "board_hardware_layer/pwm_esc.h"

#include "tim.h"

// Written by task_ctrl, read by the TIM7 ISR: single aligned 32-bit/8-bit accesses, no tearing.
static volatile uint32_t last_write_ms;
static volatile bool outputs_active;  // some channel above ESC_STOP_US
static volatile bool wd_tripped;

static void write_ccr(uint32_t c1, uint32_t c2, uint32_t c3, uint32_t c4)
{
    // CCR preload (OCxPE) is enabled by HAL_TIM_PWM_ConfigChannel: new values apply at the next period.
    TIM3->CCR1 = c1;
    TIM3->CCR2 = c2;
    TIM3->CCR3 = c3;
    TIM3->CCR4 = c4;
}

void esc_emergency_stop(void)
{
    write_ccr(ESC_STOP_US, ESC_STOP_US, ESC_STOP_US, ESC_STOP_US);
    outputs_active = false;
}

void pwm_esc_init(void)
{
    // Stop value and period are loaded before the outputs are enabled, so the first pulse is already 900 us.
    esc_emergency_stop();
    __HAL_TIM_SET_AUTORELOAD(&htim3, 1000000u / ESC_PWM_HZ - 1u);  // 1 MHz tick (PSC 83 on 84 MHz)
    TIM3->EGR = TIM_EGR_UG;  // load the preloaded ARR/CCR now instead of at the next 50 Hz update
    last_write_ms = HAL_GetTick();
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

static uint32_t clamp_us(uint16_t us)
{
    return (us < ESC_STOP_US) ? ESC_STOP_US : (us > ESC_MAX_US) ? ESC_MAX_US : us;
}

void pwm_esc_write_us(const uint16_t us[4])
{
    last_write_ms = HAL_GetTick();
    if (wd_tripped) {
        esc_emergency_stop();
        return;
    }
    uint32_t c[4];
    bool active = false;
    for (int i = 0; i < 4; i++) {
        c[i] = clamp_us(us[i]);
        active |= (c[i] > ESC_STOP_US);
    }
    write_ccr(c[0], c[1], c[2], c[3]);
    outputs_active = active;
}

void pwm_esc_stop(void)
{
    last_write_ms = HAL_GetTick();
    esc_emergency_stop();
}

void pwm_esc_watchdog_1ms(void)
{
    if (outputs_active && HAL_GetTick() - last_write_ms > ESC_WD_TIMEOUT_MS) {
        esc_emergency_stop();
        wd_tripped = true;
    }
}

bool pwm_esc_watchdog_tripped(void)
{
    return wd_tripped;
}

void pwm_esc_watchdog_clear(void)
{
    wd_tripped = false;
}

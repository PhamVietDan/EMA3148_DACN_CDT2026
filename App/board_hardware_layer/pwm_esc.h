// ESC PWM on TIM3 CH1..CH4 (PA6, PA7, PB0, PB1), 1 us/tick. The only code that writes the CCRs.
#pragma once

#include <stdbool.h>
#include <stdint.h>

// Range the ESCs were calibrated to with the legacy firmware (900 us min, 2000 us max). Never re-calibrated.
#define ESC_STOP_US 900u
#define ESC_MAX_US  2000u
// 50 Hz: these ESCs did not take off at 400 Hz in the user's test (2026-10-16); PLAN §5 target is 400 Hz.
// ARR is set in pwm_esc_init, matching the .ioc value (19999 at 1 MHz).
#define ESC_PWM_HZ  400u

void pwm_esc_init(void);                     // outputs ESC_STOP_US on all channels, then starts PWM
void pwm_esc_write_us(const uint16_t us[4]); // per TIM3 channel, clamped to [STOP, MAX]
void pwm_esc_stop(void);

// Any context, including fault handlers: forces ESC_STOP_US by direct register writes.
void esc_emergency_stop(void);

// Called from the TIM7 HAL-tick ISR (NVIC 0, above BASEPRI masking): if motors are commanded above
// stop and pwm_esc_write_us() has not run for ESC_WD_TIMEOUT_MS (task_ctrl stalled), force stop.
#define ESC_WD_TIMEOUT_MS 50u
void pwm_esc_watchdog_1ms(void);
bool pwm_esc_watchdog_tripped(void);
void pwm_esc_watchdog_clear(void);

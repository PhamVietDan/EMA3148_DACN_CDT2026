// FreeRTOS fault hooks (override the __weak ones in freertos.c). No MCU watchdog: stop the motors, then halt.
#include "FreeRTOS.h"
#include "task.h"

#include "board_hardware_layer/pwm_esc.h"

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    esc_emergency_stop();
    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}

void vApplicationMallocFailedHook(void)
{
    esc_emergency_stop();
    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}

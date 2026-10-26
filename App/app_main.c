// Application start-up: creates task_ctrl and task_slow; task_telem is the CubeMX-generated telemTask.
#include "app_main.h"

#include "control_logic/shared_state.h"
#include "control_logic/tasks.h"
#include "board_hardware_layer/pwm_esc.h"
#include "board_hardware_layer/time_us.h"
#include "board_hardware_layer/trace.h"
#include "main.h"

void app_init(void)
{
    task_telem_capture_reset_cause();
    time_init();
    pwm_esc_init();  // ESC sees the 900 us stop pulse from boot; nothing else may drive TIM3
    params_boot_load();
    trace_boot("app_init is running (creating task_ctrl, task_slow)");

    osThreadDef(ctrlTask, task_ctrl, osPriorityRealtime, 0, 1024);
    ctrlTaskHandle = osThreadCreate(osThread(ctrlTask), NULL);
    osThreadDef(slowTask, task_slow, osPriorityHigh, 0, 1024);
    slowTaskHandle = osThreadCreate(osThread(slowTask), NULL);
    if (ctrlTaskHandle == NULL || slowTaskHandle == NULL) {
        Error_Handler();
    }
}

void app_telem_task(void const *argument)
{
    task_telem(argument);
}

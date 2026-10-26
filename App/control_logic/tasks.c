// RTOS task bodies: each calls an Init once, then an Update in a loop (pattern from AMR project).
// Logic lives in ctrl_task.c, slow_task.c, telem_task.c.
#include "control_logic/tasks.h"

#include "cmsis_os.h"

#include "control_logic/ctrl_task.h"
#include "control_logic/slow_task.h"
#include "control_logic/telem_task.h"
#include "control_logic/shared_state.h"

void task_ctrl(void const *argument)
{
    (void)argument;
    ctrl_init();
    for (;;) {
        osEvent ev = osSignalWait(SIG_IMU, 5u);
        ctrl_update(ev.status == osEventSignal);
    }
}

void task_slow(void const *argument)
{
    (void)argument;
    slow_init();
    for (;;) {
        (void)osSignalWait(SIG_TOF, 10u);
        slow_update();
    }
}

void task_telem(void const *argument)
{
    (void)argument;
    telem_init();
    uint32_t last_wake = osKernelSysTick();
    for (;;) {
        osDelayUntil(&last_wake, 20u);
        telem_update();
    }
}

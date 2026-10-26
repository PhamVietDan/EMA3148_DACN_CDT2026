// task_slow implementation: RC input, sensor polling (PLAN §4.2).
#include "control_logic/slow_task.h"

#include "cmsis_os.h"

#include "control_logic/log.h"
#include "control_logic/params.h"
#include "control_logic/rc.h"
#include "control_logic/shared_state.h"
#include "board_hardware_layer/dbg_gpio.h"
#include "board_hardware_layer/rc_uart.h"
#include "board_hardware_layer/trace.h"

#define SLOW_PRINT_PERIOD_MS 1000u

static slow_stats_t st;
static uint32_t last_print_ms;
static uint32_t prev_cycles;
static uint32_t prm_seq_slow;

void slow_init(void)
{
    st = (slow_stats_t){0};
    trace_boot("task_slow is running (osPriorityHigh, ~100 Hz)");
    last_print_ms = osKernelSysTick();
    prev_cycles = 0;
    rc_start();
    trace_boot("RC SBUS is running (USART1 DMA + IDLE)");
    prm_seq_slow = g_params_seq - 1u;  // forces the first copy
}

void slow_update(void)
{
    DBG2_TOGGLE();
    if (g_params_seq != prm_seq_slow) {
        params_t p;
        prm_seq_slow = g_params_seq;
        shared_read(&p, &g_params, sizeof p);
        rc_set_deadband(p.rc_deadband);
    }
    rc_update(osKernelSysTick());
    st.cycles++;
    shared_write(&g_slow_stats, &st, sizeof st);

    uint32_t now_ms = osKernelSysTick();
    if (LOG_PERIODIC_STATS && LOG_RAW_MODE == 0 && now_ms - last_print_ms >= SLOW_PRINT_PERIOD_MS) {
        mprintf("MSG,task_slow is running: %lu Hz\r\n",
                (unsigned long)((st.cycles - prev_cycles) * 1000u / (now_ms - last_print_ms)));
        prev_cycles = st.cycles;
        last_print_ms = now_ms;
    }
}

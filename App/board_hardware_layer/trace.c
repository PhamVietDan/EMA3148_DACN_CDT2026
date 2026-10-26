// Lightweight execution trace: any context records (event, DWT time) in ~0.1 µs; task_telem prints it.
#include "board_hardware_layer/trace.h"

#include "main.h"

// PRIMASK (not taskENTER_CRITICAL) so the same code works in tasks, every ISR priority and the
// scheduler hook. Masked for a handful of instructions only.
static inline uint32_t irq_save(void)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    return pm;
}

static inline void irq_restore(uint32_t pm)
{
    __set_PRIMASK(pm);
}

static trace_rec_t capture_buf[TRACE_CAPTURE_LEN];
static volatile uint32_t capture_count;
static volatile bool capture_armed;

static const char *boot_text[TRACE_BOOT_LEN];
static volatile uint32_t boot_count;

static void *task_ctrl_tcb, *task_slow_tcb, *task_telem_tcb;

void trace_mark(trace_event_t ev, uint8_t arg)
{
    if (!capture_armed) {
        return;
    }
    uint32_t pm = irq_save();
    uint32_t n = capture_count;
    if (capture_armed && n < TRACE_CAPTURE_LEN) {
        capture_buf[n].t_cyc = DWT->CYCCNT;
        capture_buf[n].event = (uint8_t)ev;
        capture_buf[n].arg = arg;
        capture_count = n + 1u;
        if (n + 1u == TRACE_CAPTURE_LEN) {
            capture_armed = false;
        }
    }
    irq_restore(pm);
}

void trace_boot(const char *static_text)
{
    uint32_t pm = irq_save();
    if (boot_count < TRACE_BOOT_LEN) {
        boot_text[boot_count] = static_text;
        boot_count = boot_count + 1u;
    }
    irq_restore(pm);
}

void trace_set_task_handles(void *ctrl, void *slow, void *telem)
{
    task_ctrl_tcb = ctrl;
    task_slow_tcb = slow;
    task_telem_tcb = telem;
}

void trace_task_switched_in(void *tcb)
{
    if (!capture_armed) {
        return;
    }
    trace_task_id_t id = TRACE_TASK_OTHER;
    if (tcb == task_ctrl_tcb) {
        id = TRACE_TASK_CTRL;
    } else if (tcb == task_slow_tcb) {
        id = TRACE_TASK_SLOW;
    } else if (tcb == task_telem_tcb) {
        id = TRACE_TASK_TELEM;
    }
    trace_mark(TR_TASK_IN, (uint8_t)id);
}

void trace_capture_arm(void)
{
    uint32_t pm = irq_save();
    capture_count = 0;
    capture_armed = true;
    irq_restore(pm);
}

bool trace_capture_done(void)
{
    return !capture_armed && capture_count == TRACE_CAPTURE_LEN;
}

uint32_t trace_capture_count(void)
{
    return capture_count;
}

const trace_rec_t *trace_capture_buf(void)
{
    return capture_buf;
}

void trace_capture_release(void)
{
    capture_count = 0;
}

uint32_t trace_boot_count(void)
{
    return boot_count;
}

const char *trace_boot_text(uint32_t i)
{
    return (i < boot_count) ? boot_text[i] : "";
}

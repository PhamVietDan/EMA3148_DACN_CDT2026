// mprintf: formatted text to the ground-station link. Safe from any task; never from an ISR.
#include "control_logic/log.h"

#include <stdarg.h>
#include <stdio.h>

#include "board_hardware_layer/usb_cdc_port.h"
#include "main.h"

// Buffer on the caller's stack (not a shared global), so tasks preempting each other cannot mix
// their text. Integer formats only: newlib-nano's %f would pull in malloc.
void mprintf(const char *fmt, ...)
{
    if (__get_IPSR() != 0u) {
        return;  // ISR: formatting is too slow there and the TX ring is task-side only
    }
    char buf[MPRINTF_MAX_LEN];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n <= 0) {
        return;
    }
    if ((unsigned)n >= sizeof buf) {
        n = (int)sizeof buf - 1;
    }
    (void)usb_cdc_port_write(buf, (size_t)n);
}

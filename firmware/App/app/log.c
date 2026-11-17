// mprintf: formatted text to the ground-station link. Safe from any task; never from an ISR.
#include "app/log.h"

#include <stdarg.h>
#include <stdio.h>

#include "app/link.h"
#include "main.h"

// Buffer on the caller's stack (not a shared global), so tasks preempting each other cannot mix
// their text. Integer formats only: newlib-nano's %f would pull in malloc.
static void vprint_to(unsigned to, const char *fmt, va_list ap)
{
    if (__get_IPSR() != 0u) {
        return;  // ISR: formatting is too slow there and the TX ring is task-side only
    }
    char buf[MPRINTF_MAX_LEN];
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    if (n <= 0) {
        return;
    }
    if ((unsigned)n >= sizeof buf) {
        // Keep the line ending, or the next line (often a T line) is glued on and the GCS drops it.
        n = (int)sizeof buf - 1;
        buf[n - 2] = '\r';
        buf[n - 1] = '\n';
    }
    link_write_to(to, buf, (size_t)n);
}

void mprintf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vprint_to(LINK_TO_ALL, fmt, ap);
    va_end(ap);
}

void mprintf_to(unsigned to, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vprint_to(to, fmt, ap);
    va_end(ap);
}

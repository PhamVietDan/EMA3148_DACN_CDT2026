// task_telem helpers: telemetry, commands, blackbox, trace (PLAN §9).
#pragma once

#include <stdint.h>

// Call once before the task loop.
void telem_init(void);

// Call every loop iteration (50 Hz).
void telem_update(void);

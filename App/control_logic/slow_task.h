// task_slow helpers: RC input, sensor polling (~100 Hz).
#pragma once

// Call once before the task loop.
void slow_init(void);

// Call every loop iteration (woken by SIG_TOF or timeout).
void slow_update(void);

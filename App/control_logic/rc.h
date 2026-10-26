// RC input: decodes SBUS frames from bsp/rc_uart into a normalized rc_t (PLAN §6.4). Runs in task_slow.
#pragma once

#include "control_logic/shared_state.h"

void rc_start(void);
void rc_set_deadband(float fraction);  // task_slow
void rc_update(uint32_t now_ms);  // publishes g_rc

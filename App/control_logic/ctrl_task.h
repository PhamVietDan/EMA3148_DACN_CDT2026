// task_ctrl helpers: IMU init, calibration, attitude KF, arming, motor output.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "control_logic/shared_state.h"

// Call once before the task loop.
void ctrl_init(void);

// Call every loop iteration (woken by SIG_IMU or timeout).
// fresh = true when a new IMU sample is available.
void ctrl_update(bool fresh);

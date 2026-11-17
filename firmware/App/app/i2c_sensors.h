// Magnetometer (IST8310) and barometer (DPS310) on I2C1, both read by task_slow only (PLAN §3.4: one user per
// bus, no mutex). Publishes g_mag and g_baro.
#pragma once

#include <stdint.h>

#include "app/params.h"

void i2c_sensors_init(void);                                // task_slow, before its loop
void i2c_sensors_step(uint32_t now_ms, const params_t *p);  // task_slow, every cycle (~100 Hz)

// Hard-iron calibration: for duration_ms collect min/max of the uncorrected body field, then print the offsets
// as SET commands (task_telem is the only parameter writer). Rotate the drone through all orientations meanwhile.
void i2c_sensors_magcal_start(uint32_t now_ms, uint32_t duration_ms);

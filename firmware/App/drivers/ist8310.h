// iSentek IST8310 magnetometer, I2C 0x0E. Register map and set-up as PX4 (BSD) iSentek_IST8310_registers.hpp /
// IST8310.cpp: single-measurement mode triggered each cycle, 16x averaging, 16-bit output, 1320 LSB/Gauss.
// Confirmed on this board: WAI (0x00) reads 0x10 (2026-10-28).
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/i2c_dev.h"

#define IST8310_ADDR        0x0Eu
#define IST8310_WAI_VALUE   0x10u
#define IST8310_UT_PER_LSB  (100.0f / 1320.0f)  // 1 Gauss = 100 uT
#define IST8310_MIN_MEAS_MS 6u                  // after a trigger, with 16x averaging (PX4 waits 20 ms)

typedef enum { IST8310_OK = 0, IST8310_ERR_BUS = -1, IST8310_ERR_ID = -2 } ist8310_status_t;

ist8310_status_t ist8310_init(const i2c_dev_t *bus);  // reset, configure, trigger the first measurement
// Reads the result of the last trigger and triggers the next one. *fresh = false if not ready yet.
// field_ut: chip axes (x, y, z as the chip reports them), uT.
ist8310_status_t ist8310_read(const i2c_dev_t *bus, float field_ut[3], bool *fresh);

// Pure conversion of the 7-byte STAT1..DATAZH burst (host-tested). false if DRDY is clear.
bool ist8310_decode(const uint8_t burst[7], float field_ut[3]);

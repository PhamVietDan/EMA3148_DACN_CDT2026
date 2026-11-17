// Infineon DPS310 barometer, I2C 0x76 (SDO low). Register map, init sequence, coefficient layout and
// compensation as PX4 (BSD) Infineon_DPS310_Registers.hpp / DPS310.cpp. PRODUCT_ID 0x10 = DPS310 (DPS368 = 0x11,
// register-compatible); this board reads 0x10 (2026-10-28).
// Configuration: pressure 32 Hz x16 (PX4's), temperature 8 Hz x1 (PX4 uses 32 Hz x16; lowered here to keep the
// total conversion time per second well under 1 s). VERIFY the time budget against the Infineon datasheet.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/i2c_dev.h"

#define DPS310_ADDR       0x76u
#define DPS310_ID_DPS310  0x10u
#define DPS310_ID_DPS368  0x11u

typedef struct {
    int32_t c0, c1, c00, c10, c01, c11, c20, c21, c30;
} dps310_coef_t;

typedef enum {
    DPS310_OK = 0,
    DPS310_ERR_BUS = -1,
    DPS310_ERR_ID = -2,
    DPS310_ERR_NOT_READY = -3,  // SENSOR_RDY / COEF_RDY not set after reset
} dps310_status_t;

dps310_status_t dps310_init(const i2c_dev_t *bus, dps310_coef_t *coef);
// *fresh = false when no new pressure result since the last call. Pa and degC.
dps310_status_t dps310_read(const i2c_dev_t *bus, const dps310_coef_t *coef, float *pressure_pa, float *temp_c,
                            bool *fresh);

// Pure helpers (host-tested).
void dps310_parse_coef(const uint8_t raw[18], dps310_coef_t *c);
int32_t dps310_sign_extend(uint32_t v, unsigned bits);
void dps310_compensate(const dps310_coef_t *c, int32_t raw_p, int32_t raw_t, float *pressure_pa, float *temp_c);

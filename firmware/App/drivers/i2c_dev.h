// Register access used by the I2C sensor drivers, so they stay free of HAL calls (host-testable).
#pragma once

#include <stdint.h>

typedef struct {
    // 0 on success. addr is the 7-bit device address.
    int (*read)(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);
    int (*write)(uint8_t addr, uint8_t reg, uint8_t value);
    void (*delay_ms)(uint32_t ms);
} i2c_dev_t;

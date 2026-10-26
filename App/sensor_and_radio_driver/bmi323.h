// Bosch BMI323 over SPI: init/config and burst decoding. Sensor frame, SI units. RTOS-agnostic.
#pragma once

#include <stdint.h>

#define BMI323_BURST_LEN 14u  // cmd + dummy + ACC_X..Z + GYR_X..Z (6 x int16)

typedef enum {
    BMI323_OK = 0,
    BMI323_ERR_SPI = -1,
    BMI323_ERR_CHIP_ID = -2,
    BMI323_ERR_CHIP_ID_AFTER_RESET = -3,
    BMI323_ERR_CONFIG = -4,  // ERR_REG != 0 or config read-back mismatch
} bmi323_status_t;

typedef struct {
    uint16_t chip_id;
    uint16_t err_reg;
    uint16_t acc_conf;
    uint16_t gyr_conf;
} bmi323_info_t;

// delay_ms must wait at least the requested time. On return INT1 emits gyro data-ready pulses.
bmi323_status_t bmi323_init(void (*delay_ms)(uint32_t ms), bmi323_info_t *info);

const uint8_t *bmi323_burst_tx(void);
void bmi323_decode_burst(const uint8_t *rx, float gyro_rad_s[3], float acc_m_s2[3]);

/*
 * sensor_baro.c
 *
 *  Created on: Aug 13, 2026
 *      Author: ADMIN
 */
#include "sensor_baro.h"
#include "i2c.h"
#include <math.h>

float current_altitude = 0.0f;
static float alt_offset = 0.0f;
static uint8_t baro_first_read = 1;

static int32_t c0, c1, c00, c10, c01, c11, c20, c21, c30;

void Baro_Init(void) {
    uint8_t baro_cfg[2];

    baro_cfg[0] = 0x24; // P rate
    HAL_I2C_Mem_Write(&hi2c1, (0x76 << 1), 0x06, I2C_MEMADD_SIZE_8BIT, baro_cfg, 1, 100);
    baro_cfg[0] = 0xA0; // T rate
    HAL_I2C_Mem_Write(&hi2c1, (0x76 << 1), 0x07, I2C_MEMADD_SIZE_8BIT, baro_cfg, 1, 100);
    baro_cfg[0] = 0x07; // Measure continuous
    HAL_I2C_Mem_Write(&hi2c1, (0x76 << 1), 0x08, I2C_MEMADD_SIZE_8BIT, baro_cfg, 1, 100);
    HAL_Delay(50);

    // Read ROM calibration coefficients
    uint8_t coef[18];
    if (HAL_I2C_Mem_Read(&hi2c1, (0x76 << 1), 0x10, I2C_MEMADD_SIZE_8BIT, coef, 18, 100) == HAL_OK) {
        c0 = (coef[0] << 4) | (coef[1] >> 4); if(c0 & 0x0800) c0 |= 0xFFFFF000;
        c1 = ((coef[1] & 0x0F) << 8) | coef[2]; if(c1 & 0x0800) c1 |= 0xFFFFF000;
        c00 = (coef[3] << 12) | (coef[4] << 4) | (coef[5] >> 4); if(c00 & 0x080000) c00 |= 0xFFF00000;
        c10 = ((coef[5] & 0x0F) << 16) | (coef[6] << 8) | coef[7]; if(c10 & 0x080000) c10 |= 0xFFF00000;
        c01 = (coef[8] << 8) | coef[9]; if(c01 & 0x8000) c01 |= 0xFFFF0000;
        c11 = (coef[10] << 8) | coef[11]; if(c11 & 0x8000) c11 |= 0xFFFF0000;
        c20 = (coef[12] << 8) | coef[13]; if(c20 & 0x8000) c20 |= 0xFFFF0000;
        c21 = (coef[14] << 8) | coef[15]; if(c21 & 0x8000) c21 |= 0xFFFF0000;
        c30 = (coef[16] << 8) | coef[17]; if(c30 & 0x8000) c30 |= 0xFFFF0000;
    }
}

void Baro_Read_Altitude(void) {
    uint8_t baro_data[6];
    if (HAL_I2C_Mem_Read(&hi2c1, (0x76 << 1), 0x00, I2C_MEMADD_SIZE_8BIT, baro_data, 6, 10) == HAL_OK) {
        int32_t raw_p = ((int32_t)baro_data[0] << 16) | ((int32_t)baro_data[1] << 8) | baro_data[2];
        if(raw_p & 0x00800000) raw_p |= 0xFF000000;

        int32_t raw_t = ((int32_t)baro_data[3] << 16) | ((int32_t)baro_data[4] << 8) | baro_data[5];
        if(raw_t & 0x00800000) raw_t |= 0xFF000000;

        float p_scaled = (float)raw_p / 253952.0f;
        float t_scaled = (float)raw_t / 524288.0f;

        float pressure_pa = c00 + p_scaled * (c10 + p_scaled * (c20 + p_scaled * c30)) +
                            t_scaled * c01 + t_scaled * p_scaled * (c11 + p_scaled * c21);
        float abs_altitude = 44330.0f * (1.0f - powf(pressure_pa / 101325.0f, 0.190295f));

        if (baro_first_read) {
            alt_offset = abs_altitude;
            baro_first_read = 0;
        }
        current_altitude = abs_altitude - alt_offset;
    }
}


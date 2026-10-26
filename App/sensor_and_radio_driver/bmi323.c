// Bosch BMI323 over SPI: init/config and burst decoding. Sensor frame, SI units. RTOS-agnostic.
// Register values per PROJECT_PLAN §6.1 (checked there against BST-BMI323-DS000-13).
#include "sensor_and_radio_driver/bmi323.h"

#include "board_hardware_layer/spi_imu.h"

#define REG_CHIP_ID     0x00u
#define REG_ERR         0x01u
#define REG_ACC_DATA_X  0x03u
#define REG_ACC_CONF    0x20u
#define REG_GYR_CONF    0x21u
#define REG_IO_INT_CTRL 0x38u
#define REG_INT_CONF    0x39u
#define REG_INT_MAP2    0x3Bu
#define REG_CMD         0x7Eu

#define CHIP_ID_VALUE   0x43u
#define CMD_SOFT_RESET  0xDEAFu

// VERIFY: DS000-13 ACC_CONF/GYR_CONF field layout (odr[3:0], range[6:4], bw[7], avg[10:8], mode[14:12]).
#define ACC_CONF_VALUE  0x702Cu  // high-perf, bw ODR/2, ±8 g, 1.6 kHz
#define GYR_CONF_VALUE  0x704Du  // high-perf, bw ODR/2, ±2000 °/s, 3.2 kHz
#define IO_INT_CTRL_VAL 0x0005u  // INT1 active high, push-pull, output enabled
#define INT_CONF_VALUE  0x0000u  // non-latched
#define INT_MAP2_VALUE  0x0100u  // gyr_drdy_int -> INT1

#define GYR_RAD_S_PER_LSB ((3.14159265f / 180.0f) / 16.384f)  // ±2000 °/s: 16.384 LSB/(°/s)
#define ACC_M_S2_PER_LSB  (9.80665f / 4096.0f)                // ±8 g: 4096 LSB/g

static const uint8_t burst_tx[BMI323_BURST_LEN] = {0x80u | REG_ACC_DATA_X};

// SPI read: addr|0x80, one dummy byte, then 16-bit registers LSB first.
static int read_reg(uint8_t reg, uint16_t *value)
{
    uint8_t tx[4] = {(uint8_t)(0x80u | reg), 0, 0, 0};
    uint8_t rx[4];
    if (spi_imu_xfer_blocking(tx, rx, sizeof tx) != 0) {
        return -1;
    }
    *value = (uint16_t)(rx[2] | (rx[3] << 8));
    return 0;
}

// SPI write: both bytes are required, a single-byte write is ignored by the chip.
static int write_reg(uint8_t reg, uint16_t value)
{
    uint8_t tx[3] = {(uint8_t)(reg & 0x7Fu), (uint8_t)(value & 0xFFu), (uint8_t)(value >> 8)};
    uint8_t rx[3];
    return spi_imu_xfer_blocking(tx, rx, sizeof tx);
}

bmi323_status_t bmi323_init(void (*delay_ms)(uint32_t ms), bmi323_info_t *info)
{
    uint16_t v = 0;
    *info = (bmi323_info_t){0};

    delay_ms(2);
    // The chip powers up in I2C mode; a rising CSB edge (this dummy read) switches it to SPI.
    if (read_reg(REG_CHIP_ID, &v) != 0 || read_reg(REG_CHIP_ID, &v) != 0) {
        return BMI323_ERR_SPI;
    }
    info->chip_id = v;
    if ((v & 0xFFu) != CHIP_ID_VALUE) {
        return BMI323_ERR_CHIP_ID;
    }

    if (write_reg(REG_CMD, CMD_SOFT_RESET) != 0) {
        return BMI323_ERR_SPI;
    }
    delay_ms(3);
    // Soft reset returns the interface to I2C mode: dummy read again.
    if (read_reg(REG_CHIP_ID, &v) != 0 || read_reg(REG_CHIP_ID, &v) != 0) {
        return BMI323_ERR_SPI;
    }
    info->chip_id = v;
    if ((v & 0xFFu) != CHIP_ID_VALUE) {
        return BMI323_ERR_CHIP_ID_AFTER_RESET;
    }

    if (write_reg(REG_IO_INT_CTRL, IO_INT_CTRL_VAL) != 0 ||
        write_reg(REG_INT_CONF, INT_CONF_VALUE) != 0 ||
        write_reg(REG_INT_MAP2, INT_MAP2_VALUE) != 0 ||
        write_reg(REG_ACC_CONF, ACC_CONF_VALUE) != 0 ||
        write_reg(REG_GYR_CONF, GYR_CONF_VALUE) != 0) {
        return BMI323_ERR_SPI;
    }

    delay_ms(50);  // gyro start-up ~30 ms per datasheet
    if (read_reg(REG_ERR, &info->err_reg) != 0 ||
        read_reg(REG_ACC_CONF, &info->acc_conf) != 0 ||
        read_reg(REG_GYR_CONF, &info->gyr_conf) != 0) {
        return BMI323_ERR_SPI;
    }
    if (info->err_reg != 0u || info->acc_conf != ACC_CONF_VALUE || info->gyr_conf != GYR_CONF_VALUE) {
        return BMI323_ERR_CONFIG;
    }
    return BMI323_OK;
}

const uint8_t *bmi323_burst_tx(void)
{
    return burst_tx;
}

void bmi323_decode_burst(const uint8_t *rx, float gyro_rad_s[3], float acc_m_s2[3])
{
    const uint8_t *d = &rx[2];  // skip command echo and dummy byte
    for (int i = 0; i < 3; i++) {
        int16_t a = (int16_t)(d[2 * i] | (d[2 * i + 1] << 8));
        int16_t g = (int16_t)(d[6 + 2 * i] | (d[6 + 2 * i + 1] << 8));
        acc_m_s2[i] = (float)a * ACC_M_S2_PER_LSB;
        gyro_rad_s[i] = (float)g * GYR_RAD_S_PER_LSB;
    }
}

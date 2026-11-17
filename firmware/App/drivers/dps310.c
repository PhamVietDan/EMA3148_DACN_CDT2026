// Infineon DPS310 barometer (see dps310.h).
#include "drivers/dps310.h"

#define REG_PSR_B2    0x00u  // PSR_B2..B0, TMP_B2..B0 (0x00..0x05)
#define REG_PRS_CFG   0x06u
#define REG_TMP_CFG   0x07u
#define REG_MEAS_CFG  0x08u
#define REG_CFG_REG   0x09u
#define REG_RESET     0x0Cu
#define REG_ID        0x0Du
#define REG_COEF      0x10u  // 18 bytes, 0x10..0x21
#define REG_COEF_SRCE 0x28u

#define MEAS_COEF_RDY   0x80u
#define MEAS_SENSOR_RDY 0x40u
#define MEAS_PRS_RDY    0x10u
#define MEAS_CONT_PT    0x07u  // continuous pressure and temperature
#define RESET_SOFT      0x09u
#define COEF_SRCE_TMP   0x80u  // temperature sensor the coefficients were made with -> TMP_CFG bit 7
#define CFG_P_SHIFT     0x04u  // required when pressure oversampling > 8x

#define PRS_CFG_VALUE   0x54u  // rate [6:4] 101 = 32 Hz, oversampling [3:0] 0100 = 16x
#define TMP_CFG_VALUE   0x30u  // rate [6:4] 011 = 8 Hz, oversampling 0000 = 1x (+ source bit from COEF_SRCE)
#define KP_16X          253952.0f
#define KT_1X           524288.0f

int32_t dps310_sign_extend(uint32_t v, unsigned bits)
{
    uint32_t sign = 1u << (bits - 1u);
    v &= (sign << 1) - 1u;
    return (int32_t)(v ^ sign) - (int32_t)sign;
}

void dps310_parse_coef(const uint8_t r[18], dps310_coef_t *c)
{
    c->c0 = dps310_sign_extend(((uint32_t)r[0] << 4) | (r[1] >> 4), 12);
    c->c1 = dps310_sign_extend(((uint32_t)(r[1] & 0x0Fu) << 8) | r[2], 12);
    c->c00 = dps310_sign_extend(((uint32_t)r[3] << 12) | ((uint32_t)r[4] << 4) | (r[5] >> 4), 20);
    c->c10 = dps310_sign_extend(((uint32_t)(r[5] & 0x0Fu) << 16) | ((uint32_t)r[6] << 8) | r[7], 20);
    c->c01 = dps310_sign_extend(((uint32_t)r[8] << 8) | r[9], 16);
    c->c11 = dps310_sign_extend(((uint32_t)r[10] << 8) | r[11], 16);
    c->c20 = dps310_sign_extend(((uint32_t)r[12] << 8) | r[13], 16);
    c->c21 = dps310_sign_extend(((uint32_t)r[14] << 8) | r[15], 16);
    c->c30 = dps310_sign_extend(((uint32_t)r[16] << 8) | r[17], 16);
}

void dps310_compensate(const dps310_coef_t *c, int32_t raw_p, int32_t raw_t, float *pressure_pa, float *temp_c)
{
    float p = (float)raw_p / KP_16X;
    float t = (float)raw_t / KT_1X;
    *pressure_pa = (float)c->c00 + p * ((float)c->c10 + p * ((float)c->c20 + p * (float)c->c30)) +
                   t * (float)c->c01 + t * p * ((float)c->c11 + p * (float)c->c21);
    *temp_c = (float)c->c0 * 0.5f + (float)c->c1 * t;
}

dps310_status_t dps310_init(const i2c_dev_t *bus, dps310_coef_t *coef)
{
    uint8_t v;
    if (bus->read(DPS310_ADDR, REG_ID, &v, 1) != 0) {
        return DPS310_ERR_BUS;
    }
    if (v != DPS310_ID_DPS310 && v != DPS310_ID_DPS368) {
        return DPS310_ERR_ID;
    }
    if (bus->write(DPS310_ADDR, REG_RESET, RESET_SOFT) != 0) {
        return DPS310_ERR_BUS;
    }
    bus->delay_ms(40);
    if (bus->read(DPS310_ADDR, REG_MEAS_CFG, &v, 1) != 0) {
        return DPS310_ERR_BUS;
    }
    if ((v & (MEAS_COEF_RDY | MEAS_SENSOR_RDY)) != (MEAS_COEF_RDY | MEAS_SENSOR_RDY)) {
        return DPS310_ERR_NOT_READY;
    }
    uint8_t raw[18];
    uint8_t src;
    if (bus->read(DPS310_ADDR, REG_COEF, raw, sizeof raw) != 0 ||
        bus->read(DPS310_ADDR, REG_COEF_SRCE, &src, 1) != 0) {
        return DPS310_ERR_BUS;
    }
    dps310_parse_coef(raw, coef);
    if (bus->write(DPS310_ADDR, REG_PRS_CFG, PRS_CFG_VALUE) != 0 ||
        bus->write(DPS310_ADDR, REG_TMP_CFG, (uint8_t)(TMP_CFG_VALUE | (src & COEF_SRCE_TMP))) != 0 ||
        bus->write(DPS310_ADDR, REG_CFG_REG, CFG_P_SHIFT) != 0 ||
        bus->write(DPS310_ADDR, REG_MEAS_CFG, MEAS_CONT_PT) != 0) {
        return DPS310_ERR_BUS;
    }
    return DPS310_OK;
}

dps310_status_t dps310_read(const i2c_dev_t *bus, const dps310_coef_t *coef, float *pressure_pa, float *temp_c,
                            bool *fresh)
{
    uint8_t v;
    *fresh = false;
    if (bus->read(DPS310_ADDR, REG_MEAS_CFG, &v, 1) != 0) {
        return DPS310_ERR_BUS;
    }
    if ((v & MEAS_PRS_RDY) == 0u) {
        return DPS310_OK;
    }
    uint8_t b[6];
    if (bus->read(DPS310_ADDR, REG_PSR_B2, b, sizeof b) != 0) {
        return DPS310_ERR_BUS;
    }
    int32_t raw_p = dps310_sign_extend(((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2], 24);
    int32_t raw_t = dps310_sign_extend(((uint32_t)b[3] << 16) | ((uint32_t)b[4] << 8) | b[5], 24);
    dps310_compensate(coef, raw_p, raw_t, pressure_pa, temp_c);
    *fresh = true;
    return DPS310_OK;
}

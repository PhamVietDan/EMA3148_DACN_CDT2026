// iSentek IST8310 magnetometer (see ist8310.h).
#include "drivers/ist8310.h"

#define REG_WAI     0x00u
#define REG_STAT1   0x02u  // followed by DATAXL..DATAZH (0x03..0x08)
#define REG_CNTL1   0x0Au
#define REG_CNTL2   0x0Bu
#define REG_CNTL3   0x0Du
#define REG_AVGCNTL 0x41u
#define REG_PDCNTL  0x42u

#define STAT1_DRDY        0x01u
#define CNTL1_SINGLE      0x01u
#define CNTL2_SRST        0x01u
#define CNTL3_XYZ_16BIT   0x70u  // bits 6:4
#define AVGCNTL_16X       0x24u  // y avg [5:3] = 100, x/z avg [2:0] = 100: 16 times
#define AVGCNTL_AVG_MASK  0x3Fu
#define PDCNTL_NORMAL     0xC0u  // pulse duration [7:6] = 11 ("please use this setting")

static int rmw(const i2c_dev_t *bus, uint8_t reg, uint8_t clear, uint8_t set)
{
    uint8_t v;
    if (bus->read(IST8310_ADDR, reg, &v, 1) != 0) {
        return -1;
    }
    return bus->write(IST8310_ADDR, reg, (uint8_t)((v & (uint8_t)~clear) | set));
}

ist8310_status_t ist8310_init(const i2c_dev_t *bus)
{
    uint8_t id;
    if (bus->read(IST8310_ADDR, REG_WAI, &id, 1) != 0) {
        return IST8310_ERR_BUS;
    }
    if (id != IST8310_WAI_VALUE) {
        return IST8310_ERR_ID;
    }
    if (bus->write(IST8310_ADDR, REG_CNTL2, CNTL2_SRST) != 0) {
        return IST8310_ERR_BUS;
    }
    bus->delay_ms(50);
    if (bus->write(IST8310_ADDR, REG_CNTL3, CNTL3_XYZ_16BIT) != 0 ||
        rmw(bus, REG_AVGCNTL, AVGCNTL_AVG_MASK, AVGCNTL_16X) != 0 ||
        rmw(bus, REG_PDCNTL, 0u, PDCNTL_NORMAL) != 0 ||
        bus->write(IST8310_ADDR, REG_CNTL1, CNTL1_SINGLE) != 0) {
        return IST8310_ERR_BUS;
    }
    return IST8310_OK;
}

bool ist8310_decode(const uint8_t b[7], float field_ut[3])
{
    if ((b[0] & STAT1_DRDY) == 0u) {
        return false;
    }
    for (int i = 0; i < 3; i++) {
        int16_t raw = (int16_t)((uint16_t)b[1 + 2 * i] | ((uint16_t)b[2 + 2 * i] << 8));
        field_ut[i] = (float)raw * IST8310_UT_PER_LSB;
    }
    return true;
}

ist8310_status_t ist8310_read(const i2c_dev_t *bus, float field_ut[3], bool *fresh)
{
    uint8_t b[7];
    *fresh = false;
    if (bus->read(IST8310_ADDR, REG_STAT1, b, sizeof b) != 0) {
        return IST8310_ERR_BUS;
    }
    *fresh = ist8310_decode(b, field_ut);
    if (bus->write(IST8310_ADDR, REG_CNTL1, CNTL1_SINGLE) != 0) {
        return IST8310_ERR_BUS;
    }
    return IST8310_OK;
}

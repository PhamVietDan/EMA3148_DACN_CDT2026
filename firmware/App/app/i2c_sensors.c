// Magnetometer (IST8310) and barometer (DPS310) on I2C1 (see i2c_sensors.h).
#include "app/i2c_sensors.h"

#include <math.h>

#include "app/config.h"
#include "app/log.h"
#include "app/shared_state.h"
#include "drivers/dps310.h"
#include "drivers/ist8310.h"
#include "i2c.h"
#include "lib/filter.h"
#include "lib/mathx.h"

#define I2C_TIMEOUT_MS     5u
#define MAG_PERIOD_MS      20u    // ~50 Hz; one measurement needs >= 6 ms with 16x averaging
#define SENSOR_STALE_MS    200u
#define BARO_RATE_HZ       32.0f
#define BARO_REF_SAMPLES   64u    // ~2 s at 32 Hz
#define BARO_ALT_LPF_HZ    2.0f
#define REINIT_PERIOD_MS   2000u  // retry a failed init

static int bus_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr << 1), reg, I2C_MEMADD_SIZE_8BIT, buf, len, I2C_TIMEOUT_MS) ==
                   HAL_OK
               ? 0
               : -1;
}

static int bus_write(uint8_t addr, uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(addr << 1), reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2C_TIMEOUT_MS) ==
                   HAL_OK
               ? 0
               : -1;
}

static void bus_delay(uint32_t ms)
{
    osDelay(ms + 1u);
}

static const i2c_dev_t bus = {bus_read, bus_write, bus_delay};
static const int mag_src[3] = CFG_MAG_AXIS_SRC;
static const float mag_sign[3] = CFG_MAG_AXIS_SIGN;

static mag_t mag;
static uint32_t mag_last_ms, mag_init_ms;
static bool mag_ready;

static baro_t baro;
static dps310_coef_t baro_coef;
static uint32_t baro_init_ms;
static bool baro_ready;
static uint32_t baro_ref_n;
static double baro_ref_sum;  // ~2 s of samples; double avoids float accumulation error (once at start)
static float baro_p0;
static pt1_t baro_alt_lpf;

static bool cal_active;
static uint32_t cal_end_ms;
static float cal_min[3], cal_max[3];

static void mag_try_init(uint32_t now_ms)
{
    mag_init_ms = now_ms;
    mag.init_status = (int8_t)ist8310_init(&bus);
    mag_ready = mag.init_status == IST8310_OK;
    mag_last_ms = now_ms;
}

static void baro_try_init(uint32_t now_ms)
{
    baro_init_ms = now_ms;
    baro.init_status = (int8_t)dps310_init(&bus, &baro_coef);
    baro_ready = baro.init_status == DPS310_OK;
    baro_ref_n = 0;
    baro_ref_sum = 0.0;
    pt1_init(&baro_alt_lpf, BARO_ALT_LPF_HZ, BARO_RATE_HZ);
}

void i2c_sensors_init(void)
{
    uint32_t now = osKernelSysTick();
    mag_try_init(now);
    baro_try_init(now);
    mprintf("MSG,I2C sensors: IST8310 init %d, DPS310 init %d\r\n", mag.init_status, baro.init_status);
}

void i2c_sensors_magcal_start(uint32_t now_ms, uint32_t duration_ms)
{
    for (int i = 0; i < 3; i++) {
        cal_min[i] = 1e9f;
        cal_max[i] = -1e9f;
    }
    cal_end_ms = now_ms + duration_ms;
    cal_active = true;
}

static void magcal_finish(void)
{
    cal_active = false;
    float off[3], half[3];
    for (int i = 0; i < 3; i++) {
        off[i] = 0.5f * (cal_max[i] + cal_min[i]);
        half[i] = 0.5f * (cal_max[i] - cal_min[i]);
    }
    // Integer output (mprintf has no %f): offsets in 0.1 uT.
    mprintf("MSG,MAGCAL done: half-ranges x %ld y %ld z %ld uT (similar values = all orientations covered)\r\n",
            (long)half[0], (long)half[1], (long)half[2]);
    static const char axis[3] = {'x', 'y', 'z'};
    for (int i = 0; i < 3; i++) {
        long t = (long)(off[i] * 10.0f + (off[i] >= 0.0f ? 0.5f : -0.5f));
        unsigned long a = (unsigned long)(t < 0 ? -t : t);
        mprintf("MSG,MAGCAL SET mag_off_%c %s%lu.%lu\r\n", axis[i], t < 0 ? "-" : "", a / 10u, a % 10u);
    }
    mprintf("MSG,MAGCAL send the 3 SET lines above, then SAVE\r\n");
}

// A retried init blocks task_slow (RC input) for up to ~0.1 s, so only while disarmed.
static bool may_reinit(void)
{
    arm_status_t arm;
    shared_read(&arm, &g_arm, sizeof arm);
    return !arm.armed;
}

static void mag_step(uint32_t now_ms, const params_t *p)
{
    if (!mag_ready) {
        if (now_ms - mag_init_ms >= REINIT_PERIOD_MS && may_reinit()) {
            mag_try_init(now_ms);
        }
        return;
    }
    if (now_ms - mag_last_ms < MAG_PERIOD_MS) {
        return;
    }
    mag_last_ms = now_ms;
    float chip[3];
    bool fresh;
    if (ist8310_read(&bus, chip, &fresh) != IST8310_OK) {
        mag.bus_errors++;
        return;
    }
    if (!fresh) {
        return;
    }
    const float off[3] = {p->mag_off_x, p->mag_off_y, p->mag_off_z};
    for (int i = 0; i < 3; i++) {
        float body = mag_sign[i] * chip[mag_src[i]];
        if (cal_active) {
            cal_min[i] = fminf(cal_min[i], body);
            cal_max[i] = fmaxf(cal_max[i], body);
        }
        mag.field[i] = body - off[i];
    }
    mag.t_ms = now_ms;
}

static void baro_step(uint32_t now_ms)
{
    if (!baro_ready) {
        if (now_ms - baro_init_ms >= REINIT_PERIOD_MS && may_reinit()) {
            baro_try_init(now_ms);
        }
        return;
    }
    float pa, tc;
    bool fresh;
    if (dps310_read(&bus, &baro_coef, &pa, &tc, &fresh) != DPS310_OK) {
        baro.bus_errors++;
        return;
    }
    if (!fresh) {
        return;
    }
    baro.t_ms = now_ms;
    baro.pressure_pa = pa;
    baro.temp_c = tc;
    if (baro_ref_n < BARO_REF_SAMPLES) {
        baro_ref_sum += (double)pa;
        if (++baro_ref_n == BARO_REF_SAMPLES) {
            baro_p0 = (float)(baro_ref_sum / (double)BARO_REF_SAMPLES);
        }
        return;
    }
    // PLAN §6.2b: alt = 44330 * (1 - (p/p0)^(1/5.255)).
    float alt = 44330.0f * (1.0f - powf(pa / baro_p0, 1.0f / 5.255f));
    baro.alt_rel_m = pt1_apply(&baro_alt_lpf, alt);
}

void i2c_sensors_step(uint32_t now_ms, const params_t *p)
{
    mag_step(now_ms, p);
    baro_step(now_ms);
    if (cal_active && (int32_t)(now_ms - cal_end_ms) >= 0) {
        magcal_finish();
    }
    mag.valid = mag_ready && mag.t_ms != 0u && now_ms - mag.t_ms < SENSOR_STALE_MS;
    baro.valid = baro_ready && baro_ref_n >= BARO_REF_SAMPLES && now_ms - baro.t_ms < SENSOR_STALE_MS;
    shared_write(&g_mag, &mag, sizeof mag);
    shared_write(&g_baro, &baro, sizeof baro);
}

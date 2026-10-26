// Tunable parameter table (PLAN §8): name, default, min, max; flash image with CRC. Pure C (host-tested).
// Angles in degrees and rates in deg/s here, because these values are typed by a person on the GCS.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    // Rate loop (normalized torque per rad/s)
    float rate_p_rp, rate_i_rp, rate_d_rp, rate_p_y, rate_i_y, rate_ilim;
    // Angle loop and stick scaling
    float ang_p_rp;          // 1/s
    float max_angle;         // deg
    float max_rate_rp;       // deg/s
    float max_yaw_rate;      // deg/s
    // Filters
    float gyro_lpf_hz, acc_lpf_hz, dterm_lpf_hz;
    // Attitude Kalman
    float kf_sg;             // deg/s
    float kf_sacc;           // deg
    // Motors / RC
    float motor_idle_us;
    float rc_deadband;       // fraction of half stick travel
    // Loss-of-control disarm
    float lc_angle;          // deg, |attitude - setpoint|
    float lc_time_ms;
} params_t;

enum { PARAM_DISARMED_ONLY = 1u << 0 };  // changing it re-initialises a filter: refused while armed

typedef struct {
    const char *name;
    uint16_t offset;  // into params_t
    float def, min, max;
    uint8_t flags;
} param_desc_t;

typedef enum { PARAM_OK, PARAM_ERR_NAME, PARAM_ERR_RANGE } param_result_t;

typedef enum {
    PARAM_LOAD_OK,
    PARAM_LOAD_EMPTY,     // erased flash / never saved
    PARAM_LOAD_CORRUPT,   // bad magic, size or CRC
} param_load_t;

size_t params_count(void);
const param_desc_t *params_desc(size_t i);
int params_find(const char *name);  // index or -1
void params_defaults(params_t *p);
float params_get(const params_t *p, size_t i);
param_result_t params_set(params_t *p, size_t i, float value);

// Flash image: header {magic, version, count, crc32} + count x {FNV-1a name hash, value}.
// Loading matches by name hash, so adding or reordering parameters keeps the saved ones;
// unknown or out-of-range entries are skipped and keep their defaults.
#define PARAMS_IMAGE_MAX 512u
size_t params_image_build(const params_t *p, uint8_t *buf, size_t size);
param_load_t params_image_load(params_t *p, const uint8_t *buf, size_t size, unsigned *n_loaded);

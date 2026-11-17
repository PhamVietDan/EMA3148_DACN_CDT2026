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
    // Gyro notch tracking the motors: center = lo + (hi - lo) * mean motor command [0,1]; lo = 0: off
    float notch_lo_hz, notch_hi_hz, notch_q;
    // Magnetometer hard-iron offsets, body FRD, uT (from MAGCAL)
    float mag_off_x, mag_off_y, mag_off_z;
    // Altitude Kalman (PLAN §7.4): process noise (m/s^2), ToF noise (m), ToF step threshold (m)
    float alt_sigma_a, alt_sigma_tof, alt_step_th;
    // Angle-mode level trim, deg, added to the stick setpoint (drifts right: lower trim_roll; back: lower trim_pitch)
    float trim_roll, trim_pitch;
    // Alt Hold (PLAN §7.5): z P (1/s), vz P (thr per m/s), vz I, vz limit (m/s), output = engage thr ± range
    float alt_p, vz_p, vz_i, vz_max, alt_thr_rng;
    // Heading hold with the yaw stick centred (app/yaw_hold.h): 1/s; 0 = rate-only yaw as before
    float yaw_hold_p;
    // Alt Hold base throttle (learnt in flight, printed as a SET line; 0 = engage on the stick throttle)
    float hover_thr;
    // Angle-mode throttle stick curve: fixed point thr_mid, flatter around it with thr_expo (0 = linear)
    float thr_mid, thr_expo;
    // Optical flow (app/flow_est.h): counts per rad/s (signed), velocity scale, min quality, noise (rad/s)
    float flow_k, flow_scale, flow_qmin, flow_sigma;
    // Position hold (app/pos_hold.h): pos P (1/s, 0 = off: Alt Hold only), vel P (deg per m/s), vel I (deg per
    // m), full-stick speed (m/s), tilt limit (deg)
    float pos_p, vel_p, vel_i, vel_max, pos_ang_max;
    // Automatic take-off height when Alt Hold is engaged on the floor (m; 0 = off: engagement refused below 0.3 m)
    float takeoff_alt;
    // M10b: baro noise in the altitude Kalman (m), used when the ToF is lost above 0.5 m
    float alt_sigma_baro;
    // Optical flow mount: deg the sensor is turned from the 2026-11-10 reference (0/90/180/270, app/flow_est.h)
    float flow_rot;
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

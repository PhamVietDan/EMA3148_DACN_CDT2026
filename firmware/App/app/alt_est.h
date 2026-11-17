// Altitude estimator (PLAN Â§7.4, Â§7.4b part 1): 2-state Kalman on accelerometer + ToF, with ToF step detection
// that moves the terrain offset h_terrain instead of the altitude (z = height above the take-off floor).
// Pure C (host-tested); task_slow runs it every ~10 ms. M9: observation only, nothing is controlled from it.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "lib/alt_kf.h"

typedef struct {
    float sigma_a;    // m/s^2, process noise
    float sigma_tof;  // m, measurement noise
    float step_th;    // m, ToF change not explained by vz that starts a step check
    float sigma_baro;    // m, baro measurement noise (M10b fallback; PLAN Â§7.4b: 0.3)
    uint32_t baro_max_ms;  // longest ToF gap bridged by the baro (PLAN: 3 s; longer for a failsafe landing)
} alt_cfg_t;

typedef struct {
    uint32_t now_ms;
    float dt;           // s since the previous call
    float a_up;         // m/s^2, gravity and rest bias removed, mean over the last ~10 ms
    bool tof_new;       // a rangefinder frame arrived since the previous call
    bool tof_in_range;  // that frame was in range
    float tof_dist;     // m, along the sensor axis
    float roll, pitch;  // rad
    bool armed;         // rising edge: z re-referenced to the floor under the drone (h_terrain = 0)
    bool baro_new;      // a new baro altitude since the previous call
    float baro_alt;     // m, relative to the boot pressure (app/i2c_sensors.c)
} alt_in_t;

typedef enum { ALT_SRC_TOF = 0, ALT_SRC_BARO = 1, ALT_SRC_PREDICT = 2 } alt_src_t;
typedef enum { ALT_NORMAL, ALT_CONFIRM } alt_step_state_t;
typedef enum {
    ALT_EV_NONE,
    ALT_EV_INIT,         // first usable ToF, or re-init after a long loss
    ALT_EV_STEP,         // step confirmed: h_terrain moved (event_val = new h_terrain)
    ALT_EV_STEP_NOISE,   // step not confirmed within the window: ignored
    ALT_EV_RESYNC,       // ToF rejected by the outlier gate too long: z and vz restarted from ToF, h_terrain kept
                         // (event_val = z error removed). The filter had drifted (accel), not the floor.
    ALT_EV_GROUND,       // disarmed and back at the rest level: h_terrain cleared (event_val = old h_terrain)
    ALT_EV_ARM,          // arming: z re-referenced to the floor under the drone
} alt_event_t;

typedef struct {
    float z, vz, h_terrain;  // m, m/s, m
    float tof_corr;          // m, tilt-corrected ToF of the last usable frame
    uint8_t src;             // alt_src_t
    uint8_t step_state;      // alt_step_state_t
    bool valid;              // initialised and a usable ToF frame within ALT_TOF_LOST_MS, or bridged by the baro
    float z_floor;           // m, z resting on the floor (ToF at boot / arming): the landing detector's reference
    float baro_ref;          // m, baro altitude - z (learnt); meaningful when baro_ref_ok
    bool baro_ref_ok;
    uint8_t event;           // alt_event_t of this call
    float event_val;
} alt_out_t;

#define ALT_TOF_LOST_MS     500u   // PLAN Â§7.4: no usable ToF this long -> not valid
#define ALT_REINIT_MS       2000u  // lost longer: restart from the ToF reading with h_terrain = 0 (no baro yet)
#define ALT_TILT_MAX_RAD    0.5235988f  // 30 deg (PLAN Â§6.3)
#define ALT_CONFIRM_N       3u     // samples within ALT_CONFIRM_BAND of the new level
#define ALT_CONFIRM_BAND    0.03f
#define ALT_CONFIRM_MS      100u
#define ALT_TERRAIN_SNAP    0.03f  // |h_terrain| below this after a step -> 0
#define ALT_GATE_SQ         9.0f   // (innovation)^2 / S
#define ALT_RESYNC_N        10u    // consecutive gate rejections (~0.1 s) before restarting z from ToF
// Short ToF spikes (2026-11-16 logs 12:24-12:31: +9..+25 cm for < 60 ms, a few per minute in Alt Hold; the 3 sigma
// gate is ~16 cm so most passed and pushed z up 1-2 cm each). A frame further than ALT_SPIKE_M from the predicted
// z is held back, up to ALT_SPIKE_N frames in a row (~80 ms at 100 Hz); still off after that: a real change, the
// frames go to the Kalman (gate, resync) as before. In-flight ToF - z is ~2 cm sigma, so 6 cm is rare otherwise.
// Step check: per-frame jump = ToF change - vz * dt. Crossing an edge the ToF moves over ~0.1-0.2 s (beam
// footprint), so a single jump missed it (2026-11-16 16:02 hand test: 0.95 -> 0.67 m, ~3 cm per frame, ended as
// a z resync). Consecutive fast jumps of one sign (> ALT_STEP_RATE, i.e. a speed vz does not explain) are summed;
// the sum beyond step_th starts the check. Slow differences (accelerometer drift, vz off by a few dm/s) never
// count, nor are they compared with z, which drifts with the accelerometer while the ToF is held back.
#define ALT_STEP_RATE       1.0f   // m/s: 1 cm per 10 ms frame
#define ALT_STEP_RECENT     3u     // frames: a run goes on while these together still move fast...
#define ALT_STEP_AVG        0.5f   // ...at this x ALT_STEP_RATE (the same way as the run)
#define ALT_SPIKE_M         0.06f
#define ALT_SPIKE_N         8u
#define ALT_P0              0.01f  // (0.1 m)^2 and (0.1 m/s)^2
#define ALT_STEP_TILT_RAD   0.1745329f  // 10 deg: above this a ToF jump is a tilted beam, never a terrain step
#define ALT_REST_BAND       0.03f  // m, ToF within this of the rest level (ToF at init / arming)...
#define ALT_REST_MS         500u   // ...this long while disarmed: on the floor again, h_terrain = 0
// M10b (PLAN Â§7.4b (2)(3)(4)): baro_ref = baro - z, set when the ToF first initialises z (drone still at boot, as
// the baro reference p0), then learnt slowly in flight while the ToF is used, z > ALT_BARO_MIN_Z and no step is
// pending. ToF lost with z > ALT_BARO_MIN_Z: the Kalman takes baro - baro_ref instead, for up to baro_max_ms.
// Never below ALT_BARO_MIN_Z: prop wash near the floor makes the baro read up to metres off (Betaflight notes).
#define ALT_BARO_MIN_Z      0.5f
#define ALT_BARO_REF_TAU_S  5.0f
#define ALT_BARO_GATE_SQ    16.0f  // 4 sigma: the baro is coarse, a tight gate would drop it when needed
#define ALT_BARO_SLOT_MS    50u    // ToF missing for 2 slots (100 ms): the baro starts carrying z
#define ALT_BARO_STALE_MS   200u   // no accepted baro sample this long: no longer valid on the baro
#define ALT_BARO_MAX_MS     3000u  // PLAN Â§7.4b (3)
#define ALT_BARO_MAX_LAND_MS 20000u  // during a failsafe / commanded landing

typedef struct {
    alt_kf_t kf;
    bool init;
    float h_terrain;
    float tof_prev;
    uint32_t tof_prev_ms;
    uint8_t state;
    float new_level;
    uint32_t confirm_n, confirm_start_ms;
    uint32_t rejects;
    uint32_t spike_n;        // consecutive frames held back as a spike (ALT_SPIKE_*)
    float jump_sum;          // m, run of fast ToF change not explained by vz (ALT_STEP_RATE)
    float jump_t;            // s, run length; 0: no run
    float recent_jump[ALT_STEP_RECENT], recent_tof[ALT_STEP_RECENT];  // last frames' jumps / ToF changes
    uint8_t src;
    bool was_armed;
    float rest_tof;          // tilt-corrected ToF on the floor (set at init and at arming)
    bool at_rest;
    uint32_t rest_start_ms;
    float baro_ref;
    bool baro_ref_ok;
    uint32_t baro_used_ms;   // last time the baro carried z (no ToF)
    bool baro_used;
    float baro_last;         // m, last baro altitude seen
    uint32_t baro_ms;        // when it came
    bool baro_seen;
} alt_est_t;

void alt_est_init(alt_est_t *e);
void alt_est_step(alt_est_t *e, const alt_cfg_t *c, const alt_in_t *in, alt_out_t *out);

// Altitude hold (PLAN §7.5 "Độ cao", M10): z P -> vz PI -> throttle, tilt-compensated, around the throttle at
// engagement. Ideas from Betaflight altitudeControl / ArduPilot AC_PosControl, not copied. Pure C (host-tested);
// task_ctrl runs it every control cycle, z/vz come from the estimator (task_slow, ~100 Hz).
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float alt_p;      // 1/s: z error -> vz setpoint
    float vz_p;       // throttle per m/s
    float vz_i;       // throttle per m (per m/s per s)
    float vz_max;     // m/s, climb/descent limit (also the full-stick rate)
    float thr_range;  // output limited to the engagement throttle ± this
    float hover_thr;  // level hover throttle (parameter, saved); 0 = unknown: engage on the stick throttle
    float takeoff_alt;  // m: automatic take-off height when engaged on the floor; 0 = no take-off (refused)
} alt_hold_cfg_t;

typedef struct {
    float dt;          // s
    bool request;      // pilot wants Alt Hold (CH6 middle; in RC failsafe: keep holding if already engaged)
    bool flying;       // armed and motors running
    bool alt_valid;
    float z, vz;       // m, m/s
    float stick_thr;   // [0,1]; within ±ALT_HOLD_DEADBAND of the engagement stick holds, beyond climbs/descends
    bool land;         // land now (RC failsafe stage 2 / LAND command): engage, descend at ALT_HOLD_LAND_VZ
    float tilt_cos;    // cos(roll) * cos(pitch)
    float z_floor;     // m, z resting on the floor (app/alt_est.h)
} alt_hold_in_t;

typedef enum {
    AH_EV_NONE,
    AH_EV_ENGAGED,         // event_val = z_sp
    AH_EV_EXIT_SWITCH,
    AH_EV_EXIT_INVALID,    // altitude estimate lost: back to the stick (switch must leave Alt Hold to retry)
    AH_EV_EXIT_ERROR,      // |z - z_sp| beyond ALT_HOLD_MAX_ERR: something is wrong, back to the stick
    AH_EV_REFUSED_INVALID,
    AH_EV_REFUSED_LOW,     // below ALT_HOLD_MIN_Z
    AH_EV_LANDED,          // descended onto the floor: throttle 0 (motors to idle) until the stick is lowered
    AH_EV_TAKEOFF,         // automatic take-off started (event_val = target height)
    AH_EV_LAND,            // failsafe / commanded landing started (event_val = z)
} alt_hold_event_t;

#define ALT_HOLD_MIN_Z      0.30f  // m: no engagement in ground effect
#define ALT_HOLD_MIN_ZSP    (-0.20f)  // m: below the floor, so a held descent keeps pressing down until landed
#define ALT_HOLD_SLOW_Z     0.50f  // m: below this, descent limited to ALT_HOLD_LAND_VZ
#define ALT_HOLD_LAND_VZ    0.30f  // m/s (PLAN §7.7 failsafe landing speed)
// Landing detector (idea from ArduPilot land_detector / land_complete_maybe, not copied; rewritten 2026-11-13):
// maybe landed = asked down, z within ALT_HOLD_FLOOR_BAND of the floor reading, z_sp more than ALT_HOLD_PRESS
// below z, low-passed |vz| under ALT_HOLD_STILL_VZ (raw vz bounces +-0.2 m/s on the legs). Then the throttle ramps
// down at ALT_HOLD_RAMP_PER_S, horizontal control stops, and after ALT_HOLD_LAND_S it is landed.
#define ALT_HOLD_FLOOR_BAND   0.05f
#define ALT_HOLD_PRESS        0.10f
#define ALT_HOLD_VZ_TAU_S     0.20f   // vz low-pass for the "stopped" test
#define ALT_HOLD_STILL_VZ     0.15f
#define ALT_HOLD_RAMP_PER_S   0.5f
#define ALT_HOLD_LAND_S       1.0f
// Landing with the estimate gone below ALT_HOLD_BLIND_Z (no ToF; no baro down there): descend on
// ALT_HOLD_BLIND_THR x base throttle for ALT_HOLD_BLIND_S, then landed (user's timeout idea).
#define ALT_HOLD_BLIND_Z      0.50f
#define ALT_HOLD_BLIND_THR    0.90f
#define ALT_HOLD_BLIND_S      3.0f
#define ALT_HOLD_STICK_LOW    0.05f  // after landing, the stick must come below this before it drives the motors
// Hover throttle learning (idea of ArduPilot MOT_HOVER_LEARN, done as in the s450drone code: a slow average of
// the level-equivalent throttle while holding still). Used as the base at the next engagement.
#define ALT_HOLD_LEARN_TAU_S  3.0f
#define ALT_HOLD_LEARN_VZ     0.10f   // m/s
#define ALT_HOLD_LEARN_ERR    0.10f   // m, |z - z_sp|
#define ALT_HOLD_LEARN_TILT   0.9659258f  // cos(15 deg)
#define ALT_HOLD_HOVER_MIN    0.15f
#define ALT_HOLD_HOVER_MAX    0.80f
// Automatic take-off (user, 2026-11-12: Angle take-offs shot up, 0.05 of stick over hover ~ 1 m/s^2): armed on
// the floor with CH6 up, the throttle is capped at ALT_HOLD_GROUND_THR; the stick past ALT_HOLD_TAKEOFF_STICK starts
// a climb at vz_max to takeoff_alt, then an ordinary hold (position hold joins above its own minimum height).
#define ALT_HOLD_TAKEOFF_STICK 0.50f
#define ALT_HOLD_GROUND_THR    0.25f  // well under hover (~0.45): spins the props, never lifts
#define ALT_HOLD_SP_LEAD    0.50f  // m: the stick moves z_sp at most this far ahead of z
#define ALT_HOLD_MAX_ERR    1.00f  // m
#define ALT_HOLD_DEADBAND   0.10f  // stick within the engagement position ± this holds (PLAN: 0.4-0.6)
#define ALT_HOLD_BLEND_S    0.50f  // after an exit, throttle moves from the hold output to the stick over this

typedef struct {
    bool active;
    bool latched;     // refused or exited by itself: the switch must leave Alt Hold before another try
    float z_sp, i_v, base_thr, thr_out;
    float stick0;     // stick at engagement: centre of the hold band
    float blend;      // weight of thr_out after an exit, 1 -> 0
    float land_t;     // s the landed conditions have held
    bool landed;      // landed: throttle 0 until the stick is below ALT_HOLD_STICK_LOW
    float hover;      // learnt level hover throttle this power-up; 0 = not yet (use cfg hover_thr)
    float takeoff_target;  // m while an automatic take-off climbs, else 0
    bool landing;          // failsafe / commanded landing in progress
    bool land_failed;      // that landing had to quit; not restarted until the request ends
    bool maybe_landed;     // touching down: throttle ramping to idle, no horizontal control
    float ramp;            // throttle cap while maybe landed
    float z_last;          // m, last valid z (blind landing decision)
    float blind_t;         // s descending blind
    float vz_f;            // m/s, low-passed vz (landing detector)
} alt_hold_t;

void alt_hold_init(alt_hold_t *h);
// Returns the throttle command [0,1]: the hold output while active, 0 after a landing until the stick is lowered,
// otherwise the stick (blended after an exit). The caller must also stop the motors (idle) while h->landed.
float alt_hold_step(alt_hold_t *h, const alt_hold_cfg_t *c, const alt_hold_in_t *in, uint8_t *event,
                    float *event_val);

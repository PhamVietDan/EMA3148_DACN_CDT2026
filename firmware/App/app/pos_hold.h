// Position hold (step 3, on top of Alt Hold): roll/pitch sticks command a horizontal velocity; centred, the
// drone brakes, then anchors where it stopped and holds it. Position (room frame) and velocity (heading frame)
// come from app/flow_est.h. Structure as ArduPilot Loiter / the s450drone code (position P -> velocity PI ->
// tilt), not copied. Output: roll/pitch angle setpoints and a blend weight towards them. Pure C (host-tested).
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float pos_p;      // 1/s: position error -> velocity setpoint
    float vel_p;      // rad per m/s: velocity error -> tilt
    float vel_i;      // rad per m: integral (wind, trim)
    float vel_max;    // m/s at full stick
    float ang_max;    // rad: tilt limit of the hold
} pos_hold_cfg_t;

typedef struct {
    float dt;
    bool request;       // Alt Hold engaged, flow valid, high enough
    float stick[2];     // [0] roll, [1] pitch, [-1,1], exactly 0 inside the dead band
    float v[2];         // m/s, heading frame: forward, right
    float pos[2];       // m, room frame
    float yaw;          // rad, same gyro yaw as the position
    // No position hold (below the flow height, take-off) but still flying under Alt Hold: keep the learnt integral
    // as a fixed tilt correction (user's backward drift near the floor, 2026-11-13). Off once touching down.
    bool freeze;
} pos_hold_in_t;

typedef struct {
    float roll_sp, pitch_sp;  // rad
    float weight;             // 0: stick angles as in Angle, 1: these setpoints (blend over POS_HOLD_BLEND_S)
    float add_roll, add_pitch;  // rad, frozen integral, added to whatever the sticks / hold command
} pos_hold_out_t;

typedef enum { PH_EV_NONE, PH_EV_ON, PH_EV_OFF, PH_EV_ANCHOR } pos_hold_event_t;

// m: below, no position hold. 0.30 -> 0.20 (2026-11-13): the flow stayed valid down to ~0.2 m in the logs
// (FLOW_MIN_H), and the drone drifted back in the last 0.3 m of a landing.
#define POS_HOLD_MIN_Z      0.20f
#define POS_HOLD_STEADY_V   0.10f  // m/s: anchored and slower than this = hovering in place (trim learning)
#define POS_HOLD_TRIM_TAU_S 2.0f
#define POS_HOLD_TRIM_MIN_S 5.0f   // steady this long before the trim hint is worth printing
#define POS_HOLD_ANCHOR_V   0.15f  // m/s: below this after the sticks are released, anchor here
#define POS_HOLD_PULL_MAX   0.5f   // m/s: return-to-anchor speed limit
#define POS_HOLD_BLEND_S    0.5f
#define POS_HOLD_I_MAX      0.0872665f  // rad (5 deg) integral limit

typedef struct {
    bool active, anchored;
    float anchor[2];  // m, room frame
    float i[2];       // rad: forward, right
    float weight;
    float last_sp[2]; // rad: roll, pitch, held while blending out
    float i_avg[2];   // rad: integral averaged while hovering in place (in-flight trim)
    float steady_t;   // s hovering in place since this engagement
} pos_hold_t;

void pos_hold_init(pos_hold_t *h);
pos_hold_event_t pos_hold_step(pos_hold_t *h, const pos_hold_cfg_t *c, const pos_hold_in_t *in,
                               pos_hold_out_t *out);

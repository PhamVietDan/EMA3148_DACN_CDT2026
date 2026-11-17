// Altitude estimator (see alt_est.h). tools/replay_kf.py mirrors this file: keep the two in step.
#include "app/alt_est.h"

#include <math.h>

void alt_est_init(alt_est_t *e)
{
    *e = (alt_est_t){0};
    e->src = ALT_SRC_PREDICT;
}

static void clear_jumps(alt_est_t *e)
{
    e->jump_sum = 0.0f;
    e->jump_t = 0.0f;
}

// Run of fast ToF change (alt_est.h ALT_STEP_RATE). jump: ToF change not explained by vz; d_tof: the raw ToF
// change. A run starts on a frame where both are fast and the same way (ToF flat with a large "jump" only means vz
// is off: accelerometer bias), then takes every frame while the last ALT_STEP_RECENT frames together still move
// that way at ALT_STEP_AVG x the rate (one slow, noisy frame inside a ramp does not end it; a spike that came
// back does).
static void track_jumps(alt_est_t *e, float jump, float d_tof, float dt)
{
    for (uint32_t i = ALT_STEP_RECENT - 1u; i > 0u; i--) {
        e->recent_jump[i] = e->recent_jump[i - 1u];
        e->recent_tof[i] = e->recent_tof[i - 1u];
    }
    e->recent_jump[0] = jump;
    e->recent_tof[0] = d_tof;
    float lim = ALT_STEP_RATE * dt;
    if (e->jump_t > 0.0f) {
        float rj = 0.0f, rt = 0.0f;
        for (uint32_t i = 0; i < ALT_STEP_RECENT; i++) {
            rj += e->recent_jump[i];
            rt += e->recent_tof[i];
        }
        float need = ALT_STEP_AVG * lim * (float)ALT_STEP_RECENT;
        bool pos = e->jump_sum > 0.0f;
        if ((rj > 0.0f) == pos && (rt > 0.0f) == pos && fabsf(rj) >= need && fabsf(rt) >= need) {
            e->jump_sum += jump;
            e->jump_t += dt;
            return;
        }
        clear_jumps(e);
    }
    if (fabsf(jump) > lim && fabsf(d_tof) > lim && (jump > 0.0f) == (d_tof > 0.0f)) {
        e->jump_sum = jump;
        e->jump_t = dt;
    }
}

// z restarted from the floor reading: h_terrain = 0.
static void restart(alt_est_t *e, float tof_corr, uint32_t now_ms)
{
    clear_jumps(e);
    alt_kf_init(&e->kf, tof_corr, ALT_P0, ALT_P0);
    e->init = true;
    e->h_terrain = 0.0f;
    e->state = ALT_NORMAL;
    e->rejects = 0;
    e->spike_n = 0;
    e->at_rest = false;
    e->tof_prev = tof_corr;
    e->tof_prev_ms = now_ms;
}

static float snap_terrain(float h)
{
    return (fabsf(h) < ALT_TERRAIN_SNAP) ? 0.0f : h;
}

static void start_confirm(alt_est_t *e, float tof_corr, uint32_t now_ms)
{
    e->state = ALT_CONFIRM;
    e->new_level = tof_corr;
    e->confirm_n = 1;
    e->confirm_start_ms = now_ms;
}

// One usable ToF frame. level: tilt small enough to tell a terrain step from a tilted beam.
static alt_event_t tof_step(alt_est_t *e, const alt_cfg_t *c, float tof_corr, bool level, uint32_t now_ms,
                            float *ev_val)
{
    uint32_t gap_ms = now_ms - e->tof_prev_ms;
    float innov = tof_corr + e->h_terrain - e->kf.z;
    float dt_tof = (float)gap_ms * 0.001f;
    float d_tof = tof_corr - e->tof_prev;
    track_jumps(e, d_tof - e->kf.vz * dt_tof, d_tof, dt_tof);
    alt_event_t ev = ALT_EV_NONE;

    bool steps_on = c->step_th > 0.0f;  // 0: flat floor, no step detection (user, 2026-10-29)
    if (steps_on && e->state == ALT_NORMAL && (gap_ms > ALT_TOF_LOST_MS || fabsf(e->jump_sum) > c->step_th)) {
        // Back after a loss (the level may have changed meanwhile, §7.4b (3)) or a jump: check before use.
        // Tilted, the beam may just hit something off to the side: not used, but still the reference for the
        // next jump (otherwise every later frame looks like a jump and the filter runs on the accel alone).
        if (level) {
            start_confirm(e, tof_corr, now_ms);
        }
    } else if (e->state == ALT_NORMAL) {
        bool spike = fabsf(innov) > ALT_SPIKE_M;
        if (!spike) {
            e->spike_n = 0;
        }
        // Held longer while the ToF is still moving fast (an edge being crossed, alt_est.h ALT_STEP_RATE).
        // Steps off (alt_step_th 0): exactly the plain spike hold, nothing of the step check changes the flight.
        if (spike && (e->spike_n < ALT_SPIKE_N || (steps_on && e->jump_t > 0.0f))) {
            e->spike_n++;  // held back (alt_est.h ALT_SPIKE_M): the accelerometer carries z meanwhile
        } else if (alt_kf_update(&e->kf, tof_corr + e->h_terrain, c->sigma_tof, ALT_GATE_SQ)) {
            e->rejects = 0;
            e->src = ALT_SRC_TOF;
        } else if ((level || !steps_on) && ++e->rejects >= ALT_RESYNC_N) {
            // A steady ToF the filter keeps rejecting while level: z has drifted (accelerometer), the floor has
            // not. Tilted, the beam may be on something off to the side: never restart z from that.
            float z_meas = tof_corr + e->h_terrain;
            *ev_val = z_meas - e->kf.z;
            alt_kf_init(&e->kf, z_meas, ALT_P0, ALT_P0);
            e->rejects = 0;
            e->spike_n = 0;
            e->src = ALT_SRC_TOF;
            ev = ALT_EV_RESYNC;
        }
    } else {  // ALT_CONFIRM; tilted frames neither confirm nor reset the new level, the time limit still runs
        bool near = fabsf(tof_corr - e->new_level) <= ALT_CONFIRM_BAND;
        if (e->jump_t > 0.0f) {
            // Still crossing the edge (fast run going on): the new level is not reached yet; confirm from its end.
            e->new_level = tof_corr;
            e->confirm_n = 1;
            e->confirm_start_ms = now_ms;
        } else if (level && near) {
            if (++e->confirm_n >= ALT_CONFIRM_N) {
                float h = snap_terrain(e->kf.z - tof_corr);
                e->state = ALT_NORMAL;
                e->rejects = 0;
                clear_jumps(e);  // this change is dealt with: not counted again
                // A step must beat both the threshold and what z itself may be off by (3 sigma): after a gap
                // carried by the baro or the accelerometer, z can be a few tenths off.
                float z_sigma3 = 3.0f * sqrtf(fmaxf(e->kf.P[0][0], 0.0f));
                if (fabsf(h - e->h_terrain) < fmaxf(c->step_th, z_sigma3)) {
                    // Old level (a spike), or z off by less than a step after a ToF gap (tilted > 30 deg):
                    // that is filter error, left to the Kalman update, not baked into h_terrain.
                    ev = ALT_EV_STEP_NOISE;
                } else {
                    e->h_terrain = h;
                    ev = ALT_EV_STEP;
                    *ev_val = h;
                }
            }
        } else if (level) {
            e->new_level = tof_corr;
            e->confirm_n = 1;
        }
        if (e->state == ALT_CONFIRM && now_ms - e->confirm_start_ms > ALT_CONFIRM_MS) {
            e->state = ALT_NORMAL;  // not a stable new level: noise, keep the old h_terrain
            ev = ALT_EV_STEP_NOISE;
        }
    }
    e->tof_prev = tof_corr;
    e->tof_prev_ms = now_ms;
    return ev;
}

// Disarmed and back at the floor reading for ALT_REST_MS: whatever steps were (mis)detected while the drone
// was carried, it is on the floor now.
static alt_event_t ground_check(alt_est_t *e, float tof_corr, uint32_t now_ms, float *ev_val)
{
    if (fabsf(tof_corr - e->rest_tof) > ALT_REST_BAND || e->state != ALT_NORMAL) {
        e->at_rest = false;
        return ALT_EV_NONE;
    }
    if (!e->at_rest) {
        e->at_rest = true;
        e->rest_start_ms = now_ms;
        return ALT_EV_NONE;
    }
    if (now_ms - e->rest_start_ms < ALT_REST_MS ||
        (e->h_terrain == 0.0f && fabsf(e->kf.z - tof_corr) <= ALT_REST_BAND)) {
        return ALT_EV_NONE;
    }
    *ev_val = e->h_terrain;
    restart(e, tof_corr, now_ms);
    return ALT_EV_GROUND;
}

// M10b: baro_ref learning while the ToF carries z, baro fallback while it does not (alt_est.h).
static void baro_step(alt_est_t *e, const alt_cfg_t *c, const alt_in_t *in, bool tof_used_now)
{
    if (!in->baro_new) {
        return;
    }
    float dt = e->baro_seen ? (float)(in->now_ms - e->baro_ms) * 0.001f : 0.0f;
    e->baro_last = in->baro_alt;
    e->baro_ms = in->now_ms;
    e->baro_seen = true;
    if (!e->init) {
        return;
    }
    if (tof_used_now) {
        if (!e->baro_ref_ok) {
            e->baro_ref = in->baro_alt - e->kf.z;  // first time: drone still (boot or arming), as the baro p0
            e->baro_ref_ok = true;
        } else if (e->kf.z > ALT_BARO_MIN_Z && e->state == ALT_NORMAL && e->h_terrain == 0.0f && dt > 0.0f &&
                   dt < 0.5f) {
            e->baro_ref += (in->baro_alt - e->kf.z - e->baro_ref) * dt / ALT_BARO_REF_TAU_S;
        }
        return;
    }
    // No ToF for a moment (the predict already ran): high enough and not for too long, the baro carries z.
    if (e->baro_ref_ok && e->kf.z > ALT_BARO_MIN_Z && in->now_ms - e->tof_prev_ms > 2u * ALT_BARO_SLOT_MS &&
        in->now_ms - e->tof_prev_ms <= c->baro_max_ms &&
        alt_kf_update(&e->kf, in->baro_alt - e->baro_ref, c->sigma_baro, ALT_BARO_GATE_SQ)) {
        e->src = ALT_SRC_BARO;
        e->baro_used = true;
        e->baro_used_ms = in->now_ms;
    }
}

void alt_est_step(alt_est_t *e, const alt_cfg_t *c, const alt_in_t *in, alt_out_t *out)
{
    float tilt_cos = cosf(in->roll) * cosf(in->pitch);
    bool usable = in->tof_new && in->tof_in_range && tilt_cos > cosf(ALT_TILT_MAX_RAD);
    bool level = tilt_cos > cosf(ALT_STEP_TILT_RAD);
    float tof_corr = in->tof_dist * tilt_cos;
    alt_event_t ev = ALT_EV_NONE;
    float ev_val = 0.0f;

    if (!in->armed) {
        e->was_armed = false;
    }
    if (e->init) {
        alt_kf_predict(&e->kf, in->a_up, in->dt, c->sigma_a);
        e->src = ALT_SRC_PREDICT;
    }
    // The baro kept z through the gap: the returning ToF is checked against it, no restart from scratch.
    bool baro_bridged = e->baro_used && in->now_ms - e->baro_used_ms <= ALT_TOF_LOST_MS;
    if (usable) {
        if (!e->init || (in->now_ms - e->tof_prev_ms > ALT_REINIT_MS && !baro_bridged)) {
            if (!e->init) {
                e->rest_tof = tof_corr;  // boot calibration needs the drone still, normally on the floor
            }
            restart(e, tof_corr, in->now_ms);
            ev = ALT_EV_INIT;
            e->src = ALT_SRC_TOF;
        } else if (in->armed && !e->was_armed) {
            // Take-off floor = where the drone is armed (§7.4: z above the take-off floor).
            e->was_armed = true;
            ev_val = e->h_terrain;
            e->rest_tof = tof_corr;
            restart(e, tof_corr, in->now_ms);
            e->baro_ref_ok = false;  // re-taken against the new floor reference at the next baro sample
            ev = ALT_EV_ARM;
            e->src = ALT_SRC_TOF;
        } else {
            ev = tof_step(e, c, tof_corr, level, in->now_ms, &ev_val);
            if (ev == ALT_EV_NONE && !in->armed) {
                ev = ground_check(e, tof_corr, in->now_ms, &ev_val);
            }
        }
        e->baro_used = false;
    }
    baro_step(e, c, in, usable && e->src == ALT_SRC_TOF);

    out->z = e->kf.z;
    out->vz = e->kf.vz;
    out->h_terrain = e->h_terrain;
    out->tof_corr = e->tof_prev;
    out->src = e->src;
    out->step_state = e->state;
    out->valid = e->init && (in->now_ms - e->tof_prev_ms <= ALT_TOF_LOST_MS ||
                             (e->baro_used && in->now_ms - e->baro_used_ms <= ALT_BARO_STALE_MS &&
                              in->now_ms - e->tof_prev_ms <= c->baro_max_ms));
    out->z_floor = e->rest_tof;
    out->baro_ref = e->baro_ref;
    out->baro_ref_ok = e->baro_ref_ok;
    out->event = (uint8_t)ev;
    out->event_val = ev_val;
}

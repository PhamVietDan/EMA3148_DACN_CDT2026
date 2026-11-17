// Altitude hold (see alt_hold.h).
#include "app/alt_hold.h"

#include <math.h>

static float clampf_(float v, float lo, float hi)
{
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

void alt_hold_init(alt_hold_t *h)
{
    *h = (alt_hold_t){0};
}

static void leave(alt_hold_t *h, bool latch, bool blend)
{
    h->active = false;
    h->landing = false;
    h->maybe_landed = false;
    h->blind_t = 0.0f;
    h->latched = latch;
    h->blend = blend ? 1.0f : 0.0f;
}

// Stick away from where it was at engagement (± dead band): climb/descent rate, stick end = vz_max.
// Centred on the engagement stick, not 0.5: this drone hovers at ~0.41, at the edge of a fixed 0.4-0.6 band.
static float stick_rate(float stick, float centre, float vz_max)
{
    float d = stick - centre;
    if (fabsf(d) <= ALT_HOLD_DEADBAND) {
        return 0.0f;
    }
    float span = fmaxf((d > 0.0f ? 1.0f - centre : centre) - ALT_HOLD_DEADBAND, 0.05f);
    return vz_max * clampf_((d > 0.0f ? d - ALT_HOLD_DEADBAND : d + ALT_HOLD_DEADBAND) / span, -1.0f, 1.0f);
}

static float hold(alt_hold_t *h, const alt_hold_cfg_t *c, const alt_hold_in_t *in)
{
    // Failsafe / commanded landing: a fixed descent, whatever the stick (the landing detector then disarms).
    float rate = h->landing ? -fminf(c->vz_max, ALT_HOLD_LAND_VZ) : stick_rate(in->stick_thr, h->stick0, c->vz_max);
    if (h->landing) {
        h->takeoff_target = 0.0f;
    }
    if (h->takeoff_target > 0.0f) {
        // Auto take-off: the setpoint climbs at vz_max to the take-off height (not ahead of z by more than the
        // lead). The stick pulled down below the band cancels it: from then on it is an ordinary hold / descent.
        if (rate < 0.0f) {
            h->takeoff_target = 0.0f;
        } else {
            h->z_sp = fminf(h->z_sp + c->vz_max * in->dt, fminf(h->takeoff_target, in->z + ALT_HOLD_SP_LEAD));
            if (h->z_sp >= h->takeoff_target) {
                h->takeoff_target = 0.0f;
            }
            rate = 0.0f;
        }
    }
    if (rate > 0.0f) {
        h->z_sp = fminf(h->z_sp + rate * in->dt, fmaxf(in->z + ALT_HOLD_SP_LEAD, h->z_sp));
    } else if (rate < 0.0f) {
        h->z_sp = fmaxf(h->z_sp + rate * in->dt, fminf(in->z - ALT_HOLD_SP_LEAD, h->z_sp));
    }
    h->z_sp = fmaxf(h->z_sp, ALT_HOLD_MIN_ZSP);

    float vz_down = (in->z < ALT_HOLD_SLOW_Z) ? fminf(c->vz_max, ALT_HOLD_LAND_VZ) : c->vz_max;
    float vz_sp = clampf_(c->alt_p * (h->z_sp - in->z), -vz_down, c->vz_max);
    float e_v = vz_sp - in->vz;
    float i_new = clampf_(h->i_v + c->vz_i * e_v * in->dt, -c->thr_range, c->thr_range);
    float lo = fmaxf(h->base_thr - c->thr_range, 0.0f), hi = fminf(h->base_thr + c->thr_range, 1.0f);
    float tilt = fmaxf(in->tilt_cos, 0.5f);
    float thr = (h->base_thr + c->vz_p * e_v + i_new) / tilt;
    // Anti-windup: keep the integral only if the output is not pinned in the direction it pushes.
    if (!((thr > hi && e_v > 0.0f) || (thr < lo && e_v < 0.0f))) {
        h->i_v = i_new;
    }
    thr = clampf_(thr, lo, hi);

    // Holding still and level, not at a limit: the throttle flying the drone is the hover throttle.
    if (rate == 0.0f && fabsf(in->vz) < ALT_HOLD_LEARN_VZ && fabsf(in->z - h->z_sp) < ALT_HOLD_LEARN_ERR &&
        in->tilt_cos > ALT_HOLD_LEARN_TILT && thr > lo && thr < hi) {
        h->hover += (thr * tilt - h->hover) * in->dt / ALT_HOLD_LEARN_TAU_S;
        h->hover = clampf_(h->hover, ALT_HOLD_HOVER_MIN, ALT_HOLD_HOVER_MAX);
    }

    // Landing detector (log 2026-11-10 20:02: on the floor for 2.1 s, vz bounced +-0.19 m/s on the legs and the
    // integral needed ~3 s more to reach the throttle floor, so the old vz / throttle-limit test never held).
    // Maybe landed: asked down, z back at the floor reading of the arming, the setpoint already well below z.
    // Then the throttle ramps to idle at once (ArduPilot does the same), horizontal control stops (the caller
    // reads maybe_landed), and after ALT_HOLD_LAND_S it is landed.
    // While descending the setpoint always leads by ~vz/alt_p (0.3 m in that log), so "pressing" alone fires
    // before touch-down: the low-passed vz must also have stopped (bounces on the legs average out).
    h->vz_f += (in->vz - h->vz_f) * fminf(in->dt / ALT_HOLD_VZ_TAU_S, 1.0f);
    bool maybe = rate < 0.0f && in->z < in->z_floor + ALT_HOLD_FLOOR_BAND && h->z_sp < in->z - ALT_HOLD_PRESS &&
                 fabsf(h->vz_f) < ALT_HOLD_STILL_VZ;
    if (maybe) {
        if (!h->maybe_landed) {
            h->maybe_landed = true;
            h->ramp = thr;
        }
        h->ramp = fmaxf(h->ramp - ALT_HOLD_RAMP_PER_S * in->dt, 0.0f);
        thr = fminf(thr, h->ramp);
        h->land_t += in->dt;
    } else {
        h->maybe_landed = false;
        h->land_t = 0.0f;
    }
    return thr;
}

// Base: the learnt / saved hover throttle; unknown: the throttle flying it now (PLAN §7.5 bumpless).
static void engage(alt_hold_t *h, const alt_hold_cfg_t *c, const alt_hold_in_t *in)
{
    h->active = true;
    h->z_sp = in->z;
    h->i_v = 0.0f;
    h->takeoff_target = 0.0f;
    if (h->hover <= 0.0f && c->hover_thr > 0.0f) {
        h->hover = c->hover_thr;
    }
    h->base_thr = (h->hover > 0.0f) ? h->hover : in->stick_thr * fmaxf(in->tilt_cos, 0.5f);
    if (h->hover <= 0.0f) {
        h->hover = h->base_thr;
    }
    h->stick0 = in->stick_thr;
    h->land_t = 0.0f;
    h->maybe_landed = false;
    h->z_last = in->z;
    h->vz_f = in->vz;
}

float alt_hold_step(alt_hold_t *h, const alt_hold_cfg_t *c, const alt_hold_in_t *in, uint8_t *event,
                    float *event_val)
{
    alt_hold_event_t ev = AH_EV_NONE;
    *event_val = 0.0f;
    bool blind = false;

    if (h->landed) {
        // Motors stay at idle (the caller forces it) until the pilot lowers the stick: no surprise spool-up.
        if (in->stick_thr < ALT_HOLD_STICK_LOW) {
            h->landed = false;
        }
        *event = (uint8_t)ev;
        return 0.0f;
    }
    // Landing request (RC failsafe stage 2, or the LAND command): engage at any height, even after a refusal,
    // and descend; it ends with the landing detector, or when the request is withdrawn (hold resumes).
    // A landing that had to quit (estimate lost / far off) is not restarted: the caller then disarms (failsafe)
    // or hands back to the pilot (LAND command).
    if (in->land && in->flying && in->alt_valid && !h->land_failed && (!h->active || !h->landing)) {
        if (!h->active) {
            engage(h, c, in);
        }
        h->landing = true;
        h->latched = false;
        ev = AH_EV_LAND;
        *event_val = in->z;
    } else if (!in->land) {
        h->landing = false;
        h->land_failed = false;
    }
    if (!in->flying) {
        leave(h, false, false);  // disarmed or idle: nothing to hold, nothing to blend
    } else if (!in->request) {
        if (h->active) {
            leave(h, false, true);
            ev = AH_EV_EXIT_SWITCH;
        }
        h->latched = false;
    } else if (h->active) {
        if (!in->alt_valid && h->landing && h->z_last < ALT_HOLD_BLIND_Z) {
            // Landing, close to the floor, estimate gone (no ToF, and no baro below 0.5 m): keep descending blind
            // on a little under hover, then call it landed (user's timeout idea, 2026-11-13).
            blind = true;
        } else if (!in->alt_valid) {
            h->land_failed = h->landing;
            leave(h, true, true);
            ev = AH_EV_EXIT_INVALID;
        } else if (fabsf(in->z - h->z_sp) > ALT_HOLD_MAX_ERR) {
            h->land_failed = h->landing;
            leave(h, true, true);
            ev = AH_EV_EXIT_ERROR;
            *event_val = in->z - h->z_sp;
        }
    } else if (!h->latched) {
        bool can_take_off = c->takeoff_alt > 0.0f && (h->hover > 0.0f || c->hover_thr > 0.0f);
        if (!in->alt_valid) {
            h->latched = true;
            ev = AH_EV_REFUSED_INVALID;
        } else if (in->z < ALT_HOLD_MIN_Z && can_take_off) {
            // On (or just above) the floor with CH6 up: wait on a low throttle; stick past the take-off point
            // starts an automatic climb to takeoff_alt, independent of how the stick is held afterwards.
            if (in->stick_thr >= ALT_HOLD_TAKEOFF_STICK) {
                engage(h, c, in);
                h->takeoff_target = fmaxf(c->takeoff_alt, ALT_HOLD_MIN_Z);
                ev = AH_EV_TAKEOFF;
                *event_val = h->takeoff_target;
            } else {
                *event = (uint8_t)ev;
                return fminf(in->stick_thr, ALT_HOLD_GROUND_THR);
            }
        } else if (in->z < ALT_HOLD_MIN_Z) {
            h->latched = true;
            ev = AH_EV_REFUSED_LOW;
            *event_val = in->z;
        } else {
            engage(h, c, in);
            ev = AH_EV_ENGAGED;
            *event_val = h->z_sp;
        }
    }
    float thr;
    if (h->active && blind) {
        h->blind_t += in->dt;
        h->maybe_landed = true;  // no horizontal control without an estimate this close to the floor
        thr = ALT_HOLD_BLIND_THR * h->base_thr / fmaxf(in->tilt_cos, 0.5f);
        h->thr_out = thr;
        if (h->blind_t >= ALT_HOLD_BLIND_S) {
            leave(h, true, false);
            h->landed = true;
            ev = AH_EV_LANDED;
            *event_val = h->z_last;
            thr = 0.0f;
        }
    } else if (h->active) {
        h->blind_t = 0.0f;
        h->z_last = in->z;
        thr = hold(h, c, in);
        h->thr_out = thr;
        if (h->land_t >= ALT_HOLD_LAND_S) {
            leave(h, true, false);  // switch must leave Alt Hold before the next engagement
            h->landed = true;
            ev = AH_EV_LANDED;
            *event_val = in->z;
            thr = 0.0f;
        }
    } else if (h->blend > 0.0f) {
        thr = in->stick_thr + (h->thr_out - in->stick_thr) * h->blend;
        h->blend = fmaxf(h->blend - in->dt / ALT_HOLD_BLEND_S, 0.0f);
    } else {
        thr = in->stick_thr;
    }
    // CH6 up on the floor before the take-off (also during the motor spool-up, when 'flying' is still false):
    // the stick never lifts the drone by itself there, only the automatic take-off does.
    if (!h->active && !h->latched && in->request && in->z < ALT_HOLD_MIN_Z && c->takeoff_alt > 0.0f) {
        thr = fminf(thr, ALT_HOLD_GROUND_THR);
    }
    *event = (uint8_t)ev;
    return thr;
}

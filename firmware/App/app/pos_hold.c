// Position hold (see pos_hold.h).
#include "app/pos_hold.h"

#include <math.h>

#include "lib/mathx.h"

void pos_hold_init(pos_hold_t *h)
{
    *h = (pos_hold_t){0};
}

// Limits the length of a 2-vector (keeps the direction).
static void limit2(float v[2], float max)
{
    float n = sqrtf(v[0] * v[0] + v[1] * v[1]);
    if (n > max && n > 0.0f) {
        v[0] *= max / n;
        v[1] *= max / n;
    }
}

pos_hold_event_t pos_hold_step(pos_hold_t *h, const pos_hold_cfg_t *c, const pos_hold_in_t *in,
                               pos_hold_out_t *out)
{
    pos_hold_event_t ev = PH_EV_NONE;
    float step = in->dt / POS_HOLD_BLEND_S;
    out->add_roll = out->add_pitch = 0.0f;
    if (!in->request) {
        if (h->active) {
            ev = PH_EV_OFF;
        }
        h->active = h->anchored = false;
        // The integral is kept (not reset): it is the tilt this drone needs not to drift (trim error, wind),
        // reused as a frozen correction below the flow height and at the next take-off.
        h->weight = fmaxf(h->weight - step, 0.0f);
        out->roll_sp = h->last_sp[0];
        out->pitch_sp = h->last_sp[1];
        out->weight = h->weight;
        if (in->freeze) {
            // last_sp (fading with weight) already holds the integral: add the rest, so it stays whole.
            out->add_roll = (1.0f - h->weight) * h->i[1];
            out->add_pitch = -(1.0f - h->weight) * h->i[0];
        }
        return ev;
    }
    if (!h->active) {
        h->active = true;
        h->anchored = false;  // brake first, wherever the drone is going
        h->steady_t = 0.0f;
        ev = PH_EV_ON;
    }

    // Velocity setpoint, heading frame (forward, right). Stick forward = pitch_cmd < 0 = nose down = forward.
    float v_sp[2] = {0.0f, 0.0f};
    bool centred = in->stick[0] == 0.0f && in->stick[1] == 0.0f;
    if (!centred) {
        h->anchored = false;
        v_sp[0] = -in->stick[1] * c->vel_max;
        v_sp[1] = in->stick[0] * c->vel_max;
    } else if (!h->anchored) {
        if (sqrtf(in->v[0] * in->v[0] + in->v[1] * in->v[1]) < POS_HOLD_ANCHOR_V) {
            h->anchored = true;
            h->anchor[0] = in->pos[0];
            h->anchor[1] = in->pos[1];
            ev = PH_EV_ANCHOR;
        }
    } else {
        // Room-frame error rotated into the heading frame.
        float en = h->anchor[0] - in->pos[0], ee = h->anchor[1] - in->pos[1];
        float cy = cosf(in->yaw), sy = sinf(in->yaw);
        v_sp[0] = c->pos_p * (en * cy + ee * sy);
        v_sp[1] = c->pos_p * (-en * sy + ee * cy);
        limit2(v_sp, POS_HOLD_PULL_MAX);
    }

    // Velocity PI -> tilt (forward acceleration needs nose down: pitch = -tilt_fwd; right needs roll right).
    float tilt[2];
    for (int a = 0; a < 2; a++) {
        float e = v_sp[a] - in->v[a];
        float i_new = clampf(h->i[a] + c->vel_i * e * in->dt, -POS_HOLD_I_MAX, POS_HOLD_I_MAX);
        float t = c->vel_p * e + i_new;
        // Anti-windup: integrate only while the tilt is not pinned in the direction the error pushes.
        if (!((t > c->ang_max && e > 0.0f) || (t < -c->ang_max && e < 0.0f))) {
            h->i[a] = i_new;
        }
        tilt[a] = clampf(t, -c->ang_max, c->ang_max);
    }
    limit2(tilt, c->ang_max);
    // Anchored and still: the integral is the tilt needed to hover in place. Its average after a steady while
    // is the in-flight trim (printed on exit as SET lines).
    if (h->anchored && sqrtf(in->v[0] * in->v[0] + in->v[1] * in->v[1]) < POS_HOLD_STEADY_V) {
        float k = fminf(in->dt / POS_HOLD_TRIM_TAU_S, 1.0f);
        h->i_avg[0] += (h->i[0] - h->i_avg[0]) * k;
        h->i_avg[1] += (h->i[1] - h->i_avg[1]) * k;
        h->steady_t += in->dt;
    }
    h->last_sp[0] = tilt[1];
    h->last_sp[1] = -tilt[0];
    h->weight = fminf(h->weight + step, 1.0f);
    out->roll_sp = h->last_sp[0];
    out->pitch_sp = h->last_sp[1];
    out->weight = h->weight;
    return ev;
}

// Horizontal velocity from optical flow (see flow_est.h).
#include "app/flow_est.h"

#include <math.h>

#include "lib/mathx.h"

enum { PVV, PVB, PBB };

void flow_est_init(flow_est_t *e)
{
    *e = (flow_est_t){0};
    for (int a = 0; a < 2; a++) {
        e->P[a][PVV] = FLOW_P0;
        e->P[a][PBB] = FLOW_B0 * FLOW_B0;
    }
}

void flow_est_zero_position(flow_est_t *e)
{
    e->pos[0] = e->pos[1] = 0.0f;
}

void flow_est_predict(flow_est_t *e, const float acc[2], float r, float yaw, float dt)
{
    if (!e->init) {
        return;
    }
    // No usable flow this long (on the floor, h < FLOW_MIN_H): the next frame reseeds v anyway, and integrating
    // the accelerometer meanwhile ran v to -11 m/s and pos to -400 m on the floor (2026-11-16 14:11). Hold both.
    e->coast_s += dt;
    if (e->coast_s > (float)FLOW_RESEED_MS * 0.001f) {
        e->v[0] = e->v[1] = 0.0f;
        return;
    }
    // The heading frame turns with the drone: a velocity fixed in the room turns by -r*dt in it.
    float vf = e->v[0], vr = e->v[1];
    e->v[0] = vf + r * dt * vr + (acc[0] - e->b[0]) * dt;
    e->v[1] = vr - r * dt * vf + (acc[1] - e->b[1]) * dt;
    float qa = FLOW_SIGMA_A * dt;
    float qb = FLOW_SIGMA_B * FLOW_SIGMA_B * dt;
    for (int a = 0; a < 2; a++) {
        // F = [1 -dt; 0 1]: P = F P F' + Q
        float *P = e->P[a];
        P[PVV] += -2.0f * dt * P[PVB] + dt * dt * P[PBB] + qa * qa;
        P[PVB] -= dt * P[PBB];
        P[PBB] += qb;
    }
    float cy = cosf(yaw), sy = sinf(yaw);
    e->pos[0] += (e->v[0] * cy - e->v[1] * sy) * dt;
    e->pos[1] += (e->v[0] * sy + e->v[1] * cy) * dt;
}

bool flow_est_frame(flow_est_t *e, const flow_cfg_t *c, const flow_frame_t *f, uint32_t now_ms)
{
    // Gyro and estimate history, one entry per frame: the flow of this frame belongs to FLOW_LAG_FRAMES ago.
    for (uint32_t i = FLOW_LAG_FRAMES; i > 0u; i--) {
        for (int a = 0; a < 2; a++) {
            e->gyro_hist[i][a] = e->gyro_hist[i - 1u][a];
            e->v_hist[i][a] = e->v_hist[i - 1u][a];
        }
    }
    e->gyro_hist[0][0] = f->p;
    e->gyro_hist[0][1] = f->q;
    e->v_hist[0][0] = e->v[0];
    e->v_hist[0][1] = e->v[1];
    e->frames++;
    if (e->frames <= FLOW_LAG_FRAMES || (float)f->quality < c->q_min || f->h < FLOW_MIN_H ||
        f->tilt_cos < FLOW_TILT_MAX || c->k == 0.0f) {
        return false;
    }
    const float *g = e->gyro_hist[FLOW_LAG_FRAMES];
    float h = f->h * c->scale;
    // Back to the reference mount (flow_est.h)
    float mx = (float)f->mx, my = (float)f->my, rx, ry;
    switch (c->rot & 3u) {
    case 1u: rx = -my; ry = mx; break;
    case 2u: rx = -mx; ry = -my; break;
    case 3u: rx = my; ry = -mx; break;
    default: rx = mx; ry = my; break;
    }
    e->meas[0] = h * (ry / c->k - g[1]);
    e->meas[1] = -h * (rx / c->k - g[0]);
    float r_var = (c->sigma * f->h) * (c->sigma * f->h);
    // First frame, or back after a long gap (the estimate ran on the accelerometer alone): restart the velocity
    // from it. The bias is kept (it changes slowly), its correlation with the old velocity is dropped.
    if (!e->init || now_ms - e->last_ok_ms > FLOW_RESEED_MS) {
        for (int a = 0; a < 2; a++) {
            e->v[a] = e->meas[a];
            e->P[a][PVV] = r_var;
            e->P[a][PVB] = 0.0f;
        }
        e->init = true;
        e->last_ok_ms = now_ms;
        e->coast_s = 0.0f;
        return true;
    }
    bool used = false;
    for (int a = 0; a < 2; a++) {
        float *P = e->P[a];
        float innov = e->meas[a] - e->v_hist[FLOW_LAG_FRAMES][a];
        float s = P[PVV] + r_var;
        if (innov * innov > FLOW_GATE_SQ * s) {
            continue;
        }
        float kv = P[PVV] / s, kb = P[PVB] / s;
        e->v[a] += kv * innov;
        e->b[a] = clampf(e->b[a] + kb * innov, -FLOW_B_MAX, FLOW_B_MAX);
        float pvv = P[PVV], pvb = P[PVB];
        P[PVV] = pvv - kv * pvv;
        P[PVB] = pvb - kv * pvb;
        P[PBB] -= kb * pvb;
        used = true;
    }
    if (used) {
        e->last_ok_ms = now_ms;
        e->coast_s = 0.0f;
    }
    return used;
}

bool flow_est_valid(const flow_est_t *e, uint32_t now_ms)
{
    return e->init && now_ms - e->last_ok_ms <= FLOW_VALID_MS;
}

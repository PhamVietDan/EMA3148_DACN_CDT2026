// Host tests for App/app/pos_hold.c on a point mass: tilt follows its setpoint with a 0.15 s lag (Angle loop),
// acceleration g*tan(tilt), velocity seen 30 ms late with noise (flow estimate).
#include <math.h>
#include <stdlib.h>

#include "app/pos_hold.h"
#include "lib/mathx.h"
#include "test_common.h"

// = the parameter defaults (app/params.c)
static const pos_hold_cfg_t cfg = {.pos_p = 0.8f, .vel_p = 10.0f * MATHX_DEG2RAD, .vel_i = 3.0f * MATHX_DEG2RAD,
                                   .vel_max = 1.0f, .ang_max = 12.0f * MATHX_DEG2RAD};

#define DELAY 30  // ms

typedef struct {
    pos_hold_t h;
    pos_hold_in_t in;
    float roll, pitch;     // actual tilt, rad
    float vn, ve, pn, pe;  // room frame truth
    float yaw, yaw_rate;
    float vhist[DELAY][2];
    float wind[2];         // room-frame disturbance acceleration
    int anchors;
} sim_t;

static void sim_init(sim_t *s)
{
    *s = (sim_t){0};
    pos_hold_init(&s->h);
    s->in.dt = 0.001f;
    s->in.request = true;
    srand(5);
}

static void sim_run(sim_t *s, int ms)
{
    for (int i = 0; i < ms; i++) {
        float cy = cosf(s->yaw), sy = sinf(s->yaw);
        // heading-frame truth, delayed + noisy as the flow estimate
        float vf = s->vn * cy + s->ve * sy, vr = -s->vn * sy + s->ve * cy;
        for (int k = DELAY - 1; k > 0; k--) {
            s->vhist[k][0] = s->vhist[k - 1][0];
            s->vhist[k][1] = s->vhist[k - 1][1];
        }
        s->vhist[0][0] = vf;
        s->vhist[0][1] = vr;
        s->in.v[0] = s->vhist[DELAY - 1][0] + 0.05f * (2.0f * (float)rand() / (float)RAND_MAX - 1.0f);
        s->in.v[1] = s->vhist[DELAY - 1][1] + 0.05f * (2.0f * (float)rand() / (float)RAND_MAX - 1.0f);
        s->in.pos[0] = s->pn;
        s->in.pos[1] = s->pe;
        s->in.yaw = s->yaw;
        pos_hold_out_t o;
        s->anchors += pos_hold_step(&s->h, &cfg, &s->in, &o) == PH_EV_ANCHOR;
        float roll_sp = o.weight * o.roll_sp, pitch_sp = o.weight * o.pitch_sp;
        s->roll += (roll_sp - s->roll) * 0.001f / 0.15f;
        s->pitch += (pitch_sp - s->pitch) * 0.001f / 0.15f;
        float af = -MATHX_G * tanf(s->pitch), ar = MATHX_G * tanf(s->roll);
        s->vn += (af * cy - ar * sy + s->wind[0]) * 0.001f;
        s->ve += (af * sy + ar * cy + s->wind[1]) * 0.001f;
        s->pn += s->vn * 0.001f;
        s->pe += s->ve * 0.001f;
        s->yaw = wrap_pi(s->yaw + s->yaw_rate * 0.001f);
    }
}

static void test_brake_and_anchor(void)
{
    sim_t s;
    sim_init(&s);
    s.vn = 0.8f;  // engaged while drifting forward (the flow estimate already sees it)
    for (int k = 0; k < DELAY; k++) {
        s.vhist[k][0] = 0.8f;
    }
    float vmax_late = 0.0f;
    sim_run(&s, 3000);
    CHECK(s.anchors == 1 && s.h.anchored, "no anchor after braking (anchors %d)", s.anchors);
    float ax = s.h.anchor[0];
    for (int i = 0; i < 5000; i++) {
        sim_run(&s, 1);
        vmax_late = fmaxf(vmax_late, fabsf(s.vn));
    }
    CHECK(ax > 0.1f && ax < 1.0f && fabsf(s.pn - ax) < 0.05f && fabsf(s.pe) < 0.05f && vmax_late < 0.1f,
          "brake: anchor %.2f m, now %.2f %.2f, |v| %.2f", (double)ax, (double)s.pn, (double)s.pe, (double)vmax_late);
}

static void test_wind(void)
{
    sim_t s;
    sim_init(&s);
    sim_run(&s, 2000);
    s.wind[1] = 0.5f;  // steady push to the east (~3 deg of tilt to hold)
    float worst = 0.0f;
    for (int i = 0; i < 15000; i++) {
        sim_run(&s, 1);
        worst = fmaxf(worst, hypotf(s.pn - s.h.anchor[0], s.pe - s.h.anchor[1]));
    }
    float err = hypotf(s.pn - s.h.anchor[0], s.pe - s.h.anchor[1]);
    CHECK(worst < 0.4f && err < 0.05f, "wind: worst %.2f m, after 15 s %.3f m", (double)worst, (double)err);
}

static void test_stick_velocity_and_yaw(void)
{
    sim_t s;
    sim_init(&s);
    s.yaw = 90.0f * MATHX_DEG2RAD;  // nose east
    sim_run(&s, 1000);
    s.in.stick[1] = -0.5f;          // half stick forward: 0.5 m/s along the nose (east)
    sim_run(&s, 4000);
    CHECK(fabsf(s.ve - 0.5f) < 0.05f && fabsf(s.vn) < 0.05f, "stick: v n %.2f e %.2f", (double)s.vn,
          (double)s.ve);
    s.in.stick[1] = 0.0f;
    sim_run(&s, 3000);
    float e0 = s.pe;
    s.yaw_rate = 1.0f;              // spin while holding: the anchor stays put in the room
    sim_run(&s, 3000);
    s.yaw_rate = 0.0f;
    sim_run(&s, 3000);
    CHECK(s.h.anchored && fabsf(s.pe - e0) < 0.15f && fabsf(s.vn) < 0.05f, "spin: drift %.2f m",
          (double)(s.pe - e0));
}

static void test_off_blends_out(void)
{
    sim_t s;
    sim_init(&s);
    sim_run(&s, 1000);
    s.in.request = false;
    pos_hold_out_t o;
    s.in.dt = 0.25f;
    CHECK(pos_hold_step(&s.h, &cfg, &s.in, &o) == PH_EV_OFF && o.weight > 0.4f && o.weight < 0.6f,
          "off: weight %.2f after 0.25 s", (double)o.weight);
    (void)pos_hold_step(&s.h, &cfg, &s.in, &o);
    CHECK(o.weight == 0.0f && !s.h.active, "off: weight %.2f after 0.5 s", (double)o.weight);
}

// Below the flow height (freeze): the learnt integral stays applied whole while the hold blends out; the steady
// hover also yields the in-flight trim (the integral that held it against the push).
static void test_freeze_and_trim(void)
{
    sim_t s;
    sim_init(&s);
    s.wind[1] = 0.5f;  // a steady push needs a steady tilt: the integral learns it
    sim_run(&s, 15000);
    CHECK(s.h.steady_t > 5.0f && fabsf(s.h.i_avg[1] - s.h.i[1]) < 0.005f && s.h.i[1] < -0.02f,
          "trim learning: steady %.1f s, i %.3f avg %.3f rad", (double)s.h.steady_t, (double)s.h.i[1],
          (double)s.h.i_avg[1]);
    float i_right = s.h.i[1];
    s.in.request = false;
    s.in.freeze = true;
    s.in.dt = 0.1f;
    pos_hold_out_t o;
    float worst = 0.0f;
    for (int k = 0; k < 8; k++) {
        (void)pos_hold_step(&s.h, &cfg, &s.in, &o);
        // the integral part of the roll command: weight x (part of last_sp) + frozen add
        float total = o.weight * i_right + o.add_roll;
        worst = fmaxf(worst, fabsf(total - i_right));
    }
    CHECK(o.weight == 0.0f && fabsf(o.add_roll - i_right) < 1e-6f && worst < 1e-5f,
          "freeze: add %.4f vs i %.4f, worst %.2g", (double)o.add_roll, (double)i_right, (double)worst);
    s.in.freeze = false;
    (void)pos_hold_step(&s.h, &cfg, &s.in, &o);
    CHECK(o.add_roll == 0.0f && o.add_pitch == 0.0f, "freeze off still adds");
}

void test_pos_hold(void)
{
    test_freeze_and_trim();
    test_brake_and_anchor();
    test_wind();
    test_stick_velocity_and_yaw();
    test_off_blends_out();
}

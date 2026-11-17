// Host tests for App/app/alt_hold.c (PLAN §7.5, M10) on a point-mass model: thrust = thr / hover * g.
#include <math.h>

#include "app/alt_hold.h"
#include "lib/mathx.h"
#include "test_common.h"

// = the parameter defaults (app/params.c)
static const alt_hold_cfg_t cfg = {.alt_p = 1.0f, .vz_p = 0.10f, .vz_i = 0.10f, .vz_max = 0.5f, .thr_range = 0.15f};

#define HOVER 0.41f  // true hover throttle of the model
#define G     9.80665f
#define FLOOR_Z 0.13f  // ToF on the floor (2026-10-29 logs)

typedef struct {
    alt_hold_t h;
    alt_hold_in_t in;
    float z, vz, thr;
    int engaged, exits, refusals;
} sim_t;

static void sim_init(sim_t *s, float z0, float stick)
{
    *s = (sim_t){0};
    alt_hold_init(&s->h);
    s->z = z0;
    s->in = (alt_hold_in_t){.dt = 0.001f, .request = false, .flying = true, .alt_valid = true,
                            .stick_thr = stick, .tilt_cos = 1.0f, .z_floor = FLOOR_Z};
}

static void sim_run(sim_t *s, uint32_t ms, float accel_dist)
{
    for (uint32_t i = 0; i < ms; i++) {
        s->in.z = s->z;
        s->in.vz = s->vz;
        uint8_t ev;
        float val;
        s->thr = alt_hold_step(&s->h, &cfg, &s->in, &ev, &val);
        s->engaged += ev == AH_EV_ENGAGED;
        s->exits += ev == AH_EV_EXIT_SWITCH || ev == AH_EV_EXIT_INVALID || ev == AH_EV_EXIT_ERROR;
        s->refusals += ev == AH_EV_REFUSED_INVALID || ev == AH_EV_REFUSED_LOW;
        float a = s->thr / HOVER * G - G + accel_dist;
        s->vz += a * 0.001f;
        s->z += s->vz * 0.001f;
        if (s->z <= FLOOR_Z) {  // resting on its legs: ToF reads the sensor height
            s->z = FLOOR_Z;
            s->vz = fmaxf(s->vz, 0.0f);
        }
    }
}

static void test_hold_with_wrong_base(void)
{
    sim_t s;
    sim_init(&s, 0.7f, 0.38f);  // engaged while the pilot was slightly under the true hover throttle
    s.in.request = true;
    sim_run(&s, 1, 0.0f);
    CHECK(s.engaged == 1 && s.h.active, "not engaged");
    sim_run(&s, 15000, 0.0f);
    CHECK(fabsf(s.z - 0.7f) < 0.03f && fabsf(s.vz) < 0.02f, "hold: z %.3f vz %.3f thr %.3f", (double)s.z,
          (double)s.vz, (double)s.thr);
    // Engaged on 0.38 with no saved hover throttle: it learnt the model's true hover.
    CHECK(fabsf(s.h.hover - HOVER) < 0.01f, "hover learnt %.3f (true %.3f)", (double)s.h.hover, (double)HOVER);
    float worst = 0.0f;
    for (int i = 0; i < 3000; i++) {  // gust: 0.5 m/s^2 down for 3 s (the learnt value follows it, as it should)
        sim_run(&s, 1, -0.5f);
        worst = fmaxf(worst, fabsf(s.z - 0.7f));
    }
    CHECK(worst < 0.11f, "gust: worst error %.3f", (double)worst);  // delay-free model: 0.100 m
}

// Saved hover throttle: engaging while climbing on a high stick does not carry that throttle into the hold.
static void test_engage_on_saved_hover(void)
{
    alt_hold_cfg_t c = cfg;
    c.hover_thr = HOVER;
    sim_t s;
    sim_init(&s, 0.8f, 0.55f);
    s.vz = 0.4f;
    s.in.request = true;
    float zmax = 0.0f;
    for (int i = 0; i < 6000; i++) {
        s.in.z = s.z;
        s.in.vz = s.vz;
        uint8_t ev;
        float val;
        s.thr = alt_hold_step(&s.h, &c, &s.in, &ev, &val);
        s.vz += (s.thr / HOVER * G - G) * 0.001f;
        s.z += s.vz * 0.001f;
        zmax = fmaxf(zmax, s.z);
    }
    CHECK(fabsf(s.h.base_thr - HOVER) < 1e-6f && zmax < 1.05f && fabsf(s.z - 0.8f) < 0.05f,
          "saved hover: base %.3f overshoot to %.3f, z %.3f", (double)s.h.base_thr, (double)zmax, (double)s.z);
}

static void test_thr_curve(void)
{
    CHECK(thr_curve(0.0f, 0.41f, 0.5f) == 0.0f && fabsf(thr_curve(1.0f, 0.41f, 0.5f) - 1.0f) < 1e-6f &&
              fabsf(thr_curve(0.41f, 0.41f, 0.5f) - 0.41f) < 1e-6f,
          "curve end points");
    CHECK(fabsf(thr_curve(0.6f, 0.41f, 0.0f) - 0.6f) < 1e-6f, "expo 0 not linear");
    float prev = -1.0f, d_lo = thr_curve(0.46f, 0.41f, 0.5f) - 0.41f;
    for (int i = 0; i <= 100; i++) {
        float y = thr_curve((float)i / 100.0f, 0.41f, 0.5f);
        CHECK(y >= prev, "not monotonic at %d", i);
        prev = y;
    }
    CHECK(d_lo < 0.03f, "not flatter around mid: +0.05 stick -> %.3f", (double)d_lo);
}

static void test_stick_climb(void)
{
    sim_t s;
    sim_init(&s, 0.7f, HOVER);
    s.in.request = true;
    sim_run(&s, 3000, 0.0f);
    s.in.stick_thr = 1.0f;
    float vmax = 0.0f;
    for (int i = 0; i < 2000; i++) {
        sim_run(&s, 1, 0.0f);
        vmax = fmaxf(vmax, s.vz);
    }
    s.in.stick_thr = 0.5f;
    sim_run(&s, 5000, 0.0f);
    CHECK(s.z > 1.4f && s.z < 1.9f && vmax < 0.7f && fabsf(s.z - s.h.z_sp) < 0.05f,
          "climb: z %.3f z_sp %.3f vmax %.3f", (double)s.z, (double)s.h.z_sp, (double)vmax);
}

static void test_exit_on_invalid_and_blend(void)
{
    sim_t s;
    sim_init(&s, 0.8f, 0.6f);  // stick at 0.6 (top of the dead band): holds
    s.in.request = true;
    sim_run(&s, 3000, 0.0f);
    float thr_hold = s.thr;
    s.in.alt_valid = false;
    sim_run(&s, 1, 0.0f);
    CHECK(!s.h.active && s.exits == 1 && fabsf(s.thr - thr_hold) < 0.01f, "exit: active %d thr %.3f (hold %.3f)",
          s.h.active, (double)s.thr, (double)thr_hold);
    sim_run(&s, 600, 0.0f);
    CHECK(fabsf(s.thr - 0.6f) < 1e-4f, "blend: thr %.3f", (double)s.thr);
    s.in.alt_valid = true;
    sim_run(&s, 100, 0.0f);
    CHECK(!s.h.active, "re-engaged without a switch cycle");
    s.in.request = false;
    sim_run(&s, 10, 0.0f);
    s.in.request = true;
    sim_run(&s, 10, 0.0f);
    CHECK(s.h.active && s.engaged == 2, "switch cycle did not re-engage");
}

static void test_refused_low(void)
{
    sim_t s;
    sim_init(&s, 0.2f, HOVER);
    s.in.request = true;
    sim_run(&s, 100, 0.0f);
    CHECK(!s.h.active && s.refusals == 1, "engaged at 0.2 m (refusals %d)", s.refusals);
    s.z = 0.6f;
    sim_run(&s, 100, 0.0f);
    CHECK(!s.h.active, "engaged after the refusal without a switch cycle");
}

static void test_output_bounded(void)
{
    sim_t s;
    sim_init(&s, 0.8f, 0.40f);
    s.in.request = true;
    sim_run(&s, 2000, 0.0f);
    // Estimate jumps 0.9 m low (bad accel): the hold pushes up, but only to base + range.
    float base = s.h.base_thr, thr_max = 0.0f;
    for (int i = 0; i < 500; i++) {
        s.in.z = s.z - 0.9f;
        s.in.vz = -0.5f;
        uint8_t ev;
        float val;
        thr_max = fmaxf(thr_max, alt_hold_step(&s.h, &cfg, &s.in, &ev, &val));
    }
    CHECK(thr_max <= base + cfg.thr_range + 1e-5f, "throttle %.3f above base %.3f + range", (double)thr_max,
          (double)base);
    s.in.z = s.z - 1.2f;
    uint8_t ev;
    float val;
    (void)alt_hold_step(&s.h, &cfg, &s.in, &ev, &val);
    CHECK(!s.h.active && ev == AH_EV_EXIT_ERROR, "no exit at 1.2 m error");
}

static void test_not_flying(void)
{
    sim_t s;
    sim_init(&s, 0.8f, 0.3f);
    s.in.request = true;
    sim_run(&s, 100, 0.0f);
    s.in.flying = false;  // disarmed / idle
    sim_run(&s, 1, 0.0f);
    CHECK(!s.h.active && s.h.blend == 0.0f && s.thr == 0.3f, "still active when not flying");
}

static void test_land_with_stick(void)
{
    sim_t s;
    sim_init(&s, 0.8f, HOVER);
    s.in.request = true;
    sim_run(&s, 3000, 0.0f);
    s.in.stick_thr = 0.15f;  // pilot pulls the stick down
    float vmin_low = 0.0f;
    int landed_ms = -1;
    for (int i = 0; i < 15000 && landed_ms < 0; i++) {
        sim_run(&s, 1, 0.0f);
        if (s.z < 0.4f) {
            vmin_low = fminf(vmin_low, s.vz);
        }
        if (s.h.landed) {
            landed_ms = i;
        }
    }
    CHECK(landed_ms > 0 && s.thr == 0.0f && !s.h.active && vmin_low > -0.4f && s.z <= FLOOR_Z + 0.01f,
          "landing: landed at %d ms thr %.3f slowest descent below 0.4 m %.3f", landed_ms, (double)s.thr,
          (double)vmin_low);
    sim_run(&s, 1000, 0.0f);  // stick still at 0.15: motors must stay off
    CHECK(s.h.landed && s.thr == 0.0f, "spooled up again before the stick was lowered");
    s.in.stick_thr = 0.0f;
    sim_run(&s, 10, 0.0f);
    CHECK(!s.h.landed, "landed not cleared with the stick down");
}

static void test_no_false_landing_in_hover(void)
{
    sim_t s;
    sim_init(&s, 0.35f, HOVER);  // low hover, stick slightly down then centred
    s.in.request = true;
    sim_run(&s, 100, 0.0f);
    s.in.stick_thr = 0.30f;  // descend
    sim_run(&s, 500, 0.0f);
    s.in.stick_thr = HOVER;  // stop: hold
    sim_run(&s, 8000, 0.0f);
    CHECK(s.h.active && !s.h.landed && s.z > 0.2f, "false landing: active %d landed %d z %.3f", s.h.active,
          s.h.landed, (double)s.z);
}

// CH6 up on the floor: a low stick never lifts it; past the take-off point it climbs to takeoff_alt and holds,
// whatever the stick does inside the hold band; pulled down during the climb, it cancels and lands.
static void test_auto_takeoff(void)
{
    alt_hold_cfg_t c = cfg;
    c.hover_thr = HOVER;
    c.takeoff_alt = 0.8f;
    sim_t s;
    sim_init(&s, FLOOR_Z, 0.40f);  // armed, motors running, stick just under hover
    s.in.request = true;
    for (int i = 0; i < 2000; i++) {
        s.in.z = s.z;
        s.in.vz = s.vz;
        uint8_t ev;
        float val;
        s.thr = alt_hold_step(&s.h, &c, &s.in, &ev, &val);
        s.vz += (s.thr / HOVER * G - G) * 0.001f;
        s.z += s.vz * 0.001f;
        if (s.z <= FLOOR_Z) {
            s.z = FLOOR_Z;
            s.vz = fmaxf(s.vz, 0.0f);
        }
    }
    CHECK(s.z == FLOOR_Z && s.thr <= ALT_HOLD_GROUND_THR + 1e-6f && !s.h.active,
          "waiting: z %.3f thr %.3f active %d", (double)s.z, (double)s.thr, s.h.active);
    s.in.stick_thr = 0.60f;  // a "normal" take-off stick, which in Angle would climb at ~3 m/s^2
    float zmax = 0.0f, vmax = 0.0f;
    for (int i = 0; i < 8000; i++) {
        s.in.z = s.z;
        s.in.vz = s.vz;
        uint8_t ev;
        float val;
        s.thr = alt_hold_step(&s.h, &c, &s.in, &ev, &val);
        s.vz += (s.thr / HOVER * G - G) * 0.001f;
        s.z += s.vz * 0.001f;
        if (s.z <= FLOOR_Z) {
            s.z = FLOOR_Z;
            s.vz = fmaxf(s.vz, 0.0f);
        }
        zmax = fmaxf(zmax, s.z);
        vmax = fmaxf(vmax, s.vz);
    }
    CHECK(s.h.active && fabsf(s.z - 0.8f) < 0.03f && zmax < 0.9f && vmax < 0.75f,
          "take-off: z %.3f (max %.3f) vmax %.2f", (double)s.z, (double)zmax, (double)vmax);
    // Cancel: a second take-off pulled down mid-climb lands.
    sim_init(&s, FLOOR_Z, 0.60f);
    s.in.request = true;
    for (int i = 0; i < 600; i++) {  // 0.6 s into the climb
        s.in.z = s.z;
        s.in.vz = s.vz;
        uint8_t ev;
        float val;
        s.thr = alt_hold_step(&s.h, &c, &s.in, &ev, &val);
        s.vz += (s.thr / HOVER * G - G) * 0.001f;
        s.z += s.vz * 0.001f;
    }
    s.in.stick_thr = 0.10f;
    for (int i = 0; i < 10000 && !s.h.landed; i++) {
        s.in.z = s.z;
        s.in.vz = s.vz;
        uint8_t ev;
        float val;
        s.thr = alt_hold_step(&s.h, &c, &s.in, &ev, &val);
        s.vz += (s.thr / HOVER * G - G) * 0.001f;
        s.z += s.vz * 0.001f;
        if (s.z <= FLOOR_Z) {
            s.z = FLOOR_Z;
            s.vz = fmaxf(s.vz, 0.0f);
        }
    }
    CHECK(s.h.landed && s.h.takeoff_target == 0.0f, "cancelled take-off did not land (z %.3f)", (double)s.z);
}

// Failsafe / LAND: flying in Angle (hold not engaged) at 1 m, the request engages and descends at <= 0.3 m/s,
// whatever the stick, down to the landing detector.
static void test_forced_landing(void)
{
    alt_hold_cfg_t c = cfg;
    c.hover_thr = HOVER;       // as in the firmware (parameter hover_thr, learnt in flight)
    sim_t s;
    sim_init(&s, 1.0f, 0.9f);  // stick far up: must be ignored
    s.in.request = true;
    s.in.land = true;
    float vmin = 0.0f;
    int landed_ms = -1, land_events = 0;
    for (int i = 0; i < 15000 && landed_ms < 0; i++) {
        s.in.z = s.z;
        s.in.vz = s.vz;
        uint8_t ev;
        float val;
        s.thr = alt_hold_step(&s.h, &c, &s.in, &ev, &val);
        land_events += ev == AH_EV_LAND;
        s.vz += (s.thr / HOVER * G - G) * 0.001f;
        s.z += s.vz * 0.001f;
        if (s.z <= FLOOR_Z) {
            s.z = FLOOR_Z;
            s.vz = fmaxf(s.vz, 0.0f);
        }
        vmin = fminf(vmin, s.vz);
        if (ev == AH_EV_LANDED) {
            landed_ms = i;
        }
    }
    CHECK(land_events == 1 && landed_ms > 0 && vmin > -0.4f && s.h.landed,
          "forced landing: events %d landed at %d ms, fastest descent %.2f m/s", land_events, landed_ms,
          (double)vmin);

    // Estimate lost during the landing: it quits once and is not restarted (no event flood).
    sim_init(&s, 1.0f, 0.5f);
    s.in.request = true;
    s.in.land = true;
    sim_run(&s, 100, 0.0f);
    s.in.alt_valid = false;
    sim_run(&s, 10, 0.0f);
    s.in.alt_valid = true;
    int before = s.engaged;
    land_events = 0;
    for (int i = 0; i < 500; i++) {
        s.in.z = s.z;
        s.in.vz = s.vz;
        uint8_t ev;
        float val;
        (void)alt_hold_step(&s.h, &c, &s.in, &ev, &val);
        land_events += ev == AH_EV_LAND;
    }
    CHECK(s.h.land_failed && !s.h.landing && land_events == 0 && s.engaged == before,
          "failed landing restarted (%d events)", land_events);
}

void test_alt_hold(void)
{
    test_forced_landing();
    test_auto_takeoff();
    test_engage_on_saved_hover();
    test_thr_curve();
    test_land_with_stick();
    test_no_false_landing_in_hover();
    test_hold_with_wrong_base();
    test_stick_climb();
    test_exit_on_invalid_and_blend();
    test_refused_low();
    test_output_bounded();
    test_not_flying();
}

// Host tests for App/lib/alt_kf.c and App/app/alt_est.c (PLAN §7.4, §7.4b, M9 acceptance).
#include <math.h>
#include <stdlib.h>

#include "app/alt_est.h"
#include "lib/mathx.h"
#include "test_common.h"

static const alt_cfg_t cfg = {.sigma_a = 0.3f, .sigma_tof = 0.05f, .step_th = 0.10f, .sigma_baro = 0.3f,
                              .baro_max_ms = 3000u};

// Deterministic noise in [-a, a].
static float noise(float a)
{
    return a * (2.0f * (float)rand() / (float)RAND_MAX - 1.0f);
}

typedef struct {
    alt_est_t e;
    alt_out_t out;
    uint32_t now_ms;
    float z, vz;  // truth
    float acc_bias, roll;
    bool armed;
    bool baro;            // baro samples every 30 ms: z + baro_offset + noise
    float baro_offset;
    int steps, noise_evs, resyncs, grounds, arms, inits;
    const alt_cfg_t *cfg;  // NULL: cfg above
} sim_t;

static void sim_init(sim_t *s, float z0)
{
    *s = (sim_t){0};
    srand(7);
    alt_est_init(&s->e);
    s->now_ms = 1000;
    s->z = z0;
}

// 10 ms per step. terrain: height of the surface below (m). tof_on: sensor delivers a frame.
static void sim_run(sim_t *s, uint32_t ms, float accel, float terrain, bool tof_on)
{
    for (uint32_t t = 0; t < ms; t += 10u) {
        const float dt = 0.01f;
        s->z += s->vz * dt + 0.5f * accel * dt * dt;
        s->vz += accel * dt;
        s->now_ms += 10u;
        alt_in_t in = {
            .now_ms = s->now_ms,
            .dt = dt,
            .a_up = accel + s->acc_bias + noise(0.2f),
            .tof_new = tof_on,
            .tof_in_range = true,
            .tof_dist = (s->z - terrain) / cosf(s->roll) + noise(0.01f),
            .roll = s->roll,
            .armed = s->armed,
            .baro_new = s->baro && (s->now_ms / 10u) % 3u == 0u,
            .baro_alt = s->z + s->baro_offset + noise(0.05f),
        };
        alt_est_step(&s->e, s->cfg ? s->cfg : &cfg, &in, &s->out);
        s->inits += s->out.event == ALT_EV_INIT;
        s->steps += s->out.event == ALT_EV_STEP;
        s->noise_evs += s->out.event == ALT_EV_STEP_NOISE;
        s->resyncs += s->out.event == ALT_EV_RESYNC;
        s->grounds += s->out.event == ALT_EV_GROUND;
        s->arms += s->out.event == ALT_EV_ARM;
    }
}

static void test_tracks_climb(void)
{
    sim_t s;
    sim_init(&s, 0.05f);
    sim_run(&s, 1000, 0.0f, 0.0f, true);
    CHECK(s.out.valid && fabsf(s.out.z - 0.05f) < 0.02f, "rest: z %.3f", (double)s.out.z);
    sim_run(&s, 1000, 1.0f, 0.0f, true);   // accelerate up 1 s
    sim_run(&s, 1000, -1.0f, 0.0f, true);  // decelerate 1 s: climbed 1 m
    CHECK(fabsf(s.out.z - s.z) < 0.03f && fabsf(s.out.vz - s.vz) < 0.1f, "climb: z %.3f (true %.3f) vz %.3f",
          (double)s.out.z, (double)s.z, (double)s.out.vz);
    CHECK(s.steps == 0 && s.resyncs == 0 && s.out.h_terrain == 0.0f, "climb mistaken for a step");
}

static void test_table(void)
{
    sim_t s;
    sim_init(&s, 1.2f);
    sim_run(&s, 1000, 0.0f, 0.0f, true);
    sim_run(&s, 2000, 0.0f, 0.7f, true);  // carried over a 0.7 m table at constant height
    CHECK(s.steps == 1 && fabsf(s.out.h_terrain - 0.7f) < 0.03f && fabsf(s.out.z - 1.2f) < 0.03f,
          "table: steps %d h %.3f z %.3f", s.steps, (double)s.out.h_terrain, (double)s.out.z);
    sim_run(&s, 2000, 0.0f, 0.0f, true);  // off the table again
    CHECK(s.steps == 2 && s.out.h_terrain == 0.0f && fabsf(s.out.z - 1.2f) < 0.03f,
          "off the table: steps %d h %.3f z %.3f", s.steps, (double)s.out.h_terrain, (double)s.out.z);
}

// 2026-11-16 16:02 hand test: crossing an edge, the ToF moved 0.95 -> 0.67 m over ~0.1 s (~3 cm per frame), never
// a frame-to-frame jump above step_th: the step became a z resync. Ramps over 50-200 ms, both ways, step_th 0.15.
static void test_gradual_edge(void)
{
    static const alt_cfg_t c15 = {.sigma_a = 0.3f, .sigma_tof = 0.05f, .step_th = 0.15f, .sigma_baro = 0.3f,
                                  .baro_max_ms = 3000u};
    for (uint32_t ramp_ms = 50u; ramp_ms <= 200u; ramp_ms += 50u) {
        sim_t s;
        sim_init(&s, 0.95f);
        s.cfg = &c15;
        sim_run(&s, 1000, 0.0f, 0.0f, true);
        uint32_t n = ramp_ms / 10u;
        for (uint32_t i = 1; i <= n; i++) {
            sim_run(&s, 10, 0.0f, 0.28f * (float)i / (float)n, true);
        }
        sim_run(&s, 1000, 0.0f, 0.28f, true);
        CHECK(s.steps == 1 && s.resyncs == 0 && fabsf(s.out.h_terrain - 0.28f) < 0.04f &&
                  fabsf(s.out.z - 0.95f) < 0.04f,
              "edge over %lu ms: steps %d resyncs %d h %.3f z %.3f", (unsigned long)ramp_ms, s.steps, s.resyncs,
              (double)s.out.h_terrain, (double)s.out.z);
        for (uint32_t i = 1; i <= n; i++) {
            sim_run(&s, 10, 0.0f, 0.28f * (1.0f - (float)i / (float)n), true);
        }
        sim_run(&s, 1000, 0.0f, 0.0f, true);
        CHECK(s.steps == 2 && s.resyncs == 0 && s.out.h_terrain == 0.0f && fabsf(s.out.z - 0.95f) < 0.04f,
              "off the edge over %lu ms: steps %d resyncs %d h %.3f z %.3f", (unsigned long)ramp_ms, s.steps,
              s.resyncs, (double)s.out.h_terrain, (double)s.out.z);
    }
}

static void test_spike_ignored(void)
{
    sim_t s;
    sim_init(&s, 1.0f);
    sim_run(&s, 1000, 0.0f, 0.0f, true);
    sim_run(&s, 10, 0.0f, 0.5f, true);  // one bad reading
    sim_run(&s, 1000, 0.0f, 0.0f, true);
    CHECK(s.steps == 0 && s.noise_evs == 1 && s.out.h_terrain == 0.0f && fabsf(s.out.z - 1.0f) < 0.03f,
          "spike: steps %d noise %d h %.3f z %.3f", s.steps, s.noise_evs, (double)s.out.h_terrain, (double)s.out.z);
}

// 2026-11-16 Alt Hold logs (alt_step_th 0, as flown): ToF spikes of +9..+25 cm for < 60 ms passed the 3 sigma
// gate and pushed z up 1-2 cm each. Held back now (ALT_SPIKE_*); a lasting change is still followed.
static void test_short_spikes_flat_floor(void)
{
    static const alt_cfg_t flat = {.sigma_a = 0.3f, .sigma_tof = 0.05f, .step_th = 0.0f, .sigma_baro = 0.3f,
                                   .baro_max_ms = 3000u};
    sim_t s;
    sim_init(&s, 0.8f);
    s.cfg = &flat;
    s.armed = true;
    sim_run(&s, 1000, 0.0f, 0.0f, true);
    float worst = 0.0f;
    for (int i = 0; i < 10; i++) {
        float h = (i % 2) ? 0.12f : 0.25f;
        sim_run(&s, 10u * (1u + (uint32_t)i % 5u), 0.0f, -h, true);  // 1..5 frames reading h too far
        for (int k = 0; k < 50; k++) {
            sim_run(&s, 10, 0.0f, 0.0f, true);
            worst = fmaxf(worst, fabsf(s.out.z - s.z));
        }
    }
    CHECK(worst < 0.012f && s.resyncs == 0 && s.steps == 0, "spikes moved z: worst %.3f resyncs %d", (double)worst,
          s.resyncs);
    sim_run(&s, 2000, 0.0f, 0.10f, true);  // a lasting 10 cm change (steps off: z follows the ToF)
    CHECK(fabsf(s.out.z - (s.z - 0.10f)) < 0.02f, "lasting change not followed: z %.3f (ToF %.3f)", (double)s.out.z,
          (double)(s.z - 0.10f));
}

static void test_tof_loss(void)
{
    sim_t s;
    sim_init(&s, 0.8f);
    sim_run(&s, 1000, 0.0f, 0.0f, true);
    sim_run(&s, 400, 0.0f, 0.0f, false);
    CHECK(s.out.valid && s.out.src == ALT_SRC_PREDICT, "invalid before 0.5 s of ToF loss");
    sim_run(&s, 200, 0.0f, 0.0f, false);
    CHECK(!s.out.valid, "still valid after 0.6 s without ToF");
    sim_run(&s, 1000, 0.0f, 0.0f, true);
    CHECK(s.out.valid && fabsf(s.out.z - 0.8f) < 0.05f && fabsf(s.out.h_terrain) < 0.05f,
          "ToF back: valid %d z %.3f h %.3f", s.out.valid, (double)s.out.z, (double)s.out.h_terrain);
}

static void test_tilt_rejected(void)
{
    sim_t s;
    sim_init(&s, 1.0f);
    sim_run(&s, 1000, 0.0f, 0.0f, true);
    alt_in_t in = {.now_ms = s.now_ms + 10u, .dt = 0.01f, .tof_new = true, .tof_in_range = true,
                   .tof_dist = 0.2f, .roll = 40.0f * MATHX_DEG2RAD};
    alt_est_step(&s.e, &cfg, &in, &s.out);
    CHECK(s.out.src == ALT_SRC_PREDICT && fabsf(s.out.z - 1.0f) < 0.03f, "ToF used at 40 deg tilt");
}

// 2026-10-28 logs: with the motors on, a_up was biased ~-1 m/s^2; z ran away and h_terrain followed to -40 m.
static void test_accel_bias(void)
{
    sim_t s;
    sim_init(&s, 0.5f);
    s.armed = true;
    sim_run(&s, 1000, 0.0f, 0.0f, true);
    s.acc_bias = -2.0f;
    float worst = 0.0f;
    for (int i = 0; i < 500; i++) {
        sim_run(&s, 10, 0.0f, 0.0f, true);
        worst = fmaxf(worst, fabsf(s.out.z - s.z));
    }
    CHECK(worst < 0.5f && s.out.h_terrain == 0.0f, "accel bias: worst z error %.3f h %.3f", (double)worst,
          (double)s.out.h_terrain);
}

static void test_tilted_jump_not_a_step(void)
{
    sim_t s;
    sim_init(&s, 1.0f);
    sim_run(&s, 1000, 0.0f, 0.0f, true);
    s.roll = 20.0f * MATHX_DEG2RAD;
    sim_run(&s, 300, 0.0f, 0.4f, true);  // tilted beam hits something 0.4 m up (a leg, a chair)
    s.roll = 0.0f;
    sim_run(&s, 1000, 0.0f, 0.0f, true);
    CHECK(s.steps == 0 && s.out.h_terrain == 0.0f && fabsf(s.out.z - 1.0f) < 0.05f,
          "tilted jump: steps %d h %.3f z %.3f", s.steps, (double)s.out.h_terrain, (double)s.out.z);
}

// 2026-10-29 hand test: pumped 0.55-1.1 m at ~1 Hz (±10 m/s^2), nose down to 40 deg at each low point
// (ToF unusable there). Small z errors after each gap went into h_terrain (8-17 cm).
static void test_hand_pumping(void)
{
    sim_t s;
    sim_init(&s, 0.8f);
    sim_run(&s, 1000, 0.0f, 0.0f, true);
    float worst = 0.0f;
    for (int i = 0; i < 1000; i++) {  // 10 s
        float ph = 2.0f * MATHX_PI * (float)i * 0.01f;
        float acc = 0.27f * 4.0f * MATHX_PI * MATHX_PI * cosf(ph);  // z = 0.8 - 0.27 cos(ph)
        s.roll = 20.0f * MATHX_DEG2RAD * (1.0f + cosf(ph));         // 40 deg at the bottom, 0 at the top
        sim_run(&s, 10, acc, 0.0f, true);
        if (cosf(s.roll) > 0.97f) {
            worst = fmaxf(worst, fabsf(s.out.z - s.z));
        }
    }
    CHECK(s.steps == 0 && s.out.h_terrain == 0.0f && worst < 0.1f, "pumping: steps %d h %.3f worst z err %.3f",
          s.steps, (double)s.out.h_terrain, (double)worst);
}

static void test_back_on_floor(void)
{
    sim_t s;
    sim_init(&s, 0.13f);
    sim_run(&s, 1000, 0.0f, 0.0f, true);
    s.e.h_terrain = -0.2f;  // left over from steps misdetected while carried
    s.e.kf.z = 0.13f - 0.2f;
    sim_run(&s, 1000, 0.0f, 0.0f, true);
    CHECK(s.grounds == 1 && s.out.h_terrain == 0.0f && fabsf(s.out.z - 0.13f) < 0.02f,
          "back on the floor: grounds %d h %.3f z %.3f", s.grounds, (double)s.out.h_terrain, (double)s.out.z);
    sim_run(&s, 1000, 0.0f, 0.0f, true);
    CHECK(s.grounds == 1, "ground event repeated");
}

static void test_arm_rereference(void)
{
    sim_t s;
    sim_init(&s, 0.13f);
    sim_run(&s, 1000, 0.0f, 0.0f, true);
    s.e.h_terrain = 0.5f;
    s.armed = true;
    sim_run(&s, 100, 0.0f, 0.0f, true);
    CHECK(s.arms == 1 && s.out.h_terrain == 0.0f && fabsf(s.out.z - 0.13f) < 0.02f, "arm: arms %d h %.3f z %.3f",
          s.arms, (double)s.out.h_terrain, (double)s.out.z);
    s.e.h_terrain = 0.3f;  // armed: no ground reset
    sim_run(&s, 1000, 0.0f, 0.0f, true);
    CHECK(s.arms == 1 && s.grounds == 0 && s.out.h_terrain == 0.3f, "armed: h changed to %.3f",
          (double)s.out.h_terrain);
}

// M10b: with the baro the ToF may drop out for a few seconds at height (here while descending), z stays usable,
// and the returning ToF continues the estimate instead of restarting it.
static void test_baro_bridges_tof_loss(void)
{
    sim_t s;
    sim_init(&s, 0.13f);
    s.baro = true;
    s.baro_offset = 0.40f;  // baro relative to its own boot reference: an arbitrary constant offset
    sim_run(&s, 500, 0.0f, 0.0f, true);  // still on the floor: baro_ref taken
    CHECK(s.e.baro_ref_ok && fabsf(s.e.baro_ref - 0.40f) < 0.08f, "baro_ref %.3f", (double)s.e.baro_ref);
    s.vz = 0.5f;
    sim_run(&s, 2000, 0.0f, 0.0f, true);  // climb to 1.13 m, 6 s of learning at height
    s.vz = 0.0f;
    sim_run(&s, 6000, 0.0f, 0.0f, true);
    s.vz = -0.2f;                          // descending, then the ToF is lost for 3 s
    float worst = 0.0f;
    bool valid_all = true;
    for (int i = 0; i < 280; i++) {
        sim_run(&s, 10, 0.0f, 0.0f, false);
        worst = fmaxf(worst, fabsf(s.out.z - s.z));
        valid_all = valid_all && s.out.valid;
    }
    // PLAN M10b accepts +-0.3 m on the baro alone
    CHECK(valid_all && s.out.src == ALT_SRC_BARO && worst < 0.25f, "baro bridge: valid %d src %u worst %.3f",
          valid_all, (unsigned)s.out.src, (double)worst);
    int inits = s.inits;
    sim_run(&s, 300, 0.0f, 0.0f, true);  // ToF back
    CHECK(s.inits == inits && fabsf(s.out.z - s.z) < 0.08f && s.out.src == ALT_SRC_TOF,
          "ToF back: inits %d->%d z err %.3f", inits, s.inits, (double)(s.out.z - s.z));
    // Beyond baro_max_ms: not valid any more.
    s.vz = 0.0f;
    sim_run(&s, 3500, 0.0f, 0.0f, false);
    CHECK(!s.out.valid, "still valid after 3.5 s on the baro alone");
}

static void test_no_baro_low(void)
{
    sim_t s;
    sim_init(&s, 0.13f);
    s.baro = true;
    sim_run(&s, 500, 0.0f, 0.0f, true);
    s.vz = 0.2f;
    sim_run(&s, 1000, 0.0f, 0.0f, true);  // 0.33 m
    s.vz = 0.0f;
    sim_run(&s, 1000, 0.0f, 0.0f, false);
    CHECK(!s.out.valid && s.out.src != ALT_SRC_BARO, "baro used below 0.5 m (z %.2f)", (double)s.out.z);
}

void test_alt(void)
{
    test_baro_bridges_tof_loss();
    test_no_baro_low();
    test_accel_bias();
    test_tilted_jump_not_a_step();
    test_hand_pumping();
    test_back_on_floor();
    test_arm_rereference();
    test_tracks_climb();
    test_table();
    test_spike_ignored();
    test_gradual_edge();
    test_short_spikes_flat_floor();
    test_tof_loss();
    test_tilt_rejected();
}

// Host tests for App/lib/crc32.c, App/lib/numfmt.c and App/app/params.c (flash image round trip).
#include <math.h>
#include <string.h>

#include "app/blackbox.h"
#include "app/capture.h"
#include "app/params.h"
#include "lib/crc32.h"
#include "lib/mathx.h"
#include "lib/numfmt.h"
#include "test_common.h"

static void test_crc32(void)
{
    CHECK(crc32_update(0, "123456789", 9) == 0xCBF43926u, "CRC-32 check value");
    uint32_t c = crc32_update(0, "1234", 4);
    c = crc32_update(c, "56789", 5);
    CHECK(c == 0xCBF43926u, "CRC-32 incremental");
}

static void test_numfmt(void)
{
    static const struct { const char *s; float v; } ok[] = {
        {"0.05", 0.05f}, {"-12.5", -12.5f}, {"3", 3.0f}, {"+7.", 7.0f}, {".25", 0.25f},
        {"1e-3", 0.001f}, {"2.5E2", 250.0f}, {"0.000123", 0.000123f}, {"1000", 1000.0f},
    };
    for (size_t i = 0; i < sizeof ok / sizeof ok[0]; i++) {
        float v = -999.0f;
        bool good = numfmt_parse_float(ok[i].s, &v);
        CHECK(good && fabsf(v - ok[i].v) <= 1e-6f * fmaxf(1.0f, fabsf(ok[i].v)), "parse '%s' -> %g", ok[i].s, (double)v);
    }
    static const char *bad[] = {"", "-", "abc", "1.2.3", "1e", "5x", "nan", " 1"};
    for (size_t i = 0; i < sizeof bad / sizeof bad[0]; i++) {
        float v;
        CHECK(!numfmt_parse_float(bad[i], &v), "accepted '%s'", bad[i]);
    }
    char b[32];
    numfmt_float(b, sizeof b, 0.05f, 5);
    CHECK(strcmp(b, "0.05") == 0, "fmt 0.05 -> %s", b);
    numfmt_float(b, sizeof b, -12.5f, 5);
    CHECK(strcmp(b, "-12.5") == 0, "fmt -12.5 -> %s", b);
    numfmt_float(b, sizeof b, 1000.0f, 5);
    CHECK(strcmp(b, "1000") == 0, "fmt 1000 -> %s", b);
    numfmt_float(b, sizeof b, -0.000001f, 5);
    CHECK(strcmp(b, "0") == 0, "fmt tiny negative -> %s", b);
    numfmt_float(b, sizeof b, 0.00012f, 5);
    CHECK(strcmp(b, "0.00012") == 0, "fmt 0.00012 -> %s", b);
}

static void test_params(void)
{
    params_t p;
    params_defaults(&p);
    int i = params_find("rate_p_rp");
    CHECK(i >= 0 && params_get(&p, (size_t)i) == 0.05f, "default rate_p_rp");
    CHECK(params_find("no_such_param") < 0, "unknown name found");
    CHECK(params_set(&p, (size_t)i, 5.0f) == PARAM_ERR_RANGE, "out-of-range accepted");
    CHECK(params_set(&p, (size_t)i, NAN) == PARAM_ERR_RANGE, "NaN accepted");
    CHECK(params_set(&p, (size_t)i, 0.08f) == PARAM_OK && p.rate_p_rp == 0.08f, "set failed");
    for (size_t k = 0; k < params_count(); k++) {
        const param_desc_t *d = params_desc(k);
        CHECK(d->def >= d->min && d->def <= d->max, "%s default outside its range", d->name);
    }

    // Round trip through the flash image.
    uint8_t img[PARAMS_IMAGE_MAX];
    size_t n = params_image_build(&p, img, sizeof img);
    CHECK(n > 0, "image build");
    params_t q;
    params_defaults(&q);
    unsigned loaded = 0;
    CHECK(params_image_load(&q, img, n, &loaded) == PARAM_LOAD_OK && loaded == params_count(), "load %u", loaded);
    CHECK(memcmp(&p, &q, sizeof p) == 0, "round trip changed values");

    // Erased flash, corrupted CRC, truncated image.
    uint8_t erased[PARAMS_IMAGE_MAX];
    memset(erased, 0xFF, sizeof erased);
    CHECK(params_image_load(&q, erased, sizeof erased, &loaded) == PARAM_LOAD_EMPTY, "erased flash not EMPTY");
    img[20] ^= 0x01;
    params_defaults(&q);
    CHECK(params_image_load(&q, img, n, &loaded) == PARAM_LOAD_CORRUPT && q.rate_p_rp == 0.05f,
          "corrupted image accepted or partly applied");
    img[20] ^= 0x01;
    CHECK(params_image_load(&q, img, 10, &loaded) == PARAM_LOAD_CORRUPT, "truncated image accepted");

    // An out-of-range stored value is skipped (keeps default), the rest still load.
    params_t r;
    params_defaults(&r);
    float bad = 99.0f;
    memcpy((uint8_t *)&r + params_desc((size_t)i)->offset, &bad, sizeof bad);  // bypass params_set
    n = params_image_build(&r, img, sizeof img);
    params_defaults(&q);
    CHECK(params_image_load(&q, img, n, &loaded) == PARAM_LOAD_OK && loaded == params_count() - 1u &&
          q.rate_p_rp == 0.05f, "out-of-range stored value not skipped");
}

static void test_blackbox(void)
{
    blackbox_reset();
    CHECK(blackbox_count() == 0u && blackbox_get(0) == NULL, "not empty after reset");
    for (uint32_t t = 0; t < BB_RECORDS + 100u; t++) {
        bb_rec_t r = {.t_ms = t, .roll = (int16_t)-1234, .m_us = {900, 1000, 1500, 2000}};
        blackbox_add(&r);
    }
    CHECK(blackbox_count() == BB_RECORDS, "count %u", (unsigned)blackbox_count());
    CHECK(blackbox_get(0)->t_ms == 100u, "oldest kept %u (want 100)", (unsigned)blackbox_get(0)->t_ms);
    CHECK(blackbox_get(BB_RECORDS - 1u)->t_ms == BB_RECORDS + 99u, "newest wrong");
    char line[120];
    size_t n = blackbox_format(blackbox_get(0), line, sizeof line);
    CHECK(n > 0 && strncmp(line, "L,100,0,0,0,0,-1234,", 20) == 0, "format: %s", line);
    int commas_h = 0, commas_l = 0;
    for (const char *c = blackbox_header(); *c; c++) commas_h += (*c == ',');
    for (const char *c = line; *c; c++) commas_l += (*c == ',');
    CHECK(commas_h == commas_l, "header %d fields vs line %d", commas_h, commas_l);
}

static void test_capture(void)
{
    cap_in_t in = {.rate = {0}, .m_us = {900.0f, 1000.0f, 1500.0f, 2000.0f}};
    capture_add(&in, 943u);
    CHECK(capture_state() == CAP_IDLE, "capture records before CAPTURE");
    CHECK(!capture_arm(0u) && !capture_arm(CAP_DECIM_MAX + 1u), "bad decimation accepted");
    CHECK(capture_arm(3u) && !capture_arm(3u), "arm twice accepted");
    uint32_t cycles = 0;
    while (capture_state() == CAP_RECORDING && cycles < 10u * CAP_RECORDS) {
        // Roll rate alternates +10 / -10 / +40 deg/s within each record of 3 cycles: average = +40/3.
        static const float pattern[3] = {10.0f, -10.0f, 40.0f};
        in.rate[0] = pattern[cycles % 3u] * MATHX_DEG2RAD;
        in.sp[1] = (float)(cycles / 3u) * 0.1f * MATHX_DEG2RAD;  // record index, 0.1 deg/s units
        capture_add(&in, 943u);
        cycles++;
    }
    CHECK(cycles == 3u * CAP_RECORDS && capture_state() == CAP_DONE, "cycles %u state %d", (unsigned)cycles,
          capture_state());
    CHECK(capture_count() == CAP_RECORDS && capture_get(CAP_RECORDS) == NULL, "count / bounds");
    CHECK(capture_get(1)->dt_us == 3u * 943u, "dt %u", capture_get(1)->dt_us);
    CHECK(capture_get(5)->rate[0] == 133, "averaged rate %d (want 133 = 13.3 deg/s)", capture_get(5)->rate[0]);
    CHECK(capture_get(CAP_RECORDS - 1u)->sp[1] == (int16_t)(CAP_RECORDS - 1u), "order");
    CHECK(capture_get(0)->m_us[3] == 2000u, "motor us %u", capture_get(0)->m_us[3]);
    CHECK(!capture_arm(3u), "re-armed before the capture was sent");
    char line[120];
    size_t n = capture_format(1, capture_get(1), line, sizeof line);
    CHECK(n > 0 && strncmp(line, "C,1,2829,133,0,0,0,1,0,", 23) == 0, "format: %s", line);
    int commas_h = 0, commas_l = 0;
    for (const char *c = capture_header(); *c; c++) commas_h += (*c == ',');
    for (const char *c = line; *c; c++) commas_l += (*c == ',');
    CHECK(commas_h == commas_l, "header %d fields vs line %d", commas_h, commas_l);
    capture_release();
    CHECK(capture_state() == CAP_IDLE && capture_count() == 0u && capture_arm(CAP_DECIM_DEFAULT), "release");
    capture_release();
}

void test_params_all(void)
{
    test_capture();
    test_blackbox();
    test_crc32();
    test_numfmt();
    test_params();
}

// Host tests for App/lib (PLAN Â§12). Build/run: tests/run_tests.ps1
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "app/telem_proto.h"
#include "drivers/dps310.h"
#include "drivers/ist8310.h"
#include "drivers/mspv2.h"
#include "drivers/mtf01p.h"
#include "drivers/sbus.h"

#include "lib/filter.h"
#include "lib/kalman1d.h"
#include "lib/mathx.h"

#include "test_common.h"

int failures;

// Steady-state amplitude of a sine at freq_hz after the filter, relative to the input.
static double gain_pt1(float fc, float fs, float freq)
{
    pt1_t f;
    pt1_init(&f, fc, fs);
    double peak = 0.0;
    int n = (int)(fs * 2.0f);
    for (int i = 0; i < n; i++) {
        float y = pt1_apply(&f, sinf(2.0f * MATHX_PI * freq * (float)i / fs));
        if (i > n / 2 && fabs(y) > peak) {
            peak = fabs(y);
        }
    }
    return peak;
}

static double gain_biquad(float fc, float fs, float freq)
{
    biquad_t f;
    biquad_lpf_init(&f, fc, fs);
    double peak = 0.0;
    int n = (int)(fs * 2.0f);
    for (int i = 0; i < n; i++) {
        float y = biquad_apply(&f, sinf(2.0f * MATHX_PI * freq * (float)i / fs));
        if (i > n / 2 && fabs(y) > peak) {
            peak = fabs(y);
        }
    }
    return peak;
}

// Gain and phase lag (deg) of a notch at freq, from the response to a sine over the last second.
static void notch_response(float center, float q, float fs, float freq, double *gain, double *lag_deg)
{
    biquad_t f;
    biquad_notch_init(&f, center, q, fs);
    double si = 0.0, co = 0.0;
    int n = (int)(fs * 2.0f);
    for (int i = 0; i < n; i++) {
        double ph = 2.0 * 3.14159265358979 * (double)freq * i / (double)fs;
        float y = biquad_apply(&f, (float)sin(ph));
        if (i >= n / 2) {
            si += (double)y * sin(ph);
            co += (double)y * cos(ph);
        }
    }
    *gain = 2.0 * sqrt(si * si + co * co) / (n - n / 2);
    *lag_deg = -atan2(co, si) * 180.0 / 3.14159265358979;
}

static void test_notch(void)
{
    double g, lag;
    notch_response(85.0f, 2.5f, 3200.0f, 85.0f, &g, &lag);
    CHECK(20.0 * log10(g + 1e-9) < -30.0, "notch 85 Hz: gain at center %.1f dB (want < -30)", 20.0 * log10(g + 1e-9));
    notch_response(85.0f, 2.5f, 3200.0f, 85.0f + 0.5f * 85.0f / 2.5f, &g, &lag);
    CHECK(fabs(20.0 * log10(g) + 3.0) < 0.6, "notch -3 dB edge: %.2f dB", 20.0 * log10(g));
    notch_response(85.0f, 2.5f, 3200.0f, 10.0f, &g, &lag);
    CHECK(g > 0.98 && fabs(lag) < 6.0, "notch at 10 Hz: gain %.3f lag %.1f deg (control band must pass)", g, lag);
    notch_response(85.0f, 2.5f, 3200.0f, 300.0f, &g, &lag);
    CHECK(g > 0.95, "notch at 300 Hz: gain %.3f", g);

    // Firmware retunes in small steps (>= 0.5 Hz, every ~10 ms). A 2 Hz step on a constant input must
    // disturb the output by < 1 % (the state is kept, only the coefficients change).
    biquad_t f;
    biquad_notch_init(&f, 80.0f, 2.5f, 3200.0f);
    for (int i = 0; i < 100; i++) {
        (void)biquad_apply(&f, 1.5f);
    }
    biquad_notch_coeffs(&f, 82.0f, 2.5f, 3200.0f);
    float worst = 0.0f;
    for (int i = 0; i < 50; i++) {
        worst = fmaxf(worst, fabsf(biquad_apply(&f, 1.5f) - 1.5f));
    }
    CHECK(worst < 0.015f, "notch 2 Hz retune transient %.4f on a constant 1.5", (double)worst);
}

static size_t feed_all(mspv2_parser_t *p, const uint8_t *b, size_t n, mspv2_frame_t *last)
{
    size_t frames = 0;
    for (size_t i = 0; i < n; i++) {
        if (mspv2_feed(p, b[i])) {
            *last = p->frame;
            frames++;
        }
    }
    return frames;
}

static void test_mspv2(void)
{
    // Captured from the user's MTF-01P (FLOWRAW, 2026-10-28), CRC byte 0x45 included as sent.
    const uint8_t flow[] = {0x24, 0x58, 0x3C, 0x00, 0x02, 0x1F, 0x09, 0x00, 0x3E,
                            0xFE, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x45};
    mspv2_parser_t p;
    mspv2_frame_t f;
    mspv2_init(&p);
    CHECK(feed_all(&p, flow, sizeof flow, &f) == 1u && p.crc_errors == 0u, "captured frame: CRC (crc8_dvb_s2)");
    mtf01p_flow_t fl;
    CHECK(mtf01p_parse_flow(&f, &fl) && fl.quality == 0x3E && fl.motion_x == -2 && fl.motion_y == 1,
          "flow decode q %u x %ld y %ld", fl.quality, (long)fl.motion_x, (long)fl.motion_y);

    // Rangefinder frame built here (layout per PLAN Â§6.3): quality 200, 1234 mm; then -1 (out of range).
    uint8_t rng[] = {'$', 'X', '>', 0x00, 0x01, 0x1F, 0x05, 0x00, 200, 0xD2, 0x04, 0x00, 0x00, 0};
    uint8_t crc = 0;
    for (size_t i = 3; i < sizeof rng - 1u; i++) {
        crc = mspv2_crc8_dvb_s2(crc, rng[i]);
    }
    rng[sizeof rng - 1u] = crc;
    // Noise and a broken frame before it must not stop the parser from syncing.
    const uint8_t junk[] = {0x00, '$', 'X', 0x13, '$', 'X', '<', 0x00, 0x01, 0x1F, 0x05, 0x00, 1, 2, 3, 4, 5, 0x99};
    mspv2_init(&p);
    size_t n = feed_all(&p, junk, sizeof junk, &f);
    n += feed_all(&p, rng, sizeof rng, &f);
    mtf01p_range_t r;
    CHECK(n == 1u && p.crc_errors == 1u, "resync after junk: frames %u crc_err %u", (unsigned)n,
          (unsigned)p.crc_errors);
    CHECK(mtf01p_parse_range(&f, &r) && r.quality == 200u && r.distance_mm == 1234, "range %ld mm",
          (long)r.distance_mm);
    CHECK(!mtf01p_parse_flow(&f, &fl), "range frame parsed as flow");
}

static void test_i2c_sensors(void)
{
    CHECK(dps310_sign_extend(0x800u, 12) == -2048 && dps310_sign_extend(0x7FFu, 12) == 2047 &&
              dps310_sign_extend(0xFFFFFFu, 24) == -1 && dps310_sign_extend(0x80000u, 20) == -524288,
          "DPS310 sign extension");
    // Coefficients packed by hand per the PX4 layout: c0 = -2 (12 bit), c1 = 300 (12 bit), c00 = -123456 (20 bit),
    // c10 = 54321 (20 bit), then c01, c11, c20, c21, c30 = -1, 2, -3, 4, -5 (16 bit).
    uint32_t c0 = 0xFFEu, c1 = 300u, c00 = (uint32_t)(-123456) & 0xFFFFFu, c10 = 54321u;
    uint8_t r[18] = {
        (uint8_t)(c0 >> 4), (uint8_t)(((c0 & 0xFu) << 4) | (c1 >> 8)), (uint8_t)c1,
        (uint8_t)(c00 >> 12), (uint8_t)(c00 >> 4), (uint8_t)(((c00 & 0xFu) << 4) | (c10 >> 16)),
        (uint8_t)(c10 >> 8), (uint8_t)c10,
        0xFF, 0xFF, 0x00, 0x02, 0xFF, 0xFD, 0x00, 0x04, 0xFF, 0xFB,
    };
    dps310_coef_t c;
    dps310_parse_coef(r, &c);
    CHECK(c.c0 == -2 && c.c1 == 300 && c.c00 == -123456 && c.c10 == 54321 && c.c01 == -1 && c.c11 == 2 &&
              c.c20 == -3 && c.c21 == 4 && c.c30 == -5,
          "DPS310 coef parse: %ld %ld %ld %ld %ld %ld %ld %ld %ld", (long)c.c0, (long)c.c1, (long)c.c00,
          (long)c.c10, (long)c.c01, (long)c.c11, (long)c.c20, (long)c.c21, (long)c.c30);
    dps310_coef_t k = {.c0 = 50, .c1 = 0, .c00 = 100000, .c10 = 2000};
    float pa, tc;
    dps310_compensate(&k, 253952, 0, &pa, &tc);  // raw_p = 1.0 after the 16x scale
    CHECK(fabsf(pa - 102000.0f) < 0.5f && fabsf(tc - 25.0f) < 1e-4f, "DPS310 compensation %f Pa %f C", (double)pa,
          (double)tc);

    float f[3];
    const uint8_t not_ready[7] = {0x00, 1, 0, 2, 0, 3, 0};
    CHECK(!ist8310_decode(not_ready, f), "IST8310 decoded without DRDY");
    const uint8_t burst[7] = {0x01, 0x28, 0x05, 0xD8, 0xFA, 0x00, 0x80};  // x 1320, y -1320, z -32768
    CHECK(ist8310_decode(burst, f) && fabsf(f[0] - 100.0f) < 1e-3f && fabsf(f[1] + 100.0f) < 1e-3f &&
              fabsf(f[2] + 32768.0f * 100.0f / 1320.0f) < 1e-2f,
          "IST8310 decode %f %f %f uT", (double)f[0], (double)f[1], (double)f[2]);
}

static void test_filters(void)
{
    double db = 20.0 * log10(gain_biquad(150.0f, 3200.0f, 150.0f));
    CHECK(fabs(db + 3.01) < 0.3, "biquad 150 Hz @3200: gain at fc = %.2f dB (want -3.0)", db);
    db = 20.0 * log10(gain_biquad(150.0f, 3200.0f, 15.0f));
    CHECK(fabs(db) < 0.1, "biquad passband gain at 15 Hz = %.3f dB", db);

    // PT1 discretisation is only approximately -3 dB at fc when fc << fs.
    db = 20.0 * log10(gain_pt1(25.0f, 3200.0f, 25.0f));
    CHECK(fabs(db + 3.01) < 0.3, "pt1 25 Hz @3200: gain at fc = %.2f dB (want -3.0)", db);

    biquad_t b;
    biquad_lpf_init(&b, 150.0f, 3200.0f);
    float y = biquad_apply(&b, 9.8f);
    CHECK(fabsf(y - 9.8f) < 1e-4f, "biquad primed on first sample: %f", (double)y);
    pt1_t p;
    pt1_init(&p, 25.0f, 3200.0f);
    y = pt1_apply(&p, -3.0f);
    CHECK(fabsf(y + 3.0f) < 1e-6f, "pt1 primed on first sample: %f", (double)y);
}

static void test_kalman1d(void)
{
    const float sg = 4.0f * MATHX_DEG2RAD;
    const float sa = 3.0f * MATHX_DEG2RAD;
    const float dt = 0.004f;
    kalman1d_t kf;
    kalman1d_init(&kf, 0.0f, (2.0f * MATHX_DEG2RAD) * (2.0f * MATHX_DEG2RAD));

    // One step by hand: P = P0 + (dt*sg)^2, K = P / (P + sa^2).
    double P = pow(2.0 * (double)MATHX_DEG2RAD, 2) + pow((double)(dt * sg), 2);
    double K = P / (P + pow((double)sa, 2));
    kalman1d_predict(&kf, 0.0f, dt, sg);
    kalman1d_update(&kf, 0.1f, sa);
    CHECK(fabs((double)kf.angle - K * 0.1) < 1e-6, "first update angle %f, hand %f", (double)kf.angle, K * 0.1);
    CHECK(fabs((double)kf.P - (1.0 - K) * P) < 1e-9, "first update P %g, hand %g", (double)kf.P, (1.0 - K) * P);

    // Constant measurement: converges to it; steady-state K solves K = (P+q)/(P+q+r), P = (1-K)(P+q).
    const float meas = 10.0f * MATHX_DEG2RAD;
    for (int i = 0; i < 5000; i++) {
        kalman1d_predict(&kf, 0.0f, dt, sg);
        kalman1d_update(&kf, meas, sa);
    }
    CHECK(fabsf(kf.angle - meas) < 1e-4f, "converged angle %f want %f", (double)kf.angle, (double)meas);
    double q = pow((double)(dt * sg), 2), r = pow((double)sa, 2);
    double Pm = (-q + sqrt(q * q + 4.0 * q * r)) / 2.0;  // steady-state posterior P: P^2 + qP - qr = 0
    CHECK(fabs((double)kf.P - Pm) / Pm < 1e-3, "steady-state P %g, analytic %g", (double)kf.P, Pm);

    // Gyro-only: a constant rate integrates exactly.
    kalman1d_init(&kf, 0.0f, 0.0f);
    for (int i = 0; i < 250; i++) {
        kalman1d_predict(&kf, 1.0f, dt, sg);
    }
    CHECK(fabsf(kf.angle - 1.0f) < 1e-4f, "integrated angle %f want 1.0", (double)kf.angle);

    // Wrapped update across +-pi.
    kalman1d_init(&kf, 3.1f, 1.0f);
    kalman1d_update_wrapped(&kf, -3.1f, 0.01f);
    CHECK(fabsf(fabsf(kf.angle) - 3.1f) < 0.01f || fabsf(kf.angle) > 3.1f,
          "wrapped update went the short way: %f", (double)kf.angle);
}

static void test_telem_line(void)
{
    telem_frame_t f = {0};
    f.t_ms = 1234;
    f.arm = 1;
    f.roll = -0.05f * MATHX_DEG2RAD;  // -0.05 deg must keep its sign
    f.pitch = 12.346f * MATHX_DEG2RAD;
    f.p = -1.0f * MATHX_DEG2RAD;
    f.z = 0.0004f;                    // rounds to 0.000, no "-0.000"
    f.vz = -0.0004f;
    f.motor[3] = 1.0f;
    f.ctrl_us_max = 7;
    f.alt_src = 2;
    f.p_sp = -150.0f * MATHX_DEG2RAD;
    f.acro = 1;
    char line[TELEM_LINE_MAX];
    size_t n = telem_proto_build_T(line, sizeof line, &f);
    const char *want = "T,1234,1,0,0,-0.05,12.35,0.00,-1.0,0.0,0.0,0.00,0.00,0.0,0.000,0.000,0.000,0.000,0.000,"
                       "0.000,0.000,0.000,1.000,7,0,0.000,0.000,2,-150.0,0.0,1,0,0.000,0.000,0.000,0.000,0,0\r\n";
    CHECK(n == strlen(want) && strcmp(line, want) == 0, "T line\n got: %s\nwant: %s", line, want);
    int commas = 0;
    for (size_t i = 0; i < n; i++) {
        commas += (line[i] == ',');
    }
    CHECK(commas == 37, "T line has %d fields after 'T' (want 37)", commas);
}

// Packs channels LSB-first the same way an SBUS receiver does, independently of the decoder's loop.
static void sbus_pack(const uint16_t ch[16], uint8_t flags, uint8_t end, uint8_t f[25])
{
    memset(f, 0, 25);
    f[0] = 0x0F;
    for (int c = 0; c < 16; c++) {
        for (int b = 0; b < 11; b++) {
            if (ch[c] & (1u << b)) {
                int bit = c * 11 + b;
                f[1 + bit / 8] |= (uint8_t)(1u << (bit % 8));
            }
        }
    }
    f[23] = flags;
    f[24] = end;
}

static void test_sbus(void)
{
    uint16_t ch[16];
    for (int c = 0; c < 16; c++) {
        ch[c] = (uint16_t)((c * 131 + 172) & 0x7FF);
    }
    ch[0] = 0;
    ch[15] = 2047;
    uint8_t f[25];
    sbus_frame_t out;
    sbus_pack(ch, 0x00, 0x00, f);
    CHECK(sbus_decode(f, &out), "valid frame rejected");
    for (int c = 0; c < 16; c++) {
        CHECK(out.ch[c] == ch[c], "ch%d decoded %u want %u", c + 1, out.ch[c], ch[c]);
    }
    CHECK(!out.failsafe && !out.frame_lost, "flags should be clear");

    sbus_pack(ch, 0x0C, 0x14, f);  // lost + failsafe, SBUS2 end byte
    CHECK(sbus_decode(f, &out) && out.failsafe && out.frame_lost, "flags / SBUS2 end byte");

    sbus_pack(ch, 0x00, 0x00, f);
    f[0] = 0x0E;
    CHECK(!sbus_decode(f, &out), "bad header accepted");
    sbus_pack(ch, 0x00, 0x55, f);
    CHECK(!sbus_decode(f, &out), "bad end byte accepted");
}

int main(void)
{
    test_sbus();
    test_filters();
    test_notch();
    test_mspv2();
    test_i2c_sensors();
    test_telem_line();
    test_kalman1d();
    test_arming();
    test_ctrl();
    test_params_all();
    test_alt();
    test_alt_hold();
    test_levelcal();
    test_yaw_hold();
    test_flow_est();
    test_pos_hold();
    test_ubx();
    if (failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("%d FAILURE(S)\n", failures);
    return 1;
}

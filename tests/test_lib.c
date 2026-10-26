// Host tests for App/math_algorithm (PLAN §12). Build/run: tests/run_tests.ps1
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "control_logic/telem_proto.h"
#include "sensor_and_radio_driver/sbus.h"

#include "math_algorithm/filter.h"
#include "math_algorithm/kalman1d.h"
#include "math_algorithm/mathx.h"

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
    char line[TELEM_LINE_MAX];
    size_t n = telem_proto_build_T(line, sizeof line, &f);
    const char *want = "T,1234,1,0,0,-0.05,12.35,0.00,-1.0,0.0,0.0,0.00,0.00,0.0,0.000,0.000,0.000,0.000,0.000,"
                       "0.000,0.000,0.000,1.000,7,0,0.000,0.000,2\r\n";
    CHECK(n == strlen(want) && strcmp(line, want) == 0, "T line\n got: %s\nwant: %s", line, want);
    int commas = 0;
    for (size_t i = 0; i < n; i++) {
        commas += (line[i] == ',');
    }
    CHECK(commas == 27, "T line has %d fields after 'T' (want 27)", commas);
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
    test_telem_line();
    test_kalman1d();
    test_arming();
    test_ctrl();
    test_params_all();
    if (failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("%d FAILURE(S)\n", failures);
    return 1;
}

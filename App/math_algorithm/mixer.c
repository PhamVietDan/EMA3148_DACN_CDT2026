// Quad X mixer (PLAN §7.6, PX4/ArduPilot motor order) with desaturation that keeps roll/pitch over throttle.
#include "math_algorithm/mixer.h"

// M1 front-right CCW, M2 rear-left CCW, M3 front-left CW, M4 rear-right CW (viewed from above).
// Roll +: left motors up (right wing goes down). Pitch +: front motors up (nose up).
// Yaw +: CCW motors up; their reaction torque turns the frame clockwise.
static const float k_roll[4] = {-1.0f, 1.0f, 1.0f, -1.0f};
static const float k_pitch[4] = {1.0f, -1.0f, 1.0f, -1.0f};
static const float k_yaw[4] = {1.0f, 1.0f, -1.0f, -1.0f};

static float spread(const float v[4])
{
    float lo = v[0], hi = v[0];
    for (int i = 1; i < 4; i++) {
        lo = (v[i] < lo) ? v[i] : lo;
        hi = (v[i] > hi) ? v[i] : hi;
    }
    return hi - lo;
}

static void mark(mixer_sat_t *sat, int axis, float cmd)
{
    if (cmd > 0.0f) {
        sat->pos[axis] = true;
    } else if (cmd < 0.0f) {
        sat->neg[axis] = true;
    }
}

void mixer_quadx(float thr, const float torque[3], float idle, float motor[4], mixer_sat_t *sat)
{
    *sat = (mixer_sat_t){0};
    const float avail = 1.0f - idle;  // usable output span
    float roll = torque[0], pitch = torque[1], yaw = torque[2];

    // 1) Roll/pitch alone must fit the span; scale them down together if not.
    float rp[4];
    for (int i = 0; i < 4; i++) {
        rp[i] = roll * k_roll[i] + pitch * k_pitch[i];
    }
    float rp_spread = spread(rp);
    if (rp_spread > avail) {
        float k = avail / rp_spread;
        roll *= k;
        pitch *= k;
        for (int i = 0; i < 4; i++) {
            rp[i] *= k;
        }
        mark(sat, 0, torque[0]);
        mark(sat, 1, torque[1]);
    }

    // 2) Yaw gets what span is left (lowest priority). Spread grows monotonically with |yaw|, so bisect.
    float all[4];
    for (int i = 0; i < 4; i++) {
        all[i] = rp[i] + yaw * k_yaw[i];
    }
    if (spread(all) > avail) {
        float lo = 0.0f, hi = 1.0f;
        for (int it = 0; it < 12; it++) {
            float mid = 0.5f * (lo + hi);
            float t[4];
            for (int i = 0; i < 4; i++) {
                t[i] = rp[i] + mid * yaw * k_yaw[i];
            }
            if (spread(t) > avail) {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        yaw *= lo;
        for (int i = 0; i < 4; i++) {
            all[i] = rp[i] + yaw * k_yaw[i];
        }
        mark(sat, 2, torque[2]);
    }

    // 3) Throttle is shifted (not the torques) so every motor stays in [idle, 1].
    float base = idle + avail * thr;
    float hi = all[0] + base, lo = all[0] + base;
    for (int i = 1; i < 4; i++) {
        float v = all[i] + base;
        hi = (v > hi) ? v : hi;
        lo = (v < lo) ? v : lo;
    }
    if (hi > 1.0f) {
        base -= hi - 1.0f;
    } else if (lo < idle) {
        base += idle - lo;
    }
    for (int i = 0; i < 4; i++) {
        float v = all[i] + base;
        motor[i] = (v < idle) ? idle : (v > 1.0f) ? 1.0f : v;  // guards float rounding only
    }
}

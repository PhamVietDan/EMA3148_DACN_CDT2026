// Horizontal velocity from the MTF-01P optical flow (position-hold step 2, observation only).
// Calibrated 2026-11-10 (PROGRESS "hiệu chuẩn flow"): motion_x follows the roll rate, motion_y the pitch rate,
// k = -4.8 counts per rad/s, flow 20 ms (2 frames) behind the gyro, and
//     v_fwd   = +h * (motion_y / k - q),    v_right = -h * (motion_x / k - p)
// with h the ToF distance (as during the calibration). motion_x/y above are in that reference mount; a sensor
// turned since (flow_rot, quarter turns) is mapped back first: 1 (90): (-my, mx), 2 (180): (-mx, -my),
// 3 (270): (my, -mx) (hand tests 2026-11-14: logs 16:51 = 90, 17:27 = 180). One Kalman filter per axis in the
// heading frame (level, yaw-aligned), states velocity and accelerometer bias: predicted with the horizontal
// acceleration minus the bias, rotated by the yaw rate, updated with the flow velocity against the estimate of
// 2 frames ago (delayed measurement). The bias state was added 2026-11-16: in flight the right axis read a
// steady -0.1 m/s against the flow alone (log 11:49, FLOWLOG in flight, correlation 0.98), i.e. ~0.6 m/s^2 of
// accel bias that a velocity-only filter turns into a constant drift held by position hold. Same idea as the
// accel bias states of PX4 EKF2 / ArduPilot EKF (ideas only, no code). Position integrated in the room frame
// (gyro yaw) for checking only. Pure C (host-tested); task_slow runs it.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define FLOW_LAG_FRAMES  2u      // flow vs gyro, from the calibration (20 ms at 100 Hz)
#define FLOW_MIN_H       0.20f   // m: below this the ToF / flow are unreliable (s450drone used 0.26)
#define FLOW_TILT_MAX    0.8660254f  // cos(30 deg), as the ToF (PLAN §6.3)
#define FLOW_VALID_MS    200u    // no accepted flow frame this long: not valid
#define FLOW_RESEED_MS   1000u   // ... this long: the next usable frame restarts the estimate
#define FLOW_SIGMA_A     1.0f    // m/s^2: horizontal accel error (attitude error x g, vibration)
#define FLOW_GATE_SQ     16.0f   // innovation^2 / S beyond this: frame dropped (4 sigma)
#define FLOW_P0          1.0f    // (m/s)^2
// m/s^2 per sqrt(s): accel bias random walk. 0.05 (bias time constant ~2 s) followed the 0.2-0.3 Hz roll sway of
// the 2026-11-16 12:10 flight (learnt -1.5..-4.1 deg between hold-offs); 0.005 gives ~20 s: only the steady part.
#define FLOW_SIGMA_B     0.005f
#define FLOW_B0          0.5f    // m/s^2: initial bias sigma (~3 deg of attitude error x g)
#define FLOW_B_MAX       1.5f    // m/s^2: bias clamp (~9 deg); beyond this something else is wrong

typedef struct {
    float k;          // counts per rad/s (signed)
    float scale;      // velocity scale (1 = calibration)
    float q_min;      // minimum sensor quality (0..255)
    float sigma;      // rad/s: flow noise; the velocity noise is sigma * h
    uint8_t rot;      // sensor mount, quarter turns from the 2026-11-10 reference (0..3)
} flow_cfg_t;

typedef struct {
    int32_t mx, my;
    uint8_t quality;
    float p, q;       // rad/s, body rates when the frame arrived (bias removed)
    float h;          // m, ToF distance; < 0: no valid range
    float tilt_cos;   // cos(roll) * cos(pitch)
} flow_frame_t;

typedef struct {
    float v[2];       // m/s, heading frame: forward, right
    float b[2];       // m/s^2, accelerometer bias in the same frame (subtracted from acc in predict)
    float P[2][3];    // per axis: var(v), cov(v, b), var(b)
    float gyro_hist[FLOW_LAG_FRAMES + 1u][2];  // [0] newest
    float v_hist[FLOW_LAG_FRAMES + 1u][2];
    uint32_t frames;
    uint32_t last_ok_ms;
    float coast_s;    // s of predicts since the last used frame; > FLOW_RESEED_MS: v held at 0, pos frozen
    bool init;
    float pos[2];     // m, room frame (north/east of the gyro yaw at boot), checking only
    float meas[2];    // m/s, last flow velocity (heading frame), for logs
} flow_est_t;

void flow_est_init(flow_est_t *e);
// Every task_slow cycle. acc: horizontal acceleration in the heading frame (fwd, right), m/s^2; r: yaw rate
// (rad/s); yaw (rad) for the position.
void flow_est_predict(flow_est_t *e, const float acc[2], float r, float yaw, float dt);
// Every optical flow frame. Returns true when the frame was used.
bool flow_est_frame(flow_est_t *e, const flow_cfg_t *c, const flow_frame_t *f, uint32_t now_ms);
bool flow_est_valid(const flow_est_t *e, uint32_t now_ms);
void flow_est_zero_position(flow_est_t *e);

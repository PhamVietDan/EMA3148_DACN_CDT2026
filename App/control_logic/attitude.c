// Attitude estimate: one Kalman 1D per axis (PLAN §7.3). Roll/pitch corrected by accel; yaw gyro-only until M11.
#include "control_logic/attitude.h"

#include <math.h>

#include "control_logic/config.h"
#include "math_algorithm/kalman1d.h"
#include "math_algorithm/mathx.h"

static kalman1d_t kf_roll, kf_pitch, kf_yaw;
static float sigma_gyro = 4.0f * MATHX_DEG2RAD;  // overwritten from the parameter table at start-up
static float sigma_acc = 3.0f * MATHX_DEG2RAD;

void attitude_set_sigmas(float sigma_gyro_rad_s, float sigma_acc_rad)
{
    sigma_gyro = sigma_gyro_rad_s;
    sigma_acc = sigma_acc_rad;
}

void attitude_acc_angles(const float acc[3], float *roll, float *pitch)
{
    *roll = atan2f(-acc[1], -acc[2]);
    *pitch = atan2f(acc[0], sqrtf(acc[1] * acc[1] + acc[2] * acc[2]));
}

float attitude_acc_up(const float acc[3], float roll, float pitch)
{
    float sr = sinf(roll), cr = cosf(roll);
    float sp = sinf(pitch), cp = cosf(pitch);
    return sp * acc[0] - sr * cp * acc[1] - cr * cp * acc[2] - MATHX_G;
}

void attitude_init(const float acc[3])
{
    float roll, pitch;
    attitude_acc_angles(acc, &roll, &pitch);
    kalman1d_init(&kf_roll, roll, CFG_KF_P0);
    kalman1d_init(&kf_pitch, pitch, CFG_KF_P0);
    kalman1d_init(&kf_yaw, 0.0f, CFG_KF_P0);
}

void attitude_update(attitude_t *out, const float rate[3], const float acc[3], float dt)
{
    float roll_rate = rate[0], pitch_rate = rate[1], yaw_rate = rate[2];
#if CFG_KF_EULER_RATES
    // Body rates -> Euler angle rates (PLAN §7.3, optional).
    float sr = sinf(kf_roll.angle), cr = cosf(kf_roll.angle);
    float cp = cosf(kf_pitch.angle), tp = tanf(kf_pitch.angle);
    roll_rate = rate[0] + sr * tp * rate[1] + cr * tp * rate[2];
    pitch_rate = cr * rate[1] - sr * rate[2];
    yaw_rate = (sr * rate[1] + cr * rate[2]) / cp;
#endif
    kalman1d_predict(&kf_roll, roll_rate, dt, sigma_gyro);
    kalman1d_predict(&kf_pitch, pitch_rate, dt, sigma_gyro);
    kalman1d_predict(&kf_yaw, yaw_rate, dt, sigma_gyro);
    kf_yaw.angle = wrap_pi(kf_yaw.angle);

    // Accel is a valid gravity reference only when not accelerating much (Betaflight uses the same band).
    float norm_g = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]) / MATHX_G;
    if (norm_g > CFG_KF_ACC_MIN_G && norm_g < CFG_KF_ACC_MAX_G) {
        float roll_acc, pitch_acc;
        attitude_acc_angles(acc, &roll_acc, &pitch_acc);
        kalman1d_update(&kf_roll, roll_acc, sigma_acc);
        kalman1d_update(&kf_pitch, pitch_acc, sigma_acc);
    }

    out->roll = kf_roll.angle;
    out->pitch = kf_pitch.angle;
    out->yaw = kf_yaw.angle;
    out->P[0] = kf_roll.P;
    out->P[1] = kf_pitch.P;
    out->P[2] = kf_yaw.P;
    for (int i = 0; i < 3; i++) {
        out->rate[i] = rate[i];
    }
}

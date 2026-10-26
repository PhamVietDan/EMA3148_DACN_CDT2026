// 1-state Kalman filter for one attitude angle (Carbon Aeronautics form, PLAN §7.3).
#include "math_algorithm/kalman1d.h"

#include "math_algorithm/mathx.h"

void kalman1d_init(kalman1d_t *kf, float angle0, float P0)
{
    kf->angle = angle0;
    kf->P = P0;
}

void kalman1d_predict(kalman1d_t *kf, float rate, float dt, float sigma_rate)
{
    kf->angle += dt * rate;
    float s = dt * sigma_rate;
    kf->P += s * s;
}

static void update_innovation(kalman1d_t *kf, float innovation, float sigma_meas)
{
    float K = kf->P / (kf->P + sigma_meas * sigma_meas);
    kf->angle += K * innovation;
    kf->P = (1.0f - K) * kf->P;  // (I - K*H) * P with H = 1
}

void kalman1d_update(kalman1d_t *kf, float meas, float sigma_meas)
{
    update_innovation(kf, meas - kf->angle, sigma_meas);
}

void kalman1d_update_wrapped(kalman1d_t *kf, float meas, float sigma_meas)
{
    update_innovation(kf, wrap_pi(meas - kf->angle), sigma_meas);
    kf->angle = wrap_pi(kf->angle);
}

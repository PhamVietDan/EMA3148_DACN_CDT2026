// Level calibration (see levelcal.h).
#include "app/levelcal.h"

#include <math.h>

void levelcal_start(levelcal_t *c)
{
    *c = (levelcal_t){.norm_min = 1e9f, .norm_max = 0.0f};
}

bool levelcal_add(levelcal_t *c, const float acc[3])
{
    float norm = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    c->norm_min = fminf(c->norm_min, norm);
    c->norm_max = fmaxf(c->norm_max, norm);
    for (int i = 0; i < 3; i++) {
        c->sum[i] += acc[i];
    }
    return ++c->n >= LEVELCAL_SAMPLES;
}

levelcal_result_t levelcal_finish(const levelcal_t *c, float *roll, float *pitch, bool *inverted)
{
    float f[3];
    for (int i = 0; i < 3; i++) {
        f[i] = c->sum[i] / (float)c->n;
    }
    // At rest upright the specific force is (0, 0, -g) in FRD; resting on the motor caps it points the other way.
    *inverted = f[2] > 0.0f;
    if (*inverted) {
        for (int i = 0; i < 3; i++) {
            f[i] = -f[i];
        }
    }
    // = attitude_acc_angles() (app/attitude.c, which needs the RTOS headers): the angles the estimator converges to.
    *roll = atan2f(-f[1], -f[2]);
    *pitch = atan2f(f[0], sqrtf(f[1] * f[1] + f[2] * f[2]));
    if (c->norm_max - c->norm_min > LEVELCAL_MOTION) {
        return LEVELCAL_MOVED;
    }
    if (fabsf(*roll) > LEVELCAL_MAX_TILT || fabsf(*pitch) > LEVELCAL_MAX_TILT) {
        return LEVELCAL_TILTED;
    }
    return LEVELCAL_OK;
}

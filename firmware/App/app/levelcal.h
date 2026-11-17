// Level calibration: the attitude the IMU reads when the prop plane is level, used as trim_roll/trim_pitch.
// Works upright or upside down resting on the four motor caps (= the prop plane): inverted on a level floor the
// measured specific force is exactly minus the upright-level one, so it is negated, whatever the flip axis or
// heading. Pure C (host-tested); task_slow feeds it at ~100 Hz.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define LEVELCAL_SAMPLES   200u   // ~2 s at 100 Hz
#define LEVELCAL_MOTION    0.3f   // m/s^2: |a| peak-to-peak above this = moved, result refused
#define LEVELCAL_MAX_TILT  0.1745329f  // 10 deg: beyond, not resting on a near-level surface

typedef struct {
    float sum[3];
    float norm_min, norm_max;
    uint32_t n;
} levelcal_t;

typedef enum { LEVELCAL_OK, LEVELCAL_MOVED, LEVELCAL_TILTED } levelcal_result_t;

void levelcal_start(levelcal_t *c);
bool levelcal_add(levelcal_t *c, const float acc[3]);  // body FRD, m/s^2; true when LEVELCAL_SAMPLES reached
// roll/pitch (rad): trim for a level prop plane; inverted: the drone was upside down.
levelcal_result_t levelcal_finish(const levelcal_t *c, float *roll, float *pitch, bool *inverted);

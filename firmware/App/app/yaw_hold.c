// Heading hold (see yaw_hold.h).
#include "app/yaw_hold.h"

#include <math.h>

#include "lib/mathx.h"

void yaw_hold_reset(yaw_hold_t *h)
{
    h->holding = false;
    h->target = 0.0f;
}

float yaw_hold_step(yaw_hold_t *h, float stick_rate, bool centred, float yaw, float yaw_rate, float kp)
{
    if (kp <= 0.0f || !centred) {
        h->holding = false;
        return stick_rate;
    }
    if (!h->holding) {
        // Capturing while still turning would pull the nose back to where the stick was released.
        if (fabsf(yaw_rate) > YAW_HOLD_CAPTURE_RATE) {
            return 0.0f;
        }
        h->holding = true;
        h->target = yaw;
    }
    return clampf(kp * wrap_pi(h->target - yaw), -YAW_HOLD_MAX_RATE, YAW_HOLD_MAX_RATE);
}

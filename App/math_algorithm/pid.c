// Rate PID (PLAN §7.5; structure as PX4 rate_control): D on measurement through a PT1, integrator clamped
// and frozen in the direction the mixer is saturated.
#include "math_algorithm/pid.h"

#include "math_algorithm/mathx.h"

void pid_init(pid_ctrl_t *p, float dterm_cutoff_hz, float loop_hz)
{
    p->integ = 0.0f;
    p->prev_meas_filt = 0.0f;
    p->primed = false;
    pt1_init(&p->dterm_lpf, dterm_cutoff_hz, loop_hz);
}

void pid_reset(pid_ctrl_t *p)
{
    p->integ = 0.0f;
    p->primed = false;
    p->dterm_lpf.primed = false;  // restart the D filter from the next measurement
}

float pid_update(pid_ctrl_t *p, const pid_gains_t *g, float sp, float meas, float dt, bool integrate,
                 bool sat_pos, bool sat_neg)
{
    float err = sp - meas;

    // D on the measurement, not the error: a setpoint step does not kick the output.
    float meas_filt = pt1_apply(&p->dterm_lpf, meas);
    float d = 0.0f;
    if (p->primed && dt > 0.0f) {
        d = -g->kd * (meas_filt - p->prev_meas_filt) / dt;
    }
    p->prev_meas_filt = meas_filt;
    p->primed = true;

    if (integrate) {
        float di = g->ki * err * dt;
        // Anti-windup (PX4): do not integrate further into a saturated direction.
        if ((di > 0.0f && sat_pos) || (di < 0.0f && sat_neg)) {
            di = 0.0f;
        }
        p->integ = clampf(p->integ + di, -g->i_limit, g->i_limit);
    }
    return g->kp * err + p->integ + d + g->kff * sp;
}

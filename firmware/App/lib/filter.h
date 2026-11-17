// Discrete low-pass filters (pure C, float). Coefficients are computed for a fixed sample rate.
#pragma once

#include <stdbool.h>

typedef struct {
    float k;      // smoothing factor in (0, 1]
    float state;
    bool primed;  // first sample seeds the state (no start-up transient from 0)
} pt1_t;

// Direct form II transposed, Butterworth (Q = 1/sqrt(2)).
typedef struct {
    float b0, b1, b2, a1, a2;
    float z1, z2;
    bool primed;
} biquad_t;

void  pt1_init(pt1_t *f, float cutoff_hz, float sample_hz);
float pt1_apply(pt1_t *f, float x);

void  biquad_lpf_init(biquad_t *f, float cutoff_hz, float sample_hz);
float biquad_apply(biquad_t *f, float x);

// Notch (band-stop), unity gain away from center_hz; -3 dB width = center_hz / q.
// biquad_notch_coeffs changes only the coefficients, keeping the state, so the center can move while running.
void  biquad_notch_init(biquad_t *f, float center_hz, float q, float sample_hz);
void  biquad_notch_coeffs(biquad_t *f, float center_hz, float q, float sample_hz);

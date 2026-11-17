// High-rate capture for rate-loop tuning: 1280 records of the control loop in RAM, sent to the GCS
// afterwards (telemetry and the black box are only 50 Hz). Each record is the average over `decim`
// task_ctrl cycles (~1060 Hz): decim 2 = 530 Hz / 2.4 s ... decim 6 = 177 Hz / 7.2 s. Averaging (not
// picking one cycle) keeps vibration above the record rate from folding down as fake slow motion.
// Writer: task_ctrl while RECORDING; reader: task_telem once DONE. The state hands the buffer over.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CAP_RECORDS     1280u  // x 40 B = 51 KB of SRAM
#define CAP_DECIM_MIN   1u
#define CAP_DECIM_MAX   8u
#define CAP_DECIM_DEFAULT 4u   // ~265 Hz, ~4.8 s

typedef enum { CAP_IDLE, CAP_RECORDING, CAP_DONE } cap_state_t;

// One control cycle, physical units: rad/s, normalized torque, [0,1], us.
typedef struct {
    float rate[3];   // p, q, r measured
    float sp[3];     // p, q, r setpoints of the rate loop
    float term[8];   // P, I, D roll | P, I, D pitch | P, I yaw
    float thr;
    float m_us[4];   // M1..M4 ESC pulse
} cap_in_t;

// Stored record: rates in 0.1 deg/s; PID terms in 1e-4 of the normalized torque; thr in 1e-3.
typedef struct {
    uint16_t dt_us;     // time covered by this record (decim cycles)
    int16_t rate[3];
    int16_t sp[3];
    int16_t term[8];
    uint16_t thr;
    uint16_t m_us[4];
} cap_rec_t;

cap_state_t capture_state(void);
bool capture_arm(uint32_t decim);                 // task_telem: IDLE -> RECORDING; false if busy or bad decim
uint32_t capture_decim(void);
void capture_add(const cap_in_t *in, uint32_t dt_us);  // task_ctrl, each cycle with a new IMU sample
size_t capture_count(void);
const cap_rec_t *capture_get(size_t i);           // task_telem, only while DONE
void capture_release(void);                       // task_telem: DONE -> IDLE

// Each line starts with the record index, so the GCS can place samples in time and spot lines lost on the
// radio link.
const char *capture_header(void);
size_t capture_format(size_t index, const cap_rec_t *r, char *out, size_t size);

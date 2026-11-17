// ASCII telemetry line builder (PLAN §9.1). Transport-agnostic: returns text, caller sends it.
#pragma once

#include <stddef.h>
#include <stdint.h>

#define TELEM_LINE_MAX 320u

// Fields of the T line in protocol order. Angles in rad and rates in rad/s here; the line uses degrees.
typedef struct {
    uint32_t t_ms;
    uint8_t arm, mode, fs;
    float roll, pitch, yaw;        // rad
    float p, q, r;                 // rad/s
    float roll_sp, pitch_sp;       // rad
    float yawr_sp;                 // rad/s
    float z, vz, z_sp, tof;        // m, m/s
    float thr;                     // [0,1]
    float motor[4];                // [0,1]
    uint32_t ctrl_us_max, jitter_us_max;
    float h_terrain, baro_rel;     // m
    uint8_t alt_src;
    float p_sp, q_sp;              // rad/s, roll/pitch rate setpoints
    uint8_t acro;                  // 1 = Acro active
    uint8_t alt_valid;             // altitude estimate usable (PLAN §7.4)
    float vel_fwd, vel_right;      // m/s, optical flow velocity, heading frame
    float pos_x, pos_y;            // m, flow position, room frame (POSZERO)
    uint8_t flow_q, flow_valid;
} telem_frame_t;

size_t telem_proto_build_T(char *buf, size_t size, const telem_frame_t *f);

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
} telem_frame_t;

size_t telem_proto_build_T(char *buf, size_t size, const telem_frame_t *f);

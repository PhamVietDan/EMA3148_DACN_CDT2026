// MicoAir MTF-01P frames over MSPv2 (PLAN §6.3): 0x1F01 rangefinder {u8 quality, i32 distance_mm (< 0 = out of
// range)}, 0x1F02 optical flow {u8 quality, i32 motion_x, i32 motion_y}. Layouts as in Betaflight
// rangefinder_lidarmt.c / ArduPilot AP_MSP msp_sensors.h; 0x1F02 checked against a captured frame (2026-10-28).
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/mspv2.h"

#define MTF01P_CMD_RANGE 0x1F01u
#define MTF01P_CMD_FLOW  0x1F02u

typedef struct { uint8_t quality; int32_t distance_mm; } mtf01p_range_t;
typedef struct { uint8_t quality; int32_t motion_x, motion_y; } mtf01p_flow_t;

// false if the frame is not that type or has the wrong size.
bool mtf01p_parse_range(const mspv2_frame_t *f, mtf01p_range_t *out);
bool mtf01p_parse_flow(const mspv2_frame_t *f, mtf01p_flow_t *out);

// mprintf: formatted text to the ground-station link. Safe from any task; never from an ISR.
#pragma once

#define MPRINTF_MAX_LEN 128u  // longer output is truncated

// Raw IMU print at 5 Hz (body frame FRD, filtered, bias not removed); all other logs are muted when non-zero.
// 0 = normal logs, 1 = accel X Y Z (mm/s^2), 2 = gyro X Y Z (deg/s).
#define LOG_RAW_MODE 0

// 1 = each task prints its 1 Hz status line (rates, timing, stack, IMU). 0 = events only (CALIB, errors).
#define LOG_PERIODIC_STATS 0

void mprintf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

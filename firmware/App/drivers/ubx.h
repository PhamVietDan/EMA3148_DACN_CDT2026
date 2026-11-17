// u-blox UBX protocol (GEP-M10DI = u-blox M10 on USART3): frame parser, NAV-PVT decode, CFG-VALSET builder.
// Frame: 0xB5 0x62, class, id, length (u16 LE), payload, ck_a ck_b (8-bit Fletcher over class..payload).
// Constants, the NAV-PVT layout (92 bytes, little-endian) and the CFG key IDs as PX4 PX4-GPSDrivers ubx.h /
// ubx.cpp (BSD-3, read 2026-11-14), not copied; the NAV-PVT msgout key for UART1 = the I2C key + 1 as PX4's
// cfgValsetPort. Pure C (host-tested).
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define UBX_SYNC1          0xB5u
#define UBX_SYNC2          0x62u
#define UBX_CLASS_NAV      0x01u
#define UBX_ID_NAV_PVT     0x07u
#define UBX_CLASS_ACK      0x05u
#define UBX_ID_ACK_NAK     0x00u
#define UBX_ID_ACK_ACK     0x01u
#define UBX_CLASS_CFG      0x06u
#define UBX_ID_CFG_VALSET  0x8Au
#define UBX_NAV_PVT_LEN    92u
#define UBX_MAX_PAYLOAD    100u  // NAV-PVT and ACK fit; longer messages are skipped

// CFG-VALSET layers (bit mask) and keys. Value size from key bits 28..30: 1 = bit (1 byte), 2 = 1 byte,
// 3 = 2 bytes, 4 = 4 bytes.
#define UBX_CFG_LAYER_RAM                  0x01u
#define UBX_CFG_KEY_UART1OUTPROT_UBX       0x10740001u
#define UBX_CFG_KEY_UART1OUTPROT_NMEA      0x10740002u
#define UBX_CFG_KEY_MSGOUT_NAV_PVT_UART1   0x20910007u  // PX4: UBX_CFG_KEY_MSGOUT_UBX_NAV_PVT_I2C 0x20910006 + 1
#define UBX_CFG_KEY_RATE_MEAS              0x30210001u  // ms between measurements (100 = 10 Hz, as PX4)

typedef struct {
    uint8_t state;
    uint8_t cls, id;
    uint16_t len, idx;
    uint8_t ck_a, ck_b;
    uint8_t payload[UBX_MAX_PAYLOAD];
    uint32_t frames, ck_errors, too_long;
} ubx_parser_t;

typedef struct {
    uint32_t itow_ms;
    uint8_t fix_type;      // 0 none, 1 dead reckoning, 2 2D, 3 3D, 4 GNSS + DR, 5 time only
    bool fix_ok;           // flags.gnssFixOK: within the receiver's DOP / accuracy masks
    uint8_t num_sv;
    int32_t lat_e7, lon_e7;  // deg x 1e7
    int32_t hmsl_mm;
    uint32_t h_acc_mm, v_acc_mm;
    int32_t vel_n_mm, vel_e_mm, vel_d_mm;  // mm/s, NED
    uint32_t s_acc_mm;     // mm/s
    uint16_t pdop;         // x 0.01
} ubx_pvt_t;

typedef struct {
    uint32_t key;
    uint32_t value;
} ubx_kv_t;

void ubx_parser_init(ubx_parser_t *p);
// Feeds one byte; true when a complete frame with a good checksum is in p->cls / id / len / payload (valid
// until the next call).
bool ubx_parse_byte(ubx_parser_t *p, uint8_t b);
bool ubx_decode_nav_pvt(const uint8_t *payload, uint16_t len, ubx_pvt_t *out);
// Whole CFG-VALSET frame (RAM layer) into buf; returns its length, 0 if it does not fit or a key has an
// unsupported size.
size_t ubx_build_valset(uint8_t *buf, size_t size, const ubx_kv_t *kv, size_t n);

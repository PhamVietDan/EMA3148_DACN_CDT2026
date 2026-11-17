// MSPv2 frame parser (PLAN §6.3), pure C: '$' 'X' '<'|'>' flag(u8) cmd(u16 LE) size(u16 LE) payload crc8_dvb_s2,
// the CRC covering flag..payload. Feed bytes; a complete frame with a good CRC is returned once.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MSPV2_PAYLOAD_MAX 32u  // MTF-01P frames carry 5 or 9 bytes; longer frames are skipped

typedef struct {
    uint16_t cmd;
    uint16_t size;
    uint8_t payload[MSPV2_PAYLOAD_MAX];
} mspv2_frame_t;

typedef struct {
    uint8_t state;
    uint8_t crc;
    uint8_t hdr[5];  // flag, cmd lo/hi, size lo/hi
    uint8_t n_hdr;
    uint16_t n_payload;
    mspv2_frame_t frame;
    uint32_t frames, crc_errors, oversize;
} mspv2_parser_t;

uint8_t mspv2_crc8_dvb_s2(uint8_t crc, uint8_t b);
void mspv2_init(mspv2_parser_t *p);
bool mspv2_feed(mspv2_parser_t *p, uint8_t b);  // true: p->frame holds a new valid frame

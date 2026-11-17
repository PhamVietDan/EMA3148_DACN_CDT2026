// MSPv2 frame parser (PLAN §6.3).
#include "drivers/mspv2.h"

enum { ST_SYNC, ST_X, ST_DIR, ST_HDR, ST_PAYLOAD, ST_CRC };

uint8_t mspv2_crc8_dvb_s2(uint8_t crc, uint8_t b)
{
    crc ^= b;
    for (int i = 0; i < 8; i++) {
        crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0xD5u) : (uint8_t)(crc << 1);
    }
    return crc;
}

void mspv2_init(mspv2_parser_t *p)
{
    *p = (mspv2_parser_t){0};
}

bool mspv2_feed(mspv2_parser_t *p, uint8_t b)
{
    switch (p->state) {
    case ST_SYNC:
        p->state = (b == '$') ? ST_X : ST_SYNC;
        return false;
    case ST_X:
        p->state = (b == 'X') ? ST_DIR : (b == '$') ? ST_X : ST_SYNC;
        return false;
    case ST_DIR:
        if (b == '<' || b == '>') {
            p->state = ST_HDR;
            p->n_hdr = 0;
            p->crc = 0;
        } else {
            p->state = (b == '$') ? ST_X : ST_SYNC;
        }
        return false;
    case ST_HDR:
        p->hdr[p->n_hdr++] = b;
        p->crc = mspv2_crc8_dvb_s2(p->crc, b);
        if (p->n_hdr == sizeof p->hdr) {
            p->frame.cmd = (uint16_t)(p->hdr[1] | (p->hdr[2] << 8));
            p->frame.size = (uint16_t)(p->hdr[3] | (p->hdr[4] << 8));
            p->n_payload = 0;
            if (p->frame.size > MSPV2_PAYLOAD_MAX) {
                p->oversize++;
                p->state = ST_SYNC;  // resynchronise on the next '$'
            } else {
                p->state = (p->frame.size == 0u) ? ST_CRC : ST_PAYLOAD;
            }
        }
        return false;
    case ST_PAYLOAD:
        p->frame.payload[p->n_payload++] = b;
        p->crc = mspv2_crc8_dvb_s2(p->crc, b);
        if (p->n_payload == p->frame.size) {
            p->state = ST_CRC;
        }
        return false;
    case ST_CRC:
    default:
        p->state = ST_SYNC;
        if (b != p->crc) {
            p->crc_errors++;
            return false;
        }
        p->frames++;
        return true;
    }
}

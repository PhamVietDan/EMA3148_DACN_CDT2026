// u-blox UBX protocol, see ubx.h.
#include "drivers/ubx.h"

enum { ST_SYNC1, ST_SYNC2, ST_CLASS, ST_ID, ST_LEN1, ST_LEN2, ST_PAYLOAD, ST_CK_A, ST_CK_B };

void ubx_parser_init(ubx_parser_t *p)
{
    *p = (ubx_parser_t){0};
}

static void ck_add(ubx_parser_t *p, uint8_t b)
{
    p->ck_a = (uint8_t)(p->ck_a + b);
    p->ck_b = (uint8_t)(p->ck_b + p->ck_a);
}

bool ubx_parse_byte(ubx_parser_t *p, uint8_t b)
{
    switch (p->state) {
    case ST_SYNC1:
        if (b == UBX_SYNC1) {
            p->state = ST_SYNC2;
        }
        break;
    case ST_SYNC2:
        p->state = b == UBX_SYNC2 ? ST_CLASS : (b == UBX_SYNC1 ? ST_SYNC2 : ST_SYNC1);
        break;
    case ST_CLASS:
        p->ck_a = p->ck_b = 0u;
        ck_add(p, b);
        p->cls = b;
        p->state = ST_ID;
        break;
    case ST_ID:
        ck_add(p, b);
        p->id = b;
        p->state = ST_LEN1;
        break;
    case ST_LEN1:
        ck_add(p, b);
        p->len = b;
        p->state = ST_LEN2;
        break;
    case ST_LEN2:
        ck_add(p, b);
        p->len |= (uint16_t)((uint16_t)b << 8);
        p->idx = 0u;
        if (p->len > UBX_MAX_PAYLOAD) {
            p->too_long++;
            p->state = ST_SYNC1;  // resynchronise on the next 0xB5
        } else {
            p->state = p->len > 0u ? ST_PAYLOAD : ST_CK_A;
        }
        break;
    case ST_PAYLOAD:
        ck_add(p, b);
        p->payload[p->idx++] = b;
        if (p->idx >= p->len) {
            p->state = ST_CK_A;
        }
        break;
    case ST_CK_A:
        p->state = b == p->ck_a ? ST_CK_B : ST_SYNC1;
        if (b != p->ck_a) {
            p->ck_errors++;
        }
        break;
    default:  // ST_CK_B
        p->state = ST_SYNC1;
        if (b == p->ck_b) {
            p->frames++;
            return true;
        }
        p->ck_errors++;
        break;
    }
    return false;
}

static uint32_t u32le(const uint8_t *s)
{
    return (uint32_t)s[0] | ((uint32_t)s[1] << 8) | ((uint32_t)s[2] << 16) | ((uint32_t)s[3] << 24);
}

static int32_t i32le(const uint8_t *s)
{
    return (int32_t)u32le(s);
}

bool ubx_decode_nav_pvt(const uint8_t *s, uint16_t len, ubx_pvt_t *o)
{
    if (len < UBX_NAV_PVT_LEN) {
        return false;
    }
    o->itow_ms = u32le(s + 0);
    o->fix_type = s[20];
    o->fix_ok = (s[21] & 0x01u) != 0u;
    o->num_sv = s[23];
    o->lon_e7 = i32le(s + 24);
    o->lat_e7 = i32le(s + 28);
    o->hmsl_mm = i32le(s + 36);
    o->h_acc_mm = u32le(s + 40);
    o->v_acc_mm = u32le(s + 44);
    o->vel_n_mm = i32le(s + 48);
    o->vel_e_mm = i32le(s + 52);
    o->vel_d_mm = i32le(s + 56);
    o->s_acc_mm = u32le(s + 68);
    o->pdop = (uint16_t)(s[76] | (s[77] << 8));
    return true;
}

size_t ubx_build_valset(uint8_t *buf, size_t size, const ubx_kv_t *kv, size_t n)
{
    size_t len = 4u;  // version, layers, reserved[2]
    for (size_t i = 0; i < n; i++) {
        uint32_t sz = (kv[i].key >> 28) & 7u;
        if (sz < 1u || sz > 4u) {
            return 0u;
        }
        len += 4u + (sz == 3u ? 2u : sz == 4u ? 4u : 1u);
    }
    if (size < len + 8u) {
        return 0u;
    }
    uint8_t *p = buf;
    *p++ = UBX_SYNC1;
    *p++ = UBX_SYNC2;
    *p++ = UBX_CLASS_CFG;
    *p++ = UBX_ID_CFG_VALSET;
    *p++ = (uint8_t)(len & 0xFFu);
    *p++ = (uint8_t)(len >> 8);
    *p++ = 0u;                   // version 0
    *p++ = UBX_CFG_LAYER_RAM;    // RAM only: the module keeps its own saved setup after a power cycle
    *p++ = 0u;
    *p++ = 0u;
    for (size_t i = 0; i < n; i++) {
        uint32_t sz = (kv[i].key >> 28) & 7u;
        uint32_t bytes = sz == 3u ? 2u : sz == 4u ? 4u : 1u;
        for (uint32_t k = 0; k < 4u; k++) {
            *p++ = (uint8_t)(kv[i].key >> (8u * k));
        }
        for (uint32_t k = 0; k < bytes; k++) {
            *p++ = (uint8_t)(kv[i].value >> (8u * k));
        }
    }
    uint8_t a = 0u, b = 0u;
    for (uint8_t *q = buf + 2; q < p; q++) {
        a = (uint8_t)(a + *q);
        b = (uint8_t)(b + a);
    }
    *p++ = a;
    *p++ = b;
    return (size_t)(p - buf);
}

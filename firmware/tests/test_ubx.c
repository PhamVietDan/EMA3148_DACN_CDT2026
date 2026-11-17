// Host tests for App/drivers/ubx.c: frames built here byte by byte from the documented layout (ubx.h).
#include <string.h>

#include "drivers/ubx.h"
#include "test_common.h"

static size_t frame(uint8_t *out, uint8_t cls, uint8_t id, const uint8_t *pl, uint16_t len)
{
    out[0] = UBX_SYNC1;
    out[1] = UBX_SYNC2;
    out[2] = cls;
    out[3] = id;
    out[4] = (uint8_t)len;
    out[5] = (uint8_t)(len >> 8);
    memcpy(out + 6, pl, len);
    uint8_t a = 0, b = 0;
    for (size_t i = 2; i < 6u + len; i++) {
        a = (uint8_t)(a + out[i]);
        b = (uint8_t)(b + a);
    }
    out[6 + len] = a;
    out[7 + len] = b;
    return 8u + len;
}

static void put32(uint8_t *p, uint32_t v)
{
    for (int i = 0; i < 4; i++) {
        p[i] = (uint8_t)(v >> (8 * i));
    }
}

static void test_nav_pvt(void)
{
    uint8_t pl[UBX_NAV_PVT_LEN] = {0};
    put32(pl + 0, 123456u);
    pl[20] = 3u;     // 3D
    pl[21] = 0x01u;  // gnssFixOK
    pl[23] = 14u;
    put32(pl + 24, (uint32_t)1058123456);   // lon 105.8123456
    put32(pl + 28, (uint32_t)210345678);    // lat 21.0345678
    put32(pl + 36, 12345u);
    put32(pl + 40, 1500u);
    put32(pl + 44, 2500u);
    put32(pl + 48, (uint32_t)-250);
    put32(pl + 52, 400u);
    put32(pl + 56, (uint32_t)-30);
    put32(pl + 68, 80u);
    pl[76] = 0x2Cu;  // pDOP 1.32
    pl[77] = 0x01u;
    // NMEA junk, a frame with a bad checksum and an oversize header before the good frame
    uint8_t buf[400];
    size_t n = 0;
    const char *nmea = "$GNGGA,123519,2103.4,N,10548.7,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    memcpy(buf, nmea, strlen(nmea));
    n += strlen(nmea);
    size_t bad = frame(buf + n, UBX_CLASS_NAV, UBX_ID_NAV_PVT, pl, sizeof pl);
    buf[n + bad - 1] ^= 0x55u;
    n += bad;
    const uint8_t big[] = {UBX_SYNC1, UBX_SYNC2, 0x01, 0x35, 0x00, 0x04};
    memcpy(buf + n, big, sizeof big);
    n += sizeof big;
    n += frame(buf + n, UBX_CLASS_NAV, UBX_ID_NAV_PVT, pl, sizeof pl);

    ubx_parser_t p;
    ubx_parser_init(&p);
    int got = 0;
    ubx_pvt_t o = {0};
    for (size_t i = 0; i < n; i++) {
        if (ubx_parse_byte(&p, buf[i]) && p.cls == UBX_CLASS_NAV && p.id == UBX_ID_NAV_PVT) {
            got += ubx_decode_nav_pvt(p.payload, p.len, &o);
        }
    }
    CHECK(got == 1 && p.ck_errors == 1u && p.too_long == 1u, "parser: got %d, ck errors %lu, too long %lu", got,
          (unsigned long)p.ck_errors, (unsigned long)p.too_long);
    CHECK(o.itow_ms == 123456u && o.fix_type == 3u && o.fix_ok && o.num_sv == 14u && o.lon_e7 == 1058123456 &&
              o.lat_e7 == 210345678 && o.hmsl_mm == 12345 && o.h_acc_mm == 1500u && o.v_acc_mm == 2500u &&
              o.vel_n_mm == -250 && o.vel_e_mm == 400 && o.vel_d_mm == -30 && o.s_acc_mm == 80u && o.pdop == 300u,
          "NAV-PVT decode: fix %u sv %u lat %ld lon %ld vn %ld pdop %u", (unsigned)o.fix_type, (unsigned)o.num_sv,
          (long)o.lat_e7, (long)o.lon_e7, (long)o.vel_n_mm, (unsigned)o.pdop);
}

static void test_valset(void)
{
    const ubx_kv_t kv[] = {
        {UBX_CFG_KEY_UART1OUTPROT_UBX, 1u},
        {UBX_CFG_KEY_MSGOUT_NAV_PVT_UART1, 1u},
        {UBX_CFG_KEY_RATE_MEAS, 100u},
        {UBX_CFG_KEY_UART1OUTPROT_NMEA, 0u},
    };
    uint8_t buf[64];
    size_t n = ubx_build_valset(buf, sizeof buf, kv, 4u);
    // payload: 4 header + (4+1) + (4+1) + (4+2) + (4+1) = 25; frame 33
    static const uint8_t want_head[] = {0xB5, 0x62, 0x06, 0x8A, 25, 0, 0, 0x01, 0, 0,
                                        0x01, 0x00, 0x74, 0x10, 0x01,   // UART1OUTPROT_UBX = 1
                                        0x07, 0x00, 0x91, 0x20, 0x01,   // MSGOUT NAV-PVT UART1 = 1
                                        0x01, 0x00, 0x21, 0x30, 100, 0, // RATE_MEAS = 100 ms
                                        0x02, 0x00, 0x74, 0x10, 0x00};  // UART1OUTPROT_NMEA = 0
    bool same = n == 33u && memcmp(buf, want_head, sizeof want_head) == 0;
    // The parser must accept the frame it builds (checksum)
    ubx_parser_t p;
    ubx_parser_init(&p);
    int ok = 0;
    for (size_t i = 0; i < n; i++) {
        ok += ubx_parse_byte(&p, buf[i]);
    }
    CHECK(same && ok == 1 && p.cls == UBX_CLASS_CFG && p.id == UBX_ID_CFG_VALSET && p.len == 25u,
          "VALSET: len %u, layout %s, parsed %d", (unsigned)n, same ? "ok" : "WRONG", ok);
    CHECK(ubx_build_valset(buf, 20u, kv, 4u) == 0u, "VALSET: no size check");
}

void test_ubx(void)
{
    test_nav_pvt();
    test_valset();
}

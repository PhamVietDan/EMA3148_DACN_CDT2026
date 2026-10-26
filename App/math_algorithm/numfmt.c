// Float <-> text without newlib's strtof/printf %f (both pull in malloc-based dtoa code).
#include "math_algorithm/numfmt.h"

#include <stdint.h>

bool numfmt_parse_float(const char *s, float *out)
{
    const char *p = s;
    bool neg = false;
    if (*p == '+' || *p == '-') {
        neg = (*p == '-');
        p++;
    }
    // Up to 9 significant digits go into a uint32 mantissa; the rest only shift the decimal exponent.
    uint32_t mant = 0;
    int exp10 = 0;
    int digits = 0;
    bool any = false;
    while (*p >= '0' && *p <= '9') {
        if (digits < 9) {
            mant = mant * 10u + (uint32_t)(*p - '0');
            if (mant != 0u) {
                digits++;
            }
        } else {
            exp10++;
        }
        any = true;
        p++;
    }
    if (*p == '.') {
        p++;
        while (*p >= '0' && *p <= '9') {
            if (digits < 9) {
                mant = mant * 10u + (uint32_t)(*p - '0');
                if (mant != 0u) {
                    digits++;
                }
                exp10--;
            }
            any = true;
            p++;
        }
    }
    if (!any) {
        return false;
    }
    if (*p == 'e' || *p == 'E') {
        p++;
        bool eneg = false;
        if (*p == '+' || *p == '-') {
            eneg = (*p == '-');
            p++;
        }
        if (!(*p >= '0' && *p <= '9')) {
            return false;
        }
        int e = 0;
        while (*p >= '0' && *p <= '9') {
            if (e < 1000) {
                e = e * 10 + (*p - '0');
            }
            p++;
        }
        exp10 += eneg ? -e : e;
    }
    if (*p != '\0' || exp10 > 38 || exp10 < -45) {
        return false;
    }
    float v = (float)mant;
    float scale = 1.0f;
    int n = (exp10 < 0) ? -exp10 : exp10;
    for (int i = 0; i < n; i++) {
        scale *= 10.0f;
    }
    v = (exp10 < 0) ? v / scale : v * scale;
    *out = neg ? -v : v;
    return true;
}

size_t numfmt_float(char *buf, size_t size, float v, unsigned decimals)
{
    if (size == 0u) {
        return 0;
    }
    if (decimals > 6u) {
        decimals = 6u;
    }
    char tmp[32];
    size_t n = 0;
    bool neg = v < 0.0f;
    float a = neg ? -v : v;
    uint32_t scale = 1u;
    for (unsigned i = 0; i < decimals; i++) {
        scale *= 10u;
    }
    if (a > 4.0e9f / (float)scale) {
        a = 4.0e9f / (float)scale;  // out of printable range: clip rather than overflow
    }
    uint32_t t = (uint32_t)(a * (float)scale + 0.5f);
    uint32_t ip = t / scale;
    uint32_t fp = t % scale;
    if (neg && t != 0u) {
        tmp[n++] = '-';
    }
    char rev[12];
    int r = 0;
    do {
        rev[r++] = (char)('0' + ip % 10u);
        ip /= 10u;
    } while (ip != 0u);
    while (r > 0) {
        tmp[n++] = rev[--r];
    }
    if (fp != 0u) {
        tmp[n++] = '.';
        for (uint32_t d = scale / 10u; d > 0u && fp != 0u; d /= 10u) {
            tmp[n++] = (char)('0' + fp / d);
            fp %= d;
        }
    }
    if (n >= size) {
        n = size - 1u;
    }
    for (size_t i = 0; i < n; i++) {
        buf[i] = tmp[i];
    }
    buf[n] = '\0';
    return n;
}

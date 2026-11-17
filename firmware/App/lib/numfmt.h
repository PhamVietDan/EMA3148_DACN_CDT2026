// Float <-> text without newlib's strtof/printf %f (both pull in malloc-based dtoa code).
#pragma once

#include <stdbool.h>
#include <stddef.h>

// Parses "[-+]digits[.digits][e[-+]digits]" covering the whole string. Returns false on any junk.
bool numfmt_parse_float(const char *s, float *out);
// Writes v with up to `decimals` fraction digits, trailing zeros removed ("0.05", "-12.5", "3").
size_t numfmt_float(char *buf, size_t size, float v, unsigned decimals);

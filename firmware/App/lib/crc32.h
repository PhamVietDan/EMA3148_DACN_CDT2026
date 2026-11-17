// CRC-32 (IEEE 802.3, reflected, poly 0xEDB88320), bitwise: small and only used on short flash images.
#pragma once

#include <stddef.h>
#include <stdint.h>

uint32_t crc32_update(uint32_t crc, const void *data, size_t len);  // start with crc = 0

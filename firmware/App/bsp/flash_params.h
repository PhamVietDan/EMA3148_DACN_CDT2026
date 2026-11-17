// Parameter storage in flash sector 11 (0x080E0000, 128 KB), reserved in STM32F405RGTX_FLASH.ld.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

const uint8_t *flash_params_image(size_t *len);  // memory-mapped read of the stored image
// Erases sector 11 and programs the image. The CPU stalls for the whole erase (single-bank F405):
// only call while disarmed. Returns false on any HAL error or read-back mismatch.
bool flash_params_write(const uint8_t *img, size_t len);

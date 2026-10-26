// Parameter storage in flash sector 11 (0x080E0000, 128 KB), reserved in STM32F405RGTX_FLASH.ld.
#include "board_hardware_layer/flash_params.h"

#include <string.h>

#include "control_logic/params.h"
#include "main.h"

// VERIFY: RM0090 §3.3 Table 5 (sector 11 = 0x080E0000..0x080FFFFF, 128 KB);
// DS8626 flash table: 128 KB sector erase ~1 s typ, 2 s max at x32 (VOLTAGE_RANGE_3, 2.7-3.6 V).
#define PARAMS_FLASH_ADDR 0x080E0000u

const uint8_t *flash_params_image(size_t *len)
{
    *len = PARAMS_IMAGE_MAX;
    return (const uint8_t *)PARAMS_FLASH_ADDR;
}

bool flash_params_write(const uint8_t *img, size_t len)
{
    if (len == 0u || len > PARAMS_IMAGE_MAX) {
        return false;
    }
    bool ok = HAL_FLASH_Unlock() == HAL_OK;
    if (ok) {
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                               FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
        FLASH_EraseInitTypeDef erase = {
            .TypeErase = FLASH_TYPEERASE_SECTORS,
            .Sector = FLASH_SECTOR_11,
            .NbSectors = 1,
            .VoltageRange = FLASH_VOLTAGE_RANGE_3,
        };
        uint32_t bad_sector = 0;
        ok = HAL_FLASHEx_Erase(&erase, &bad_sector) == HAL_OK;
        for (size_t off = 0; ok && off < len; off += 4u) {
            uint32_t word = 0xFFFFFFFFu;
            memcpy(&word, &img[off], (len - off < 4u) ? len - off : 4u);
            ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAMS_FLASH_ADDR + off, word) == HAL_OK;
        }
        HAL_FLASH_Lock();
    }
    return ok && memcmp((const void *)PARAMS_FLASH_ADDR, img, len) == 0;
}

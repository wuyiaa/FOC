#ifndef USER_BSP_FLASH_SAVE_H
#define USER_BSP_FLASH_SAVE_H

#include <stdint.h>
#include "stm32g4xx_hal.h"

/* Existing final 2 KB page, shared by encoder and J/B/Tf calibration. */
#define FLASH_SAVE_ADDRESS      0x0807F800UL
#define FLASH_SAVE_RECORD_COUNT (FLASH_PAGE_SIZE / 8U)
#define FLASH_SAVE_ERASED       0xFFFFFFFFFFFFFFFFULL

/* Append doublewords and verify them immediately; call with MOS outputs off.
   Page rollover retains the latest complete record of both data types. */
HAL_StatusTypeDef flash_save_append(const uint64_t *words, uint32_t count);

#endif /* USER_BSP_FLASH_SAVE_H */

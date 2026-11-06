#include "flash_save.h"
#include "encoder_zero_calibration.h"
#include "B_J_Tf_Identification.h"
#include "stm32g4xx_hal_flash_ex.h"

#define FLASH_SAVE_PAGE 127U
#define FLASH_SAVE_BANK FLASH_BANK_2

static HAL_StatusTypeDef flash_save_program(uint32_t index,
                                                   const uint64_t *words,
                                                   uint32_t count)
{
    const volatile uint64_t *records =
        (const volatile uint64_t *)FLASH_SAVE_ADDRESS;
    uint32_t i;
    HAL_StatusTypeDef status;
    for (i = 0U; i < count; i++)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                  FLASH_SAVE_ADDRESS + (index + i) * 8U,
                                  words[i]);
        if (status != HAL_OK)
        {
            return status;
        }
        if (records[index + i] != words[i])
        {
            return HAL_ERROR;
        }
    }
    return HAL_OK;
}

/* Both record types share the existing reserved page. On rollover, retain
   the latest complete record of each type before appending the new result. */
HAL_StatusTypeDef flash_save_append(const uint64_t *words,
                                                  uint32_t count)
{
    const volatile uint64_t *records =
        (const volatile uint64_t *)FLASH_SAVE_ADDRESS;
    FLASH_EraseInitTypeDef erase = {0};
    uint64_t latest_encoder = FLASH_SAVE_ERASED;
    uint64_t latest_motor[MOTOR_PARAMETERS_RECORD_WORDS];
    uint8_t motor_found = 0U;
    uint32_t next = 0U;
    uint32_t i;
    uint32_t j;
    uint32_t page_error = 0xFFFFFFFFUL;
    HAL_StatusTypeDef status;

#if defined(FLASH_OPTR_DBANK)
    if (((FLASH->OPTR & FLASH_OPTR_DBANK) == 0U) ||
        ((SYSCFG->MEMRMP & SYSCFG_MEMRMP_FB_MODE) != 0U))
    {
        return HAL_ERROR;
    }
#else
    return HAL_ERROR;
#endif

    for (i = 0U; i < FLASH_SAVE_RECORD_COUNT; i++)
    {
        if (records[i] != FLASH_SAVE_ERASED)
        {
            next = i + 1U;
        }
        if (encoder_zero_flash_record_valid(records[i]) != 0U)
        {
            latest_encoder = records[i];
        }
        if ((i + MOTOR_PARAMETERS_RECORD_WORDS <= FLASH_SAVE_RECORD_COUNT) &&
            (motor_parameters_flash_record_valid(&records[i]) != 0U))
        {
            for (j = 0U; j < MOTOR_PARAMETERS_RECORD_WORDS; j++)
            {
                latest_motor[j] = records[i + j];
            }
            motor_found = 1U;
        }
    }
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
    {
        return status;
    }
    if (next + count > FLASH_SAVE_RECORD_COUNT)
    {
        erase.TypeErase = FLASH_TYPEERASE_PAGES;
        erase.Banks = FLASH_SAVE_BANK;
        erase.Page = FLASH_SAVE_PAGE;
        erase.NbPages = 1U;
        status = HAL_FLASHEx_Erase(&erase, &page_error);
        if ((status == HAL_OK) && (page_error != 0xFFFFFFFFUL))
        {
            status = HAL_ERROR;
        }
        next = 0U;
        if ((status == HAL_OK) && (latest_encoder != FLASH_SAVE_ERASED))
        {
            status = flash_save_program(next, &latest_encoder, 1U);
            next++;
        }
        if ((status == HAL_OK) && (motor_found != 0U))
        {
            status = flash_save_program(next, latest_motor,
                                                MOTOR_PARAMETERS_RECORD_WORDS);
            next += MOTOR_PARAMETERS_RECORD_WORDS;
        }
    }
    if (status == HAL_OK)
    {
        status = flash_save_program(next, words, count);
    }
    if (HAL_FLASH_Lock() != HAL_OK)
    {
        status = HAL_ERROR;
    }
    return status;
}

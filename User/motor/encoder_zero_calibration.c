#include "encoder_zero_calibration.h"

#include <stddef.h>

#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_flash_ex.h"

#define ENCODER_ZERO_FLASH_ADDRESS       0x0807F800UL
#define ENCODER_ZERO_FLASH_PAGE          127U
#define ENCODER_ZERO_FLASH_BANK          FLASH_BANK_2
#define ENCODER_ZERO_FLASH_RECORD_COUNT  (FLASH_PAGE_SIZE / 8U)
#define ENCODER_ZERO_RECORD_MAGIC        0xA5470000UL
#define ENCODER_ZERO_RECORD_MAGIC_MASK   0xFFFF0000UL
#define ENCODER_ZERO_RECORD_ERASED       0xFFFFFFFFFFFFFFFFULL

volatile pmsm_encoder_zero_cal_t calibration;

static uint64_t encoder_zero_record_make(uint16_t raw_offset)
{
    uint32_t payload = ENCODER_ZERO_RECORD_MAGIC | (uint32_t)raw_offset;
    uint32_t payload_inverse = ~payload;

    return ((uint64_t)payload_inverse << 32U) | (uint64_t)payload;
}

static uint8_t encoder_zero_record_parse(uint64_t record, uint16_t *raw_offset)
{
    uint32_t payload = (uint32_t)record;
    uint32_t payload_inverse = (uint32_t)(record >> 32U);
    uint16_t raw = (uint16_t)payload;

    if ((payload_inverse != ~payload) ||
        ((payload & ENCODER_ZERO_RECORD_MAGIC_MASK) !=
         ENCODER_ZERO_RECORD_MAGIC) ||
        (raw > AS5047P_DATA_MASK))
    {
        return 0U;
    }

    *raw_offset = raw;
    return 1U;
}

HAL_StatusTypeDef encoder_zero_flash_load(volatile uint16_t *raw_offset)
{
    const volatile uint64_t *record_address =
        (const volatile uint64_t *)ENCODER_ZERO_FLASH_ADDRESS;
    uint16_t latest_raw = 0U;
    uint16_t decoded_raw;
    uint32_t index;
    uint8_t found = 0U;

    if (raw_offset == NULL)
    {
        return HAL_ERROR;
    }

    for (index = 0U; index < ENCODER_ZERO_FLASH_RECORD_COUNT; index++)
    {
        uint64_t record = record_address[index];

        if ((record != ENCODER_ZERO_RECORD_ERASED) &&
            (encoder_zero_record_parse(record, &decoded_raw) != 0U))
        {
            latest_raw = decoded_raw;
            found = 1U;
        }
    }

    if (found == 0U)
    {
        return HAL_ERROR;
    }

    *raw_offset = latest_raw;
    return HAL_OK;
}

HAL_StatusTypeDef encoder_zero_flash_save(uint16_t raw_offset)
{
    const volatile uint64_t *record_address =
        (const volatile uint64_t *)ENCODER_ZERO_FLASH_ADDRESS;
    FLASH_EraseInitTypeDef erase = {0};
    uint64_t record;
    uint16_t latest_raw = 0U;
    uint16_t decoded_raw;
    uint32_t index;
    uint32_t first_erased_index = ENCODER_ZERO_FLASH_RECORD_COUNT;
    uint32_t page_error = 0xFFFFFFFFUL;
    uint8_t found = 0U;
    HAL_StatusTypeDef status;

    if (raw_offset > AS5047P_DATA_MASK)
    {
        return HAL_ERROR;
    }

#if defined(FLASH_OPTR_DBANK)
    /* This reserved page is Bank 2 page 127 in the configured dual-bank map. */
    if ((FLASH->OPTR & FLASH_OPTR_DBANK) == 0U)
    {
        return HAL_ERROR;
    }
    if ((SYSCFG->MEMRMP & SYSCFG_MEMRMP_FB_MODE) != 0U)
    {
        return HAL_ERROR;
    }
#else
    return HAL_ERROR;
#endif

    for (index = 0U; index < ENCODER_ZERO_FLASH_RECORD_COUNT; index++)
    {
        uint64_t existing_record = record_address[index];

        if ((existing_record == ENCODER_ZERO_RECORD_ERASED) &&
            (first_erased_index == ENCODER_ZERO_FLASH_RECORD_COUNT))
        {
            first_erased_index = index;
        }
        else if ((existing_record != ENCODER_ZERO_RECORD_ERASED) &&
                 (encoder_zero_record_parse(existing_record,
                                            &decoded_raw) != 0U))
        {
            latest_raw = decoded_raw;
            found = 1U;
        }
    }

    /* Avoid consuming Flash endurance when recalibration produced no change. */
    if ((found != 0U) && (latest_raw == raw_offset))
    {
        return HAL_OK;
    }

    status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
    {
        return status;
    }
    status = HAL_OK;

    if (first_erased_index >= ENCODER_ZERO_FLASH_RECORD_COUNT)
    {
        erase.TypeErase = FLASH_TYPEERASE_PAGES;
        erase.Banks = ENCODER_ZERO_FLASH_BANK;
        erase.Page = ENCODER_ZERO_FLASH_PAGE;
        erase.NbPages = 1U;
        status = HAL_FLASHEx_Erase(&erase, &page_error);
        first_erased_index = 0U;
        if ((status == HAL_OK) && (page_error != 0xFFFFFFFFUL))
        {
            status = HAL_ERROR;
        }
    }

    if (status == HAL_OK)
    {
        record = encoder_zero_record_make(raw_offset);
        status = HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_DOUBLEWORD,
            ENCODER_ZERO_FLASH_ADDRESS + (first_erased_index * 8U),
            record);
    }

    if ((status == HAL_OK) &&
        (record_address[first_erased_index] !=
         encoder_zero_record_make(raw_offset)))
    {
        status = HAL_ERROR;
    }

    if (HAL_FLASH_Lock() != HAL_OK)
    {
        status = HAL_ERROR;
    }

    return status;
}

#if (FOC_ENCODER_ZERO_CALIBRATION_MODE != 0U)
void foc_encoder_zero_calibration_init(void)
{
    calibration.align_start_cycle = 0U;
    calibration.sample_start_cycle = 0U;
    calibration.last_sample_sequence = 0U;
    calibration.sample_count = 0U;
    calibration.last_raw = 0U;
    calibration.sample_start_angle_rad = 0.0f;
    calibration.unwrapped_angle_offset_rad = 0.0f;
    calibration.angle_offset_sum_rad = 0.0f;
    calibration.mechanical_zero_raw = 0U;
    calibration.mechanical_zero_angle_rad = 0.0f;
    calibration.started = 0U;
    calibration.sampling = 0U;
    calibration.done = 0U;
    calibration.flash_save_attempted = 0U;
    calibration.flash_saved = 0U;
}

void foc_encoder_zero_calibration_step(pmsm_t *pm, float v_dc, float v_min)
{
    uint16_t raw;
    uint32_t sample_sequence;

    if (pm == NULL)
    {
        return;
    }

    if ((pm->encoder.fault != 0U) ||
        (pm->foc.current_limit_fault != 0U) ||
        (calibration.done != 0U))
    {
        foc_pwm_output_disable();
        return;
    }

    if (pm->encoder_foc_sample.valid == 0U)
    {
        return;
    }

    if (calibration.started == 0U)
    {
        calibration.align_start_cycle = DWT->CYCCNT;
        calibration.started = 1U;
    }

    /* Hold a fixed d-axis stator field while the rotor settles into alignment. */
    pm->foc.electrical_angle_rad = 0.0f;
    pm->foc.u_d = ENCODER_ZERO_ALIGN_VOLTAGE_V;
    pm->foc.u_q = 0.0f;
    inverse_park_transform(pm);
    svpwm_calc(pm, v_dc, v_min);

    if (DWT_GetElapsedCycles(calibration.align_start_cycle) >=
        (DWT_CPU_FREQ_HZ * ENCODER_ZERO_ALIGN_TIME_S))
    {
        raw = pm->encoder_foc_sample.raw;
        sample_sequence = pm->encoder_foc_sample.sequence;

        if (calibration.sampling == 0U)
        {
            calibration.sample_start_cycle = DWT->CYCCNT;
            calibration.last_raw = raw;
            calibration.last_sample_sequence = sample_sequence;
            calibration.sample_start_angle_rad =
                (float)raw * AS5047P_RAD_PER_COUNT;
            calibration.unwrapped_angle_offset_rad = 0.0f;
            calibration.angle_offset_sum_rad = 0.0f;
            calibration.sample_count = 1U;
            calibration.sampling = 1U;
        }
        else if (sample_sequence != calibration.last_sample_sequence)
        {
            float delta_angle_rad =
                (float)((int32_t)raw - (int32_t)calibration.last_raw) *
                AS5047P_RAD_PER_COUNT;
            float half_turn_rad = TWO_PI * 0.5f;

            if (delta_angle_rad > half_turn_rad)
            {
                delta_angle_rad -= TWO_PI;
            }
            else if (delta_angle_rad < -half_turn_rad)
            {
                delta_angle_rad += TWO_PI;
            }

            calibration.unwrapped_angle_offset_rad += delta_angle_rad;
            calibration.angle_offset_sum_rad +=
                calibration.unwrapped_angle_offset_rad;
            calibration.last_raw = raw;
            calibration.last_sample_sequence = sample_sequence;
            calibration.sample_count++;
        }

        if ((calibration.sampling != 0U) &&
            (DWT_GetElapsedCycles(calibration.sample_start_cycle) >=
             ((DWT_CPU_FREQ_HZ / 1000U) * ENCODER_ZERO_SAMPLE_TIME_MS)) &&
            (calibration.sample_count != 0U))
        {
            float average_angle_rad = calibration.sample_start_angle_rad +
                calibration.angle_offset_sum_rad /
                (float)calibration.sample_count;

            if (average_angle_rad < 0.0f)
            {
                average_angle_rad += TWO_PI;
            }
            else if (average_angle_rad >= TWO_PI)
            {
                average_angle_rad -= TWO_PI;
            }

            calibration.mechanical_zero_angle_rad = average_angle_rad;
            calibration.mechanical_zero_raw = (uint16_t)(
                average_angle_rad * AS5047P_COUNT_PER_RAD + 0.5f);
            if (calibration.mechanical_zero_raw > AS5047P_DATA_MASK)
            {
                calibration.mechanical_zero_raw = 0U;
            }

            calibration.done = 1U;
            foc_pwm_output_disable();
        }
    }
}
#endif

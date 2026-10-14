#include "encoder_zero_calibration.h"

#include <stddef.h>

#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_flash_ex.h"

#define ENCODER_ZERO_FLASH_ADDRESS       0x0807F800UL
#define ENCODER_ZERO_FLASH_PAGE          127U
#define ENCODER_ZERO_FLASH_BANK          FLASH_BANK_2
#define ENCODER_ZERO_FLASH_RECORD_COUNT  (FLASH_PAGE_SIZE / 8U)
#define ENCODER_ZERO_RECORD_MAGIC        0xA5480000UL
#define ENCODER_ZERO_RECORD_OLD_MAGIC    0xA5470000UL
#define ENCODER_ZERO_RECORD_MAGIC_MASK   0xFFFF0000UL
#define ENCODER_ZERO_RECORD_ERASED       0xFFFFFFFFFFFFFFFFULL
#define ENCODER_ZERO_RECORD_PHASE_BIT    0x00004000UL
#define ENCODER_ZERO_RECORD_ANGLE_BIT    0x00008000UL

volatile pmsm_encoder_zero_cal_t calibration;
volatile uint8_t encoder_zero_calibration_requested = FOC_ENCODER_ZERO_CALIBRATION_MODE;

static uint64_t encoder_zero_record_make(uint16_t raw_offset,
                                         uint8_t phase_reversed,
                                         int8_t electrical_direction)
{
    uint32_t payload = ENCODER_ZERO_RECORD_MAGIC | (uint32_t)raw_offset |
        ((phase_reversed != 0U) ? ENCODER_ZERO_RECORD_PHASE_BIT : 0U) |
        ((electrical_direction < 0) ? ENCODER_ZERO_RECORD_ANGLE_BIT : 0U);
    uint32_t payload_inverse = ~payload;

    return ((uint64_t)payload_inverse << 32U) | (uint64_t)payload;
}

static uint8_t encoder_zero_record_parse(uint64_t record,
                                         uint16_t *raw_offset,
                                         uint8_t *phase_reversed,
                                         int8_t *electrical_direction)
{
    uint32_t payload = (uint32_t)record;
    uint32_t payload_inverse = (uint32_t)(record >> 32U);
    uint16_t raw = (uint16_t)(payload & AS5047P_DATA_MASK);

    if ((payload_inverse != ~payload) ||
        (((payload & ENCODER_ZERO_RECORD_MAGIC_MASK) !=
          ENCODER_ZERO_RECORD_MAGIC) &&
         ((payload & ENCODER_ZERO_RECORD_MAGIC_MASK) !=
          ENCODER_ZERO_RECORD_OLD_MAGIC)))
    {
        return 0U;
    }

    *raw_offset = raw;
    if ((payload & ENCODER_ZERO_RECORD_MAGIC_MASK) ==
        ENCODER_ZERO_RECORD_OLD_MAGIC)
    {
        *phase_reversed = MOTOR_PHASE_REVERSED_DEFAULT;
        *electrical_direction = MOTOR_ELECTRICAL_DIRECTION_DEFAULT;
    }
    else
    {
        *phase_reversed =
            ((payload & ENCODER_ZERO_RECORD_PHASE_BIT) != 0U) ? 1U : 0U;
        *electrical_direction =
            ((payload & ENCODER_ZERO_RECORD_ANGLE_BIT) != 0U) ? -1 : 1;
    }
    return 1U;
}

HAL_StatusTypeDef encoder_zero_flash_load(volatile pmsm_encoder_zero_cal_t *config)
{
    const volatile uint64_t *record_address =
        (const volatile uint64_t *)ENCODER_ZERO_FLASH_ADDRESS;
    uint16_t latest_raw = 0U;
    uint16_t decoded_raw;
    uint8_t latest_phase = MOTOR_PHASE_REVERSED_DEFAULT;
    uint8_t decoded_phase;
    int8_t latest_direction = MOTOR_ELECTRICAL_DIRECTION_DEFAULT;
    int8_t decoded_direction;
    uint32_t index;
    uint8_t found = 0U;

    if (config == NULL)
    {
        return HAL_ERROR;
    }

    for (index = 0U; index < ENCODER_ZERO_FLASH_RECORD_COUNT; index++)
    {
        uint64_t record = record_address[index];

        if ((record != ENCODER_ZERO_RECORD_ERASED) &&
            (encoder_zero_record_parse(record, &decoded_raw,
                                       &decoded_phase,
                                       &decoded_direction) != 0U))
        {
            latest_raw = decoded_raw;
            latest_phase = decoded_phase;
            latest_direction = decoded_direction;
            found = 1U;
        }
    }

    if (found == 0U)
    {
        return HAL_ERROR;
    }

    config->mechanical_zero_raw = latest_raw;
    config->phase_reversed = latest_phase;
    config->electrical_direction = latest_direction;
    return HAL_OK;
}

HAL_StatusTypeDef encoder_zero_flash_save(const volatile pmsm_encoder_zero_cal_t *config)
{
    const volatile uint64_t *record_address =
        (const volatile uint64_t *)ENCODER_ZERO_FLASH_ADDRESS;
    FLASH_EraseInitTypeDef erase = {0};
    uint64_t record;
    uint16_t latest_raw = 0U;
    uint16_t decoded_raw;
    uint8_t latest_phase = 0U;
    uint8_t decoded_phase;
    int8_t latest_direction = 0;
    int8_t decoded_direction;
    uint32_t index;
    uint32_t first_erased_index = ENCODER_ZERO_FLASH_RECORD_COUNT;
    uint32_t page_error = 0xFFFFFFFFUL;
    uint8_t found = 0U;
    uint8_t latest_new_format = 0U;
    HAL_StatusTypeDef status;

    if ((config == NULL) ||
        (config->mechanical_zero_raw > AS5047P_DATA_MASK) ||
        (config->phase_reversed > 1U) ||
        ((config->electrical_direction != -1) &&
         (config->electrical_direction != 1)))
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
                                            &decoded_raw, &decoded_phase,
                                            &decoded_direction) != 0U))
        {
            latest_raw = decoded_raw;
            latest_phase = decoded_phase;
            latest_direction = decoded_direction;
            latest_new_format =
                ((((uint32_t)existing_record &
                   ENCODER_ZERO_RECORD_MAGIC_MASK) ==
                  ENCODER_ZERO_RECORD_MAGIC) ? 1U : 0U);
            found = 1U;
        }
    }

    /* Avoid consuming Flash endurance when recalibration produced no change. */
    if ((found != 0U) && (latest_new_format != 0U) &&
        (latest_raw == config->mechanical_zero_raw) &&
        (latest_phase == config->phase_reversed) &&
        (latest_direction == config->electrical_direction))
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
        record = encoder_zero_record_make(config->mechanical_zero_raw,
                                          config->phase_reversed,
                                          config->electrical_direction);
        status = HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_DOUBLEWORD,
            ENCODER_ZERO_FLASH_ADDRESS + (first_erased_index * 8U),
            record);
    }

    if ((status == HAL_OK) &&
        (record_address[first_erased_index] !=
         encoder_zero_record_make(config->mechanical_zero_raw,
                                  config->phase_reversed,
                                  config->electrical_direction)))
    {
        status = HAL_ERROR;
    }

    if (HAL_FLASH_Lock() != HAL_OK)
    {
        status = HAL_ERROR;
    }

    return status;
}

#if ((FOC_ENCODER_ZERO_CALIBRATION_MODE != 0U) || \
     (FOC_PHASE_AUTO_CALIBRATION_MODE != 0U))
void foc_encoder_zero_calibration_init(void)
{
    encoder_zero_calibration_requested = FOC_ENCODER_ZERO_CALIBRATION_MODE;
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
    calibration.state = 0U;
    calibration.failed = 0U;
    calibration.direction_retried = 0U;
    calibration.phase_retried = 0U;
    calibration.drive_start_cycle = 0U;
    calibration.drive_observation_started = 0U;
    calibration.drive_last_raw = 0U;
    calibration.drive_delta_counts = 0;
}
#endif

#if ((FOC_ENCODER_ZERO_CALIBRATION_MODE != 0U) || \
     (FOC_PHASE_AUTO_CALIBRATION_MODE != 0U))
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
        (calibration.failed != 0U) ||
        (calibration.done != 0U))
    {
        encoder_zero_calibration_requested = 0U;
        foc_pwm_output_disable();
        return;
    }
    if (encoder_zero_calibration_requested == 0U)
    {
        return;
    }
    if (pm->encoder_foc_sample.valid == 0U)
    {
        return;
    }

    if (calibration.started == 0U)
    {
        calibration.align_start_cycle = DWT->CYCCNT;
        calibration.sampling = 0U;
        calibration.sample_count = 0U;
        calibration.started = 1U;
    }

    /* Hold the d-axis at electrical zero while aligning and sampling. */
    pm->foc.electrical_angle_rad = 0.0f;
    pm->foc.u_d = ENCODER_ZERO_ALIGN_VOLTAGE_V;
    pm->foc.u_q = 0.0f;
    inverse_park_transform(pm);
    svpwm_calc(pm, v_dc, v_min);

    if (DWT_GetElapsedCycles(calibration.align_start_cycle) <
        (DWT_CPU_FREQ_HZ * ENCODER_ZERO_ALIGN_TIME_S))
    {
        return;
    }

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
        float delta = (float)((int32_t)raw - (int32_t)calibration.last_raw) *
                      AS5047P_RAD_PER_COUNT;
        if (delta > (TWO_PI * 0.5f))
        {
            delta -= TWO_PI;
        }
        else if (delta < -(TWO_PI * 0.5f))
        {
            delta += TWO_PI;
        }
        calibration.unwrapped_angle_offset_rad += delta;
        calibration.angle_offset_sum_rad +=
            calibration.unwrapped_angle_offset_rad;
        calibration.last_raw = raw;
        calibration.last_sample_sequence = sample_sequence;
        calibration.sample_count++;
    }

    if ((DWT_GetElapsedCycles(calibration.sample_start_cycle) <
         ((DWT_CPU_FREQ_HZ / 1000U) * ENCODER_ZERO_SAMPLE_TIME_MS)) ||
        (calibration.sample_count == 0U))
    {
        return;
    }

    {
        float average = calibration.sample_start_angle_rad +
            calibration.angle_offset_sum_rad / (float)calibration.sample_count;
        if (average < 0.0f)
        {
            average += TWO_PI;
        }
        else if (average >= TWO_PI)
        {
            average -= TWO_PI;
        }
        calibration.mechanical_zero_angle_rad = average;
        calibration.mechanical_zero_raw = (uint16_t)(
            average * AS5047P_COUNT_PER_RAD + 0.5f);
        if (calibration.mechanical_zero_raw > AS5047P_DATA_MASK)
        {
            calibration.mechanical_zero_raw = 0U;
        }
    }

    calibration.sampling = 0U;
    calibration.started = 0U;
    encoder_zero_calibration_requested = 0U;

#if (FOC_PHASE_AUTO_CALIBRATION_MODE == 0U)
    calibration.done = 1U;
    foc_pwm_output_disable();
#endif
}
#endif

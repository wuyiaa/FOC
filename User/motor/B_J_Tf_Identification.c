#include "B_J_Tf_Identification.h"
#include "common.h"
#include "flash_save.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* Four doublewords: tagged J/B/Tf followed by a checksum commit word. */
#define MOTOR_PARAMETERS_RECORD_TAG 0x4A425401UL

static uint32_t motor_parameters_record_checksum(const volatile uint64_t *record)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i;
    uint32_t byte;
    uint32_t bit;
    for (i = 0U; i < 3U; i++)
    {
        uint64_t word = record[i];
        for (byte = 0U; byte < 8U; byte++)
        {
            crc ^= (uint8_t)(word >> (byte * 8U));
            for (bit = 0U; bit < 8U; bit++)
            {
                crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
            }
        }
    }
    return ~crc;
}

static uint8_t motor_parameters_record_parse(const volatile uint64_t *record,
                                             float *J, float *B, float *Tf)
{
    uint32_t i;
    uint32_t bits;
    float values[3];
    for (i = 0U; i < MOTOR_PARAMETERS_RECORD_WORDS; i++)
    {
        if ((uint32_t)(record[i] >> 32U) != MOTOR_PARAMETERS_RECORD_TAG + i)
        {
            return 0U;
        }
    }
    if ((uint32_t)record[3] != motor_parameters_record_checksum(record))
    {
        return 0U;
    }
    for (i = 0U; i < 3U; i++)
    {
        bits = (uint32_t)record[i];
        memcpy(&values[i], &bits, sizeof(bits));
    }
    if ((!isfinite(values[0])) || (values[0] <= 0.0f) ||
        (!isfinite(values[1])) || (values[1] < 0.0f) ||
        (!isfinite(values[2])) || (values[2] < 0.0f))
    {
        return 0U;
    }
    *J = values[0];
    *B = values[1];
    *Tf = values[2];
    return 1U;
}

HAL_StatusTypeDef motor_parameters_flash_load(pmsm_motor_t *motor)
{
    const volatile uint64_t *records =
        (const volatile uint64_t *)FLASH_SAVE_ADDRESS;
    uint32_t i;
    uint8_t found = 0U;
    float J;
    float B;
    float Tf;
    if (motor == NULL)
    {
        return HAL_ERROR;
    }
    for (i = 0U; i + MOTOR_PARAMETERS_RECORD_WORDS <=
         FLASH_SAVE_RECORD_COUNT; i++)
    {
        if (motor_parameters_record_parse(&records[i], &J, &B, &Tf) != 0U)
        {
            motor->J = J;
            motor->B = B;
            motor->Tf = Tf;
            found = 1U;
        }
    }
    return (found != 0U) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef motor_parameters_flash_save(const pmsm_motor_t *motor)
{
    pmsm_motor_t previous = {0};
    uint64_t words[MOTOR_PARAMETERS_RECORD_WORDS];
    float values[3];
    uint32_t bits;
    uint32_t i;
    if ((motor == NULL) || (!isfinite(motor->J)) || (motor->J <= 0.0f) ||
        (!isfinite(motor->B)) || (motor->B < 0.0f) ||
        (!isfinite(motor->Tf)) || (motor->Tf < 0.0f))
    {
        return HAL_ERROR;
    }
    if ((motor_parameters_flash_load(&previous) == HAL_OK) &&
        (previous.J == motor->J) && (previous.B == motor->B) &&
        (previous.Tf == motor->Tf))
    {
        return HAL_OK;
    }
    values[0] = motor->J;
    values[1] = motor->B;
    values[2] = motor->Tf;
    for (i = 0U; i < 3U; i++)
    {
        memcpy(&bits, &values[i], sizeof(bits));
        words[i] = ((uint64_t)(MOTOR_PARAMETERS_RECORD_TAG + i) << 32U) | bits;
    }
    words[3] = ((uint64_t)(MOTOR_PARAMETERS_RECORD_TAG + 3U) << 32U) |
        motor_parameters_record_checksum(words);
    return flash_save_append(words, MOTOR_PARAMETERS_RECORD_WORDS);
}

/* Shared Flash storage calls this without decoding the motor record itself. */
uint8_t motor_parameters_flash_record_valid(const volatile uint64_t *record)
{
    float J;
    float B;
    float Tf;
    return motor_parameters_record_parse(record, &J, &B, &Tf);
}

/* Mechanical parameter identification, independent of speed feedback updates. */
motor_identification_t motor_identification;
volatile uint8_t motor_identification_requested = FOC_MOTOR_IDENTIFICATION_ENABLE;

void motor_identification_save_process(void)
{
    uint32_t primask;
    if ((motor_identification.stage != MOTOR_IDENT_DONE) ||
        (motor_identification.result_ready == 0U) ||
        (motor_identification.flash_save_attempted != 0U) ||
        (motor_identification_requested != 0U) ||
        (foc_pwm_output_is_enabled() != 0U))
    {
        return;
    }
    motor_identification.flash_save_attempted = 1U;
    if ((!isfinite(motor_identification.J)) || (motor_identification.J <= 0.0f) ||
        (!isfinite(motor_identification.B)) || (motor_identification.B < 0.0f) ||
        (!isfinite(motor_identification.Tf)) || (motor_identification.Tf < 0.0f))
    {
        motor_identification.stage = MOTOR_IDENT_FAILED;
        return;
    }
    /* Publish the three model parameters together before observers run again. */
    primask = __get_PRIMASK();
    __disable_irq();
    pm.motor.J = motor_identification.J;
    pm.motor.B = motor_identification.B;
    pm.motor.Tf = motor_identification.Tf;
    if (primask == 0U)
    {
        __enable_irq();
    }
    if (motor_parameters_flash_save(&pm.motor) == HAL_OK)
    {
        motor_identification.flash_saved = 1U;
    }
}

static void motor_ident_window_begin(motor_identification_window_t *window,
                                    float speed_rad_s, float iq_a)
{
    static const motor_identification_window_t empty = {0};
    *window = empty;
    window->speed_start_rad_s = speed_rad_s;
    window->speed_end_rad_s = speed_rad_s;
    window->last_iq_a = iq_a;
    window->started = 1U;
}

static void motor_ident_window_append(motor_identification_window_t *window,
                                     float speed_rad_s, float iq_a)
{
    window->speed_sum_rad_s += 0.5f * (window->speed_end_rad_s + speed_rad_s);
    window->iq_sum_a += 0.5f * (window->last_iq_a + iq_a);
    window->speed_end_rad_s = speed_rad_s;
    window->last_iq_a = iq_a;
    window->interval_count++;
}

static void motor_ident_window_finish(motor_identification_window_t *window)
{
    const float intervals = (float)window->interval_count;
    window->duration_s = intervals * MOTOR_SPEED_UPDATE_PERIOD_S;
    window->speed_mean_rad_s = window->speed_sum_rad_s / intervals;
    window->iq_mean_a = window->iq_sum_a / intervals;
    window->acceleration_mean_rad_s2 =
        (window->speed_end_rad_s - window->speed_start_rad_s) / window->duration_s;
    window->completed = 1U;
}

static uint8_t motor_ident_settled(float speed_rad_s, float target_rad_s)
{
    if (fabsf(speed_rad_s - target_rad_s) <= MOTOR_IDENT_SPEED_TOLERANCE_RAD_S)
    {
        motor_identification.stable_count++;
    }
    else
    {
        motor_identification.stable_count = 0U;
    }
    if (motor_identification.stable_count >= MOTOR_IDENT_SETTLE_TICKS)
    {
        motor_identification.stable_count = 0U;
        return 1U;
    }
    return 0U;
}

/* Capture both dynamic windows using the same measured speed interval. */
static uint8_t motor_ident_dynamic_window_update(
    motor_identification_window_t *window, float speed_rad_s, float iq_fdb,
    uint8_t accelerating)
{
    if (window->completed != 0U)
    {
        return 1U;
    }
    if (window->started != 0U)
    {
        motor_ident_window_append(window, speed_rad_s, iq_fdb);
    }
    else if ((accelerating != 0U) ?
        ((speed_rad_s >= MOTOR_IDENT_WINDOW_LOW_RAD_S) &&
         (speed_rad_s < MOTOR_IDENT_WINDOW_HIGH_RAD_S)) :
        ((speed_rad_s <= MOTOR_IDENT_WINDOW_HIGH_RAD_S) &&
         (speed_rad_s > MOTOR_IDENT_WINDOW_LOW_RAD_S)))
    {
        motor_ident_window_begin(window, speed_rad_s, iq_fdb);
    }
    if ((accelerating != 0U) ?
        (speed_rad_s >= MOTOR_IDENT_WINDOW_HIGH_RAD_S) :
        (speed_rad_s <= MOTOR_IDENT_WINDOW_LOW_RAD_S))
    {
        if (window->interval_count == 0U)
        {
            return 0U;
        }
        motor_ident_window_finish(window);
    }
    return 1U;
}

static uint8_t motor_ident_friction_calculate(void)
{
    const float kt = motor_identification.torque_constant_nm_a;
    const float speed_difference =
        motor_identification.steady_high.speed_mean_rad_s -
        motor_identification.steady_low.speed_mean_rad_s;

    if (!(speed_difference > 0.001f))
    {
        return 0U;
    }
    motor_identification.B = kt *
        (motor_identification.steady_high.iq_mean_a -
         motor_identification.steady_low.iq_mean_a) / speed_difference;
    motor_identification.Tf = kt * motor_identification.steady_low.iq_mean_a -
        motor_identification.B * motor_identification.steady_low.speed_mean_rad_s;
    return (uint8_t)(isfinite(motor_identification.B) &&
                     isfinite(motor_identification.Tf));
}

static void motor_ident_results_calculate(void)
{
    const float acceleration_difference =
        motor_identification.acceleration.acceleration_mean_rad_s2 -
        motor_identification.deceleration.acceleration_mean_rad_s2;
    const float dynamic_speed_difference =
        motor_identification.acceleration.speed_mean_rad_s -
        motor_identification.deceleration.speed_mean_rad_s;
    const float dynamic_torque_difference =
        motor_identification.torque_constant_nm_a *
        (motor_identification.acceleration.iq_mean_a -
         motor_identification.deceleration.iq_mean_a);

    if (!(motor_identification.acceleration.acceleration_mean_rad_s2 > 0.0f) ||
        !(motor_identification.deceleration.acceleration_mean_rad_s2 < 0.0f))
    {
        motor_identification.stage = MOTOR_IDENT_FAILED;
        return;
    }
    /* Same direction cancels Tf; retain measured B * mean-speed difference. */
    motor_identification.J = (dynamic_torque_difference -
        motor_identification.B * dynamic_speed_difference) / acceleration_difference;
    if (isfinite(motor_identification.J))
    {
        motor_identification.result_ready = 1U;
        motor_identification.stage = MOTOR_IDENT_DONE;
    }
    else
    {
        motor_identification.stage = MOTOR_IDENT_FAILED;
    }
}

void J_B_Tf_Indent(motor_speed_t *speed, float iq_fdb,
                  float torque_constant_nm_a, uint8_t output_enabled)
{
    const float speed_rad_s = speed->mechanical_rad_s_fdb;
    motor_identification_window_t *window;

    if (output_enabled == 0U)
    {
        /* Retain completed observations while paused; discard partial progress. */
        if ((motor_identification.stage != MOTOR_IDENT_DONE) &&
            (motor_identification.stage != MOTOR_IDENT_FAILED))
        {
            motor_identification.stage = MOTOR_IDENT_IDLE;
            motor_identification.stable_count = 0U;
        }
        speed->mechanical_rad_s_ref = 0.0f;
        return;
    }
    if ((motor_identification.stage == MOTOR_IDENT_IDLE) ||
        (motor_identification.stage == MOTOR_IDENT_DONE) ||
        (motor_identification.stage == MOTOR_IDENT_FAILED))
    {
        static const motor_identification_t empty = {MOTOR_IDENT_IDLE};
        motor_identification = empty;
        motor_identification.torque_constant_nm_a = torque_constant_nm_a;
        motor_identification.stage = (torque_constant_nm_a > 0.0f) ?
            MOTOR_IDENT_SETTLE_INITIAL : MOTOR_IDENT_FAILED;
    }

    switch (motor_identification.stage)
    {
        case MOTOR_IDENT_SETTLE_INITIAL:
            speed->mechanical_rad_s_ref = MOTOR_IDENT_LOW_SPEED_RAD_S;
            if (motor_ident_settled(speed_rad_s, MOTOR_IDENT_LOW_SPEED_RAD_S) != 0U)
            {
                motor_ident_window_begin(&motor_identification.steady_low,
                                        speed_rad_s, iq_fdb);
                motor_identification.stage = MOTOR_IDENT_AVERAGE_LOW;
            }
            break;

        case MOTOR_IDENT_AVERAGE_LOW:
            speed->mechanical_rad_s_ref = MOTOR_IDENT_LOW_SPEED_RAD_S;
            if (fabsf(speed_rad_s - MOTOR_IDENT_LOW_SPEED_RAD_S) >
                MOTOR_IDENT_SPEED_TOLERANCE_RAD_S)
            {
                motor_identification.stage = MOTOR_IDENT_SETTLE_INITIAL;
                break;
            }
            window = &motor_identification.steady_low;
            motor_ident_window_append(window, speed_rad_s, iq_fdb);
            if (window->interval_count >= MOTOR_IDENT_AVERAGE_TICKS)
            {
                motor_ident_window_finish(window);
                motor_identification.stage = MOTOR_IDENT_SETTLE_HIGH;
            }
            break;

        case MOTOR_IDENT_SETTLE_HIGH:
            speed->mechanical_rad_s_ref = MOTOR_IDENT_HIGH_SPEED_RAD_S;
            if (motor_ident_settled(speed_rad_s, MOTOR_IDENT_HIGH_SPEED_RAD_S) != 0U)
            {
                motor_ident_window_begin(&motor_identification.steady_high,
                                        speed_rad_s, iq_fdb);
                motor_identification.stage = MOTOR_IDENT_AVERAGE_HIGH;
            }
            break;

        case MOTOR_IDENT_AVERAGE_HIGH:
            speed->mechanical_rad_s_ref = MOTOR_IDENT_HIGH_SPEED_RAD_S;
            if (fabsf(speed_rad_s - MOTOR_IDENT_HIGH_SPEED_RAD_S) >
                MOTOR_IDENT_SPEED_TOLERANCE_RAD_S)
            {
                motor_identification.stage = MOTOR_IDENT_SETTLE_HIGH;
                break;
            }
            window = &motor_identification.steady_high;
            motor_ident_window_append(window, speed_rad_s, iq_fdb);
            if (window->interval_count >= MOTOR_IDENT_AVERAGE_TICKS)
            {
                motor_ident_window_finish(window);
                motor_identification.stage = (motor_ident_friction_calculate() != 0U) ?
                    MOTOR_IDENT_RETURN_LOW : MOTOR_IDENT_FAILED;
            }
            break;

        case MOTOR_IDENT_RETURN_LOW:
            speed->mechanical_rad_s_ref = MOTOR_IDENT_LOW_SPEED_RAD_S;
            if (motor_ident_settled(speed_rad_s, MOTOR_IDENT_LOW_SPEED_RAD_S) != 0U)
            {
                motor_identification.stage = MOTOR_IDENT_ACCELERATE;
            }
            break;

        case MOTOR_IDENT_ACCELERATE:
            speed->mechanical_rad_s_ref +=
                MOTOR_IDENT_ACCELERATION_RAD_S2 * MOTOR_SPEED_UPDATE_PERIOD_S;
            if (speed->mechanical_rad_s_ref > MOTOR_IDENT_HIGH_SPEED_RAD_S)
            {
                speed->mechanical_rad_s_ref = MOTOR_IDENT_HIGH_SPEED_RAD_S;
            }
            window = &motor_identification.acceleration;
            if (motor_ident_dynamic_window_update(window, speed_rad_s, iq_fdb, 1U) == 0U)
            {
                motor_identification.stage = MOTOR_IDENT_FAILED;
            }
            else if ((window->completed != 0U) &&
                     (speed->mechanical_rad_s_ref >= MOTOR_IDENT_HIGH_SPEED_RAD_S))
            {
                motor_identification.stage = MOTOR_IDENT_SETTLE_BEFORE_DECELERATE;
            }
            break;

        case MOTOR_IDENT_SETTLE_BEFORE_DECELERATE:
            speed->mechanical_rad_s_ref = MOTOR_IDENT_HIGH_SPEED_RAD_S;
            if (motor_ident_settled(speed_rad_s, MOTOR_IDENT_HIGH_SPEED_RAD_S) != 0U)
            {
                motor_identification.stage = MOTOR_IDENT_DECELERATE;
            }
            break;

        case MOTOR_IDENT_DECELERATE:
            speed->mechanical_rad_s_ref +=
                MOTOR_IDENT_DECELERATION_RAD_S2 * MOTOR_SPEED_UPDATE_PERIOD_S;
            if (speed->mechanical_rad_s_ref < MOTOR_IDENT_LOW_SPEED_RAD_S)
            {
                speed->mechanical_rad_s_ref = MOTOR_IDENT_LOW_SPEED_RAD_S;
            }
            window = &motor_identification.deceleration;
            if (motor_ident_dynamic_window_update(window, speed_rad_s, iq_fdb, 0U) == 0U)
            {
                motor_identification.stage = MOTOR_IDENT_FAILED;
            }
            else if ((window->completed != 0U) &&
                     (speed->mechanical_rad_s_ref <= MOTOR_IDENT_LOW_SPEED_RAD_S))
            {
                motor_ident_results_calculate();
            }
            break;

        default: /* DONE/FAILED: retain observations until a new run. */
            break;
    }

    if ((motor_identification.stage == MOTOR_IDENT_DONE) ||
        (motor_identification.stage == MOTOR_IDENT_FAILED))
    {
        speed->mechanical_rad_s_ref = 0.0f;
    }
}

#include "motor_speed.h"

#include <stddef.h>

#include "dwt_time.h"
#include "motor_config.h"

/* A 1 ms difference window reduces 14-bit encoder quantization at 20 kHz. */
#define MOTOR_SPEED_WINDOW_CYCLES  (DWT_CPU_FREQ_HZ / 1000UL)
#define MOTOR_SPEED_STALE_CYCLES   (DWT_CPU_FREQ_HZ / 100UL)
#define MOTOR_SPEED_COUNTS_PER_REV (AS5047P_DATA_MASK + 1U)
#define MOTOR_SPEED_HALF_REV       (MOTOR_SPEED_COUNTS_PER_REV / 2U)

void motor_speed_update(motor_speed_t *speed,
                        const as5047p_sample_t *sample,
                        uint8_t pole_pairs,
                        uint32_t now_cycle)
{
    uint32_t elapsed_cycles;
    int32_t delta_count;

    if ((speed == NULL) || (sample == NULL))
    {
        return;
    }

    if ((speed->initialized != 0U) &&
        ((uint32_t)(now_cycle - speed->last_sample_cycle) >=
         MOTOR_SPEED_STALE_CYCLES))
    {
        speed->mechanical_rad_s = 0.0f;
        speed->electrical_rad_s = 0.0f;
        speed->initialized = 0U;

        /* A still-published old sample must not restart the estimator. */
        if (sample->sequence == speed->last_sequence)
        {
            return;
        }
    }

    if (sample->valid == 0U)
    {
        return;
    }

    if (speed->initialized == 0U)
    {
        speed->last_raw = sample->raw;
        speed->last_sample_cycle = sample->cycle;
        speed->last_sequence = sample->sequence;
        speed->initialized = 1U;
        return;
    }

    if (sample->sequence == speed->last_sequence)
    {
        return;
    }

    elapsed_cycles = (uint32_t)(sample->cycle - speed->last_sample_cycle);
    if (elapsed_cycles < MOTOR_SPEED_WINDOW_CYCLES)
    {
        return;
    }

    /* The shortest signed count difference handles the 0/16383 wrap. */
    delta_count = (int32_t)sample->raw - (int32_t)speed->last_raw;
    if (delta_count > (int32_t)MOTOR_SPEED_HALF_REV)
    {
        delta_count -= (int32_t)MOTOR_SPEED_COUNTS_PER_REV;
    }
    else if (delta_count < -(int32_t)MOTOR_SPEED_HALF_REV)
    {
        delta_count += (int32_t)MOTOR_SPEED_COUNTS_PER_REV;
    }

    /* Physical CCW speed is positive, independent of FOC phase/angle setup. */
    speed->mechanical_rad_s =
        ((float)delta_count * MOTOR_SPEED_DIRECTION * AS5047P_RAD_PER_COUNT *
         (float)DWT_CPU_FREQ_HZ) /
        (float)elapsed_cycles;
    speed->electrical_rad_s =
        speed->mechanical_rad_s * (float)pole_pairs;

    speed->last_raw = sample->raw;
    speed->last_sample_cycle = sample->cycle;
    speed->last_sequence = sample->sequence;
}

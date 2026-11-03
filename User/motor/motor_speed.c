#include "motor_speed.h"

#include <stddef.h>

//#include "dwt_time.h"
#include "motor_config.h"
#include "common.h"

/* Old DWT window disabled; TIM7 now schedules fixed 500 us updates. */
//#define MOTOR_SPEED_WINDOW_CYCLES  (DWT_CPU_FREQ_HZ / 1000UL)
//#define MOTOR_SPEED_STALE_CYCLES   (DWT_CPU_FREQ_HZ / 100UL)
#define MOTOR_SPEED_UPDATE_PERIOD_S  0.0005f
#define MOTOR_SPEED_FILTER_CUTOFF_HZ 100.0f
#define MOTOR_SPEED_FILTER_ALPHA \
    ((TWO_PI * MOTOR_SPEED_FILTER_CUTOFF_HZ * MOTOR_SPEED_UPDATE_PERIOD_S) / \
     (1.0f + TWO_PI * MOTOR_SPEED_FILTER_CUTOFF_HZ * MOTOR_SPEED_UPDATE_PERIOD_S))
#define MOTOR_SPEED_COUNTS_PER_REV (AS5047P_DATA_MASK + 1U)
#define MOTOR_SPEED_HALF_REV       (MOTOR_SPEED_COUNTS_PER_REV / 2U)

void motor_speed_update(motor_speed_t *speed,
                        const as5047p_sample_t *sample,
                        uint8_t pole_pairs)
{
    //uint32_t elapsed_cycles;
    uint16_t raw_count;
    int32_t delta_count;

    if ((speed == NULL) || (sample == NULL))
    {
        return;
    }

    //if ((speed->initialized != 0U) &&
    //    ((uint32_t)(now_cycle - speed->last_sample_cycle) >=
    //     MOTOR_SPEED_STALE_CYCLES))
    //{
    //    speed->mechanical_rad_s = 0.0f;
    //    speed->electrical_rad_s = 0.0f;
    //    speed->initialized = 0U;
    //    if (sample->sequence == speed->last_sequence)
    //    {
    //        return;
    //    }
    //}
    //if (sample->valid == 0U)
    //{
    //    return;
    //}

    /* Only this aligned halfword is needed; ADC may publish a newer sample
       during the calculation without changing this local angle. */
    raw_count = sample->raw;

    if (speed->initialized == 0U)
    {
        speed->last_raw = raw_count;
        //speed->last_sample_cycle = sample->cycle;
        //speed->last_sequence = sample->sequence;
        speed->initialized = 1U;
        return;
    }

    //if (sample->sequence == speed->last_sequence)
    //{
    //    return;
    //}
    //elapsed_cycles = (uint32_t)(sample->cycle - speed->last_sample_cycle);
    //if (elapsed_cycles < MOTOR_SPEED_WINDOW_CYCLES)
    //{
    //    return;
    //}

    /* The shortest signed count difference handles the 0/16383 wrap. */
    delta_count = (int32_t)raw_count - (int32_t)speed->last_raw;
    if (delta_count > (int32_t)MOTOR_SPEED_HALF_REV)
    {
        delta_count -= (int32_t)MOTOR_SPEED_COUNTS_PER_REV;
    }
    else if (delta_count < -(int32_t)MOTOR_SPEED_HALF_REV)
    {
        delta_count += (int32_t)MOTOR_SPEED_COUNTS_PER_REV;
    }

    /* Physical CCW speed is positive, independent of FOC phase/angle setup. */
    //speed->mechanical_rad_s =
    //    ((float)delta_count * MOTOR_SPEED_DIRECTION * AS5047P_RAD_PER_COUNT *
    //     (float)DWT_CPU_FREQ_HZ) / (float)elapsed_cycles;
		
    speed->mechanical_rad_s = (float)delta_count * MOTOR_SPEED_DIRECTION *
        AS5047P_RAD_PER_COUNT / MOTOR_SPEED_UPDATE_PERIOD_S;
    speed->electrical_rad_s = speed->mechanical_rad_s * (float)pole_pairs;

    speed->mechanical_rad_s_filtered += MOTOR_SPEED_FILTER_ALPHA *
        (speed->mechanical_rad_s - speed->mechanical_rad_s_filtered);
		speed->electrical_rad_s_filtered =  speed->mechanical_rad_s_filtered * (float)pole_pairs;
		
    speed->last_raw = raw_count;
    //speed->last_sample_cycle = sample->cycle;
    //speed->last_sequence = sample->sequence;
}

/* TIM7 update interrupt: 160 MHz / 160 / 500 = 2 kHz. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        motor_speed_update(&pm.speed, &pm.encoder_foc_sample, pm.motor.pn);
#if ((FOC_ENCODER_ZERO_CALIBRATION_MODE == 0U) && \
     (FOC_PHASE_AUTO_CALIBRATION_MODE == 0U))
				pm.speed.mechanical_rad_s_ref = 50.0f;
        foc_speed_pid_update(&pm, MOTOR_SPEED_UPDATE_PERIOD_S);
#endif
    }
}

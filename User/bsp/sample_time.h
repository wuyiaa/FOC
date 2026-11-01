#ifndef USER_BSP_SAMPLE_TIME_H
#define USER_BSP_SAMPLE_TIME_H

#include <stdint.h>

typedef struct
{
    uint32_t timer_count;
    uint8_t counting_down;
} sample_time_point_t;

/* Call after MX_TIM1_Init(); does not change or restart TIM1. */
void sample_time_init(void);

/* Read a time point in TIM1 ticks, unfolded across both count directions. */
uint32_t sample_start_time(void);

/* Convert two time points to elapsed microseconds, modulo one PWM period.
   The interval must be shorter than that period (currently 50 us). */
float sample_elapsed_time_us(uint32_t start_count, uint32_t end_count);

/* ADC diagnostics: also capture raw CNT and direction at the same time point. */
uint32_t sample_time_point_get(sample_time_point_t *point);

/* Diagnostics: also save elapsed ticks to the supplied variable. */
float sample_elapsed_time_record_us(uint32_t start_count, uint32_t end_count,
                                    volatile uint32_t *elapsed_ticks);

#endif /* USER_BSP_SAMPLE_TIME_H */

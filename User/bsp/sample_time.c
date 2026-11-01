#include "sample_time.h"

#include "main.h"

static float timer_tick_us;

void sample_time_init(void)
{
    uint32_t timer_clock_hz = HAL_RCC_GetPCLK2Freq();

    if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0U)
    {
        timer_clock_hz *= 2U;
    }
    timer_tick_us = 1000000.0f * (float)(TIM1->PSC + 1U) /
                    (float)timer_clock_hz;
}

uint32_t sample_start_time(void)
{
    sample_time_point_t point;

    return sample_time_point_get(&point);
}

uint32_t sample_time_point_get(sample_time_point_t *point)
{
    uint32_t direction_before;
    uint32_t direction_after;
    uint32_t count;

    /* Retry only if TIM1 reverses direction while CNT is being read. */
    do
    {
        direction_before = TIM1->CR1 & TIM_CR1_DIR;
        count = TIM1->CNT;
        direction_after = TIM1->CR1 & TIM_CR1_DIR;
    } while (direction_before != direction_after);

    point->timer_count = count;
    point->counting_down = (direction_after != 0U) ? 1U : 0U;

    return (direction_after != 0U) ? (2U * TIM1->ARR - count) : count;
}

float sample_elapsed_time_us(uint32_t start_count, uint32_t end_count)
{
    uint32_t elapsed_ticks;

    return sample_elapsed_time_record_us(start_count, end_count, &elapsed_ticks);
}

float sample_elapsed_time_record_us(uint32_t start_count, uint32_t end_count,
                                    volatile uint32_t *elapsed_ticks)
{
    uint32_t period = 2U * TIM1->ARR;
    uint32_t ticks = (end_count >= start_count) ?
        (end_count - start_count) : (period - start_count + end_count);

    *elapsed_ticks = ticks;
    return (float)ticks * timer_tick_us;
}

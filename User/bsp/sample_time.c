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

void foc_callback_time_measure_begin(pmsm_t *pm)
{
    ADC_TIME_MARK_GPIO_Port->BSRR = (uint32_t)ADC_TIME_MARK_Pin << 16U;
    pm->foc_measure.start_phase = sample_start_time();
    pm->foc_measure.done = 0U;
}

void foc_callback_time_measure_end(pmsm_t *pm)
{
    uint32_t end_phase = sample_start_time();
    uint32_t start_phase;
    uint32_t period;

    ADC_TIME_MARK_GPIO_Port->BSRR = ADC_TIME_MARK_Pin;
    start_phase = pm->foc_measure.start_phase;
    period = 2U * TIM1->ARR;
    pm->foc_measure.end_phase = end_phase;
    if (period == 0U)
    {
        pm->foc_measure.done = 0U;
        return;
    }

    pm->foc_measure.foc_time_us = sample_elapsed_time_record_us(
        start_phase, end_phase, &pm->foc_measure.elapsed_ticks);
    pm->foc_measure.done = 1U;
}

void foc_adc_time_measure_update(pmsm_t *pm)
{
    sample_time_point_t point;
    uint32_t period;
    uint32_t phase;
    uint32_t trigger;

    if (pm->adc_measure.active == 0U)
    {
        return;
    }

    /* Keep the raw CNT/direction diagnostics from this same time point. */
    phase = sample_time_point_get(&point);

    /* Center-aligned mode 2: CC4 compare event on the up-count.
       This is a modulo-one-PWM-period latency, not pure ADC conversion time.
       A delay of a full PWM period or more cannot be detected here. */
    period = 2U * TIM1->ARR;
    trigger = TIM1->CCR4;
    if ((period == 0U) || (trigger > TIM1->ARR))
    {
        pm->adc_measure.done = 0U;
        return;
    }
    pm->adc_measure.timer_count = point.timer_count;
    pm->adc_measure.timer_counting_down = point.counting_down;
    pm->adc_measure.sample_time_us = sample_elapsed_time_record_us(
        trigger, phase, &pm->adc_measure.elapsed_ticks);
    pm->adc_measure.done = 1U;
}

void foc_adc_time_measure_arm(pmsm_t *pm)
{
    pm->adc_measure.sample_time_us = 0.0f;
    pm->adc_measure.elapsed_ticks = 0U;
    pm->adc_measure.timer_count = 0U;
    pm->adc_measure.timer_counting_down = 0U;
    pm->adc_measure.done = 0U;

    pm->foc_measure.start_phase = 0U;
    pm->foc_measure.end_phase = 0U;
    pm->foc_measure.elapsed_ticks = 0U;
    pm->foc_measure.foc_time_us = 0.0f;
    pm->foc_measure.done = 0U;

    /* Only disable the software timing IRQ; hardware ADC triggering remains. */
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC4);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC4);
    pm->adc_measure.active = 1U;
}

#include "common.h"
#include "voltage_sample.h"

//static uint8_t foc_current_trip_reached(const pmsm_foc_t *foc)
//{
//    const float current_limit = FOC_CURRENT_TRIP_LIMIT_A;

//    return (uint8_t)((foc->ia.reconstructed >= current_limit) ||
//                     (foc->ia.reconstructed <= -current_limit) ||
//                     (foc->ib.reconstructed >= current_limit) ||
//                     (foc->ib.reconstructed <= -current_limit) ||
//                     (foc->ic.reconstructed >= current_limit) ||
//                     (foc->ic.reconstructed <= -current_limit));
//}

static float adc_timer_tick_us;

/* Map the up/down counter to a phase in [0, 2 * ARR]. */
static uint32_t foc_timer_phase_get(void)
{
    uint32_t direction_before;
    uint32_t direction_after;
    uint32_t count;

    do
    {
        direction_before = TIM1->CR1 & TIM_CR1_DIR;
        count = TIM1->CNT;
        direction_after = TIM1->CR1 & TIM_CR1_DIR;
    } while (direction_before != direction_after);

    return (direction_after != 0U) ? (2U * TIM1->ARR - count) : count;
}

static void foc_callback_time_measure_begin(pmsm_t *pm)
{
    ADC_TIME_MARK_GPIO_Port->BSRR = (uint32_t)ADC_TIME_MARK_Pin << 16U;
    pm->foc_measure.start_phase = foc_timer_phase_get();
    pm->foc_measure.done = 0U;
}

static void foc_callback_time_measure_end(pmsm_t *pm)
{
    uint32_t end_phase = foc_timer_phase_get();
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

    /* Modulo one PWM period: full additional periods cannot be detected. */
    pm->foc_measure.elapsed_ticks = (end_phase >= start_phase) ?
        (end_phase - start_phase) : (period - start_phase + end_phase);
    pm->foc_measure.execution_time_us =
        (float)pm->foc_measure.elapsed_ticks * adc_timer_tick_us;
    pm->foc_measure.done = 1U;
}

static void foc_adc_time_measure_update(pmsm_t *pm)
{
    uint32_t direction_before;
    uint32_t direction_after;
    uint32_t count;
    uint32_t period;
    uint32_t phase;
    uint32_t trigger;

    if (pm->adc_measure.active == 0U)
    {
        return;
    }

    /* PB10 is held low by the caller throughout this ADC1 callback.
       Retry the CNT snapshot if the timer reverses while it is read. */
    do
    {
        direction_before = TIM1->CR1 & TIM_CR1_DIR;
        count = TIM1->CNT;
        direction_after = TIM1->CR1 & TIM_CR1_DIR;
    } while (direction_before != direction_after);

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
    phase = (direction_after != 0U) ? (period - count) : count;
    pm->adc_measure.elapsed_ticks =
        (phase >= trigger) ? (phase - trigger) : (period - trigger + phase);
    pm->adc_measure.timer_count = count;
    pm->adc_measure.timer_counting_down = (direction_after != 0U) ? 1U : 0U;
    pm->adc_measure.sample_time_us =
        (float)pm->adc_measure.elapsed_ticks * adc_timer_tick_us;
    pm->adc_measure.done = 1U;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        /* PB10 low pulse spans ADC1 callback processing, including SPI/FOC. */
        foc_callback_time_measure_begin(&pm);
        foc_adc_time_measure_update(&pm);

        if (pm.adc.offset_ready == 0U)
        {
            voltage_sample_bus_update();
            foc_callback_time_measure_end(&pm);
            return;
        }

        foc_adc_sample(&pm);
        foc_current_sample_process(&pm);
        voltage_sample_bus_update();

//        /* Complete one SPI frame and update this FOC cycle's angle sample. */
        (void)as5047p_read_blocking(&pm.encoder, &pm.encoder_foc_sample);

        foc_electrical_angle_update(&pm);

//        motor_speed_update(&pm.speed, &pm.encoder_foc_sample,
//                           pm.motor.pn, DWT->CYCCNT);

				
        foc_current_calc(&pm);
#if ((FOC_ENCODER_ZERO_CALIBRATION_MODE == 0U) && \
     (FOC_PHASE_AUTO_CALIBRATION_MODE == 0U))
//				pm.foc.id_ref = 0.0f;
//				pm.foc.iq_ref = 0.5f;
//        foc_current_pid_update(&pm, PWM_PERIOD_S);
#endif
				pm.foc.Ud_ref = 0.0f;
				pm.foc.Uq_ref = 13.66f;
        foc_voltage_calc(&pm, voltage_sample.vbus_v, pm.svpwm.vref2_min);//voltage_sample.vbus_v pm.svpwm.vref2_min
        foc_pwm_output_update(&pm);
        foc_callback_time_measure_end(&pm);
    }
}

void foc_adc_time_measure_arm(pmsm_t *pm)
{
    uint32_t timer_clock_hz = HAL_RCC_GetPCLK2Freq();

    /* STM32G4 timer clock doubles when the APB2 prescaler is not one. */
    if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0U)
    {
        timer_clock_hz *= 2U;
    }
    adc_timer_tick_us = 1000000.0f * (float)(TIM1->PSC + 1U) /
                        (float)timer_clock_hz;
    pm->adc_measure.sample_time_us = 0.0f;
    pm->adc_measure.elapsed_ticks = 0U;
    pm->adc_measure.timer_count = 0U;
    pm->adc_measure.timer_counting_down = 0U;
    pm->adc_measure.done = 0U;

    pm->foc_measure.start_phase = 0U;
    pm->foc_measure.end_phase = 0U;
    pm->foc_measure.elapsed_ticks = 0U;
    pm->foc_measure.execution_time_us = 0.0f;
    pm->foc_measure.done = 0U;

    /* Only disable the software timing IRQ; hardware ADC triggering remains. */
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC4);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC4);
    pm->adc_measure.active = 1U;
}

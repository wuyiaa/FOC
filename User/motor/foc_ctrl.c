#include "common.h"
#include "voltage_sample.h"

static uint8_t foc_current_trip_reached(const pmsm_foc_t *foc)
{
    const float current_limit = FOC_CURRENT_TRIP_LIMIT_A;

    return (uint8_t)((foc->ia.reconstructed >= current_limit) ||
                     (foc->ia.reconstructed <= -current_limit) ||
                     (foc->ib.reconstructed >= current_limit) ||
                     (foc->ib.reconstructed <= -current_limit) ||
                     (foc->ic.reconstructed >= current_limit) ||
                     (foc->ic.reconstructed <= -current_limit));
}

static float adc_timer_tick_ms;

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

    /* PB10 / CN9 pin 4 rising edge marks the following CNT read.
       On a direction-change retry, the last rising edge is the valid one.
       The pulse width is timing-code execution, not ADC conversion time. */
    do
    {
        ADC_TIME_MARK_GPIO_Port->BSRR = (uint32_t)ADC_TIME_MARK_Pin << 16U;
        direction_before = TIM1->CR1 & TIM_CR1_DIR;
        ADC_TIME_MARK_GPIO_Port->BSRR = ADC_TIME_MARK_Pin;
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
        ADC_TIME_MARK_GPIO_Port->BSRR = (uint32_t)ADC_TIME_MARK_Pin << 16U;
        return;
    }
    phase = (direction_after != 0U) ? (period - count) : count;
    pm->adc_measure.elapsed_ticks =
        (phase >= trigger) ? (phase - trigger) : (period - trigger + phase);
    pm->adc_measure.timer_count = count;
    pm->adc_measure.timer_counting_down = (direction_after != 0U) ? 1U : 0U;
    pm->adc_measure.sample_time_ms =
        (float)pm->adc_measure.elapsed_ticks * adc_timer_tick_ms;
    pm->adc_measure.done = 1U;
    ADC_TIME_MARK_GPIO_Port->BSRR = (uint32_t)ADC_TIME_MARK_Pin << 16U;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        foc_adc_time_measure_update(&pm);

        if (pm.adc.offset_ready == 0U)
        {
            voltage_sample_update();
            return;
        }

        /* Read injected VBUS and DMA voltages after the timing marker so processing
           does not add to the measured ADC conversion latency. */
        voltage_sample_update();

        /* FOC uses one coherent copy of the most recently published angle. */
        (void)as5047p_latest_get(&pm.encoder, &pm.encoder_foc_sample);
        motor_speed_update(&pm.speed, &pm.encoder_foc_sample,
                           pm.motor.pn, DWT->CYCCNT);

//        if (as5047p_protection_update(&pm.encoder,
//                                      &pm.encoder_foc_sample) != 0U)
//        {
//            /* Latch the encoder fault and immediately gate CH1-CH3/CH1N-CH3N. */
//            __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
//            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
//        }

        foc_adc_sample(&pm);
        foc_current_sample_process(&pm);

        /* Keep a hard phase-current trip as an independent safety backup. */
        if ((pm.foc.current_limit_fault != 0U) ||
            (foc_current_trip_reached(&pm.foc) != 0U))
        {
            pm.foc.current_limit_fault = 1U;
            foc_pwm_output_disable();
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
            (void)as5047p_read_start_dma(&pm.encoder);
            return;
        }

        foc_current_calc(&pm);
#if ((FOC_ENCODER_ZERO_CALIBRATION_MODE == 0U) && \
     (FOC_PHASE_AUTO_CALIBRATION_MODE == 0U))
				pm.foc.id_ref = 0.0f;
				pm.foc.iq_ref = 0.1f;
        foc_current_pid_update(&pm, PWM_PERIOD_S);
#endif
//				pm.foc.Ud_ref = 0.0f;
//				pm.foc.Uq_ref = 13.2f;
        foc_voltage_calc(&pm, voltage_sample.vbus_v, pm.svpwm.vref2_min);
        foc_pwm_output_update(&pm);
        /* Start the next pipelined encoder DMA frame, then leave immediately. */
        (void)as5047p_read_start_dma(&pm.encoder);
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
    adc_timer_tick_ms = 1000.0f * (float)(TIM1->PSC + 1U) /
                        (float)timer_clock_hz;
    pm->adc_measure.sample_time_ms = 0.0f;
    pm->adc_measure.elapsed_ticks = 0U;
    pm->adc_measure.timer_count = 0U;
    pm->adc_measure.timer_counting_down = 0U;
    pm->adc_measure.done = 0U;

    /* Only disable the software timing IRQ; hardware ADC triggering remains. */
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC4);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC4);
    pm->adc_measure.active = 1U;
}

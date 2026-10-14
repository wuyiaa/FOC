#include "common.h"

static uint8_t foc_current_trip_reached(const pmsm_foc_t *foc)
{
    const float current_limit = FOC_CURRENT_TRIP_LIMIT_A;

    return (uint8_t)((foc->i_a >= current_limit) ||
                     (foc->i_a <= -current_limit) ||
                     (foc->i_b >= current_limit) ||
                     (foc->i_b <= -current_limit) ||
                     (foc->i_c >= current_limit) ||
                     (foc->i_c <= -current_limit));
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        if (pm.adc.offset_ready == 0U)
        {
            return;
        }

        if (pm.adc_measure.active != 0U)
        {
            pm.adc_measure.sample_time_ms =
                DWT_GetDeltaT(&pm.adc_measure.time_last) * 1000.0f;

            pm.adc_measure.active = 0U;
            pm.adc_measure.done = 1U;
            __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC4);
        }

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
//				pm.foc.i_d_ref = 0;
//				pm.foc.i_q_ref = 5.0f;
        foc_current_pid_update(&pm, PWM_PERIOD_S);
#endif
				pm.foc.u_d = 0.0f;
				pm.foc.u_q = 5.0f;
        foc_voltage_calc(&pm, FOC_DC_BUS_VOLTAGE_V, pm.svpwm.vref2_min);
        foc_pwm_output_update(&pm);
        /* Start the next pipelined encoder DMA frame, then leave immediately. */
        (void)as5047p_read_start_dma(&pm.encoder);
    }
}

void foc_adc_time_measure_arm(pmsm_t *pm)
{
    pm->adc_measure.time_last = 0U;
    pm->adc_measure.sample_time_ms = 0.0f;
    pm->adc_measure.active = 0U;
    pm->adc_measure.done = 0U;

    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC4);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim->Instance == TIM1) &&
        (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) &&
        (pm.adc_measure.active == 0U))
    {
        pm.adc_measure.time_last = DWT->CYCCNT;
        pm.adc_measure.active = 1U;
    }
}//(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim) == 0U)

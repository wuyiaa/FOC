#include "common.h"

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

        foc_adc_sample(&pm);
        foc_current_sample_process(&pm);
        foc_current_calc(&pm);
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


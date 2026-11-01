#include "common.h"
#include "voltage_sample.h"
#include "sample_time.h"

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

static void foc_callback_time_measure_begin(pmsm_t *pm)
{
    ADC_TIME_MARK_GPIO_Port->BSRR = (uint32_t)ADC_TIME_MARK_Pin << 16U;
    pm->foc_measure.start_phase = sample_start_time();
    pm->foc_measure.done = 0U;
}

static void foc_callback_time_measure_end(pmsm_t *pm)
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

static void foc_adc_time_measure_update(pmsm_t *pm)
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

/*
- PC3  下降沿：ADC 开始采样的触发时刻。							上降沿：向下计数越过 3900：PC3 从低变高，产生上升沿。
- PB10 下降沿：ADC 转换完成后，CPU 进入回调的标记。 上降沿：回调完成。
- 两者下降沿之间的间隔：ADC 采样、转换及中断响应时间。
- PC3 低电平时间：无意义
- PB10低电平时间：回调执行时间
- 只有下管的导通时间覆盖了adc的采样起始点（3900）以及adc的结束点的这段时间，电流采样才不会失真
*/
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
				pm.foc.id_ref = 0.0f;
				pm.foc.iq_ref = 0.5f;
        foc_current_pid_update(&pm, PWM_PERIOD_S);
#endif
//				pm.foc.Ud_ref = 0.0f;
//				pm.foc.Uq_ref = 13.66f;
        foc_voltage_calc(&pm, voltage_sample.vbus_v, pm.svpwm.vref2_min);//voltage_sample.vbus_v pm.svpwm.vref2_min
        foc_pwm_output_update(&pm);
        foc_callback_time_measure_end(&pm);
    }
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

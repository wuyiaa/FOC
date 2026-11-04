#include "foc_ctrl.h"
#include "encoder_zero_calibration.h"
#include "voltage_sample.h"
#include "sample_time.h"
#include "pid.h"
#include "math.h"
#include "motor_POS.h"
#include "motor_speed.h"

/**************************电流环参数**************************/		
		 
/**************************速度环参数**************************/			 



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

        /* A conversion already pending at stop must not republish old state. */
        if (foc_pwm_output_is_enabled() == 0U)
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

        //motor_speed_update(&pm.speed, &pm.encoder_foc_sample,
                           //pm.motor.pn, DWT->CYCCNT);

				
        foc_current_calc(&pm);
				pm.foc.id_fdb_filtered  += ID_FILTER_ALPHA * (pm.foc.id_fdb - pm.foc.id_fdb_filtered);
//				pm.foc.iq_fdb_filtered  += ID_FILTER_ALPHA * (pm.foc.iq_fdb - pm.foc.iq_fdb_filtered);
//				pm.foc.Is_filtered = sqrtf(pm.foc.id_fdb_filtered*pm.foc.id_fdb_filtered + pm.foc.iq_fdb_filtered*pm.foc.iq_fdb_filtered);
#if ((FOC_ENCODER_ZERO_CALIBRATION_MODE == 0U) && \
     (FOC_PHASE_AUTO_CALIBRATION_MODE == 0U))
				pm.foc.id_ref = 0.0f;
        pm.foc.iq_ref = foc_speed_pid.output;
        foc_current_pid_update(&pm, PWM_PERIOD_S, 0U);
#endif
//pm.foc.Ud_ref_filtered += ID_FILTER_ALPHA * (pm.foc.Ud_ref - pm.foc.Ud_ref_filtered);
//				 pm.foc.Ud_ref = 2.0f;
//				 pm.foc.Uq_ref = 0.0f;//13.163f
        foc_voltage_calc(&pm, voltage_sample.vbus_v, pm.svpwm.vref2_min);//voltage_sample.vbus_v pm.svpwm.vref2_min
        foc_pwm_output_update(&pm);
        foc_callback_time_measure_end(&pm);
    }
}

/* TIM7: speed updates at 2 kHz; TIM3: position updates at 1 kHz. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        motor_speed_update(&pm.speed, &pm.encoder_foc_sample, pm.motor.pn);
#if ((FOC_ENCODER_ZERO_CALIBRATION_MODE == 0U) && \
     (FOC_PHASE_AUTO_CALIBRATION_MODE == 0U))
        /* Match the position-angle direction to the speed convention. */
        pm.speed.mechanical_rad_s_ref = 50.0f;//foc_position_pid.output *
//            MOTOR_SPEED_DIRECTION * (float)calibration.electrical_direction;
        foc_speed_pid_update(&pm, MOTOR_SPEED_UPDATE_PERIOD_S);
#endif
    }
    else if (htim->Instance == TIM3)
    {
        motor_pos_update();
#if ((FOC_ENCODER_ZERO_CALIBRATION_MODE == 0U) && \
     (FOC_PHASE_AUTO_CALIBRATION_MODE == 0U))
        foc_position_pid_update(&pm, MOTOR_POS_UPDATE_PERIOD_S);
#endif
    }
}

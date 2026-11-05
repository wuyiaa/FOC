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

        /* While paused, TIM1 CC4 owns angle sampling; no current-loop output. */
        if (foc_pwm_output_is_enabled() == 0U)
        {
            voltage_sample_bus_update();
            foc_callback_time_measure_end(&pm);
            return;
        }
      				       

				voltage_sample_bus_update();
        foc_adc_sample(&pm);
				foc_current_sample_process(&pm);
        foc_current_calc(&pm);
        
				(void)as5047p_read_blocking(&pm.encoder, &pm.encoder_foc_sample);
				foc_electrical_angle_update(&pm);


        /* Complete one SPI frame and update this FOC cycle's angle sample. */

  
        //motor_speed_update(&pm.speed, &pm.encoder_foc_sample,
                           //pm.motor.pn, DWT->CYCCNT);

				
				pm.foc.id_fdb_filtered  += ID_FILTER_ALPHA * (pm.foc.id_fdb - pm.foc.id_fdb_filtered);
				pm.foc.iq_fdb_filtered  += ID_FILTER_ALPHA * (pm.foc.iq_fdb - pm.foc.iq_fdb_filtered);
				pm.foc.Is_filtered = sqrtf(pm.foc.id_fdb_filtered*pm.foc.id_fdb_filtered + pm.foc.iq_fdb_filtered*pm.foc.iq_fdb_filtered);
#if ((FOC_ENCODER_ZERO_CALIBRATION_MODE == 0U) && \
     (FOC_PHASE_AUTO_CALIBRATION_MODE == 0U))
				pm.foc.id_ref = 0.0f;
        pm.foc.iq_ref = foc_speed_pid.output;//foc_speed_pid.output;  0.042
        foc_current_pid_update(&pm, PWM_PERIOD_S, 0U);
#endif
pm.foc.Ud_ref_filtered += ID_FILTER_ALPHA * (pm.foc.Ud_ref - pm.foc.Ud_ref_filtered);
//				 pm.foc.Ud_ref = 0.0f;//2.0f
//				 pm.foc.Uq_ref = 13.0f;//13.163f
        foc_voltage_calc(&pm, voltage_sample.vbus_v, pm.svpwm.vref2_min);//voltage_sample.vbus_v pm.svpwm.vref2_min
        foc_pwm_output_update(&pm);
        foc_callback_time_measure_end(&pm);
    }
}

/* CC4 compare events continue with MOE off, even if ADC triggering stops.
   During run, CC4 IRQ is disabled and the ADC callback owns SPI instead. */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim->Instance == TIM1) &&
        (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) &&
        (pm.adc.offset_ready != 0U) &&
        (foc_pwm_output_is_enabled() == 0U))
    {
        (void)as5047p_read_blocking(&pm.encoder, &pm.encoder_foc_sample);
        foc_electrical_angle_update(&pm);
    }
}

/* TIM7: speed updates at 2 kHz; TIM3: position updates at 1 kHz. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {   
			
        motor_speed_update(&pm.speed, &pm.encoder_foc_sample, pm.motor.pn);
			  speed_lungberger_two(&pm.speed, pm.foc.mechanical_angle_rad, pm.motor.pn);
        speed_Luenberger_three(&pm.speed.observer_three,
                               pm.foc.mechanical_angle_rad,
                               MOTOR_SPEED_UPDATE_PERIOD_S,
                               MOTOR_SPEED_LUENBERGER_THREE_POLE_HZ);
			
#if ((FOC_ENCODER_ZERO_CALIBRATION_MODE == 0U) && \
     (FOC_PHASE_AUTO_CALIBRATION_MODE == 0U))
        /* Match the position-angle direction to the speed convention. */
//        pm.speed.mechanical_rad_s_ref = 50.0f;// foc_position_pid.output * MOTOR_SPEED_DIRECTION * (float)calibration.electrical_direction;
           pm.speed.mechanical_rad_s_fdb = pm.speed.mechanical_rad_s_filtered;//pm.speed.mechanical_rad_s_filtered  pm.speed.mechanical_rad_s_hat
			
			
			
#if (FOC_MOTOR_IDENTIFICATION_ENABLE != 0U)
        J_B_Tf_Indent(&pm.speed, pm.foc.iq_fdb,
                      1.5f * (float)pm.motor.pn * pm.motor.phif,
                      foc_pwm_output_is_enabled());
        if (((motor_identification.stage == MOTOR_IDENT_DONE) ||
             (motor_identification.stage == MOTOR_IDENT_FAILED)) &&
            (foc_pwm_output_is_enabled() != 0U))
        {
            foc_pwm_output_disable();
        }
#endif
        foc_speed_pid_update(&pm, MOTOR_SPEED_UPDATE_PERIOD_S);
#endif
    }
    else if (htim->Instance == TIM3)
    {
//        motor_pos_update();
			pm.pos.mechanical_angle_rad_fdb = pm.foc.mechanical_angle_rad;//pm.foc.mechanical_angle_rad; pm.speed.mechanical_rad_s_hat
//			pm.pos.mechanical_angle_rad_ref = 0.0f;
#if ((FOC_ENCODER_ZERO_CALIBRATION_MODE == 0U) && \
     (FOC_PHASE_AUTO_CALIBRATION_MODE == 0U))
        foc_position_pid_update(&pm, MOTOR_POS_UPDATE_PERIOD_S);
#endif
    }
}

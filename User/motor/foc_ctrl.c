#include "foc_ctrl.h"
#include "encoder_zero_calibration.h"
#include "B_J_Tf_Identification.h"
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
				
				pm.foc.id_fdb_filtered  += ID_FILTER_ALPHA * (pm.foc.id_fdb - pm.foc.id_fdb_filtered);
				pm.foc.iq_fdb_filtered  += ID_FILTER_ALPHA * (pm.foc.iq_fdb - pm.foc.iq_fdb_filtered);
				pm.foc.Is_filtered = sqrtf(pm.foc.id_fdb_filtered*pm.foc.id_fdb_filtered + pm.foc.iq_fdb_filtered*pm.foc.iq_fdb_filtered);
#if ((FOC_ENCODER_ZERO_CALIBRATION_MODE == 0U) && \
     (FOC_PHASE_AUTO_CALIBRATION_MODE == 0U))
				pm.foc.id_ref = 0.0f;
        pm.foc.iq_ref = foc_speed_pid.output 
											+	(pm.speed.observer_torque_3th.load_torque_nm_hat/pm.motor.Kt);//foc_speed_pid.output;  0.042
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
				float wc_l = 500.0f;
			
        motor_speed_update(&pm.speed, &pm.encoder_foc_sample, pm.motor.pn);
        speed_LPF_filtered(&pm.speed, pm.motor.pn);
			  
			  speed_lungberger_2nd(&pm.speed.observer_2nd, 
						pm.foc.mechanical_angle_rad * (MOTOR_SPEED_DIRECTION * (float)calibration.electrical_direction), pm.motor.pn);

  			speed_Luenberger_3th(&pm.speed.observer_3th,
                               pm.foc.mechanical_angle_rad,
                               MOTOR_SPEED_UPDATE_PERIOD_S,
                               MOTOR_SPEED_LUENBERGER_3TH_POLE_HZ);
			
        /* Parallel mechanical model; paused current feedback would be stale. */
        speed_torque_Luenberger_3th(
            &pm.speed.observer_torque_3th,
            pm.foc.mechanical_angle_rad, pm.speed.mechanical_rad_s,
            (foc_pwm_output_is_enabled() != 0U) ? pm.foc.iq_fdb : 0.0f,
            MOTOR_SPEED_UPDATE_PERIOD_S,
            wc_l,
            pm.motor.J, pm.motor.B, pm.motor.Kt, pm.motor.pn);
			
			//MOTOR_SPEED_TORQUE_LUENBERGER_3TH_POLE_RAD_S
#if ((FOC_ENCODER_ZERO_CALIBRATION_MODE == 0U) && \
     (FOC_PHASE_AUTO_CALIBRATION_MODE == 0U))
        /* Match the position-angle direction to the speed convention. */
				
				//如果反馈使用低通和二阶就要注意方向了
        pm.speed.mechanical_rad_s_ref = foc_position_pid.output;	
				 foc_position_pid.output * MOTOR_SPEED_DIRECTION * (float)calibration.electrical_direction;
				
			  pm.speed.mechanical_rad_s_fdb = pm.speed.observer_torque_3th.mechanical_rad_s_hat; 
//				pm.speed.mechanical_rad_s_filtered  pm.speed.observer_2nd.mechanical_rad_s_hat
//				pm.speed.observer_torque_3th.mechanical_rad_s_hat
			
			
#if (FOC_MOTOR_IDENTIFICATION_ENABLE != 0U)
        if (motor_identification_requested != 0U)
        {
            J_B_Tf_Indent(&pm.speed, pm.foc.iq_fdb,
                          pm.motor.Kt,
                          foc_pwm_output_is_enabled());
            if ((motor_identification.stage == MOTOR_IDENT_DONE) ||
                (motor_identification.stage == MOTOR_IDENT_FAILED))
            {
                motor_identification_requested = 0U;
                foc_pwm_output_disable();
                if (motor_identification.stage == MOTOR_IDENT_DONE)
                {
                    /* Apply, program and verify once, after MOS outputs stop. */
                    motor_identification_save_process();
                }
            }
        }
#endif

        foc_speed_pid_update(&pm, MOTOR_SPEED_UPDATE_PERIOD_S);
				
#endif
    }
    else if (htim->Instance == TIM3)
    {
      motor_pos_update();
			pm.pos.mechanical_angle_deg_ref = 30.2f;
			
#if ((FOC_ENCODER_ZERO_CALIBRATION_MODE == 0U) && \
     (FOC_PHASE_AUTO_CALIBRATION_MODE == 0U))
        foc_position_pid_update(&pm, MOTOR_POS_UPDATE_PERIOD_S);
#endif
    }
}

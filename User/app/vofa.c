#include "vofa.h"
#include "dwt_time.h"

#include "common.h"
#include "eccentricity_identification.h"
#include "encoder_zero_calibration.h"
#include "B_J_Tf_Identification.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "pid.h"
#include "pwm_capture.h"
#include "voltage_sample.h"

#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

#define VOFA_SEND_PERIOD_US       200U
#define VOFA_SEND_PERIOD_CYCLES \
    ((DWT_CPU_FREQ_HZ / 1000000UL) * VOFA_SEND_PERIOD_US)
#define VOFA_HOST_SETTLE_TIME_MS  10U
#define VOFA_TX_TIMEOUT_MS         500U
#define VOFA_TX_TIMEOUT_CYCLES \
    ((DWT_CPU_FREQ_HZ / 1000UL) * VOFA_TX_TIMEOUT_MS)
#define VOFA_JUSTFLOAT_TAIL_SIZE  4U
#define VOFA_MAX_CHANNEL_COUNT    24U

/*
 * Edit vofa_values[] in vofa_packet_build() to change the channel list/order.
 * Use (float)variable for integer or enum values.
 * Configure the same channel names/order in VOFA+ using the JustFloat parser.
 */
#define VOFA_TELEMETRY_BUFFER_SIZE \
    (VOFA_MAX_CHANNEL_COUNT * sizeof(float) + VOFA_JUSTFLOAT_TAIL_SIZE)
#define VOFA_PWM_FRAMES_PER_PACKET  16U
#define VOFA_PWM_FRAME_SIZE \
    (VOFA_MAX_CHANNEL_COUNT * sizeof(float) + VOFA_JUSTFLOAT_TAIL_SIZE)
#define VOFA_PWM_BUFFER_SIZE \
    (VOFA_PWM_FRAMES_PER_PACKET * VOFA_PWM_FRAME_SIZE)
#define VOFA_BUFFER_SIZE \
    ((VOFA_PWM_BUFFER_SIZE > VOFA_TELEMETRY_BUFFER_SIZE) ? \
      VOFA_PWM_BUFFER_SIZE : VOFA_TELEMETRY_BUFFER_SIZE)

volatile uint8_t vofa_mode = VOFA_DEFAULT_MODE;
volatile uint8_t vofa_pwm_auto_repeat = 1U;
volatile uint8_t vofa_pwm_capture_request;
volatile uint32_t vofa_pwm_sent_samples;

/* CDC retains the Tx buffer pointer until the USB IN transfer completes. */
static uint8_t vofa_tx_buffer[VOFA_BUFFER_SIZE];
static uint32_t vofa_last_send_cycle;
static uint32_t vofa_tx_start_cycle;
static volatile uint8_t vofa_host_ready;
static volatile uint8_t vofa_host_request_pending;
static volatile uint32_t vofa_host_request_tick;
static uint8_t vofa_active_mode = VOFA_DEFAULT_MODE;
static uint8_t vofa_session_active;
static uint8_t vofa_pwm_auto_pending = 1U;
static uint16_t vofa_pwm_packet_samples;

/* Successful USB submissions, not host receipt or ADC sample timestamps. */
volatile float vofa_send_interval_us;
volatile float vofa_send_frequency_hz;
static uint32_t vofa_send_time_last;
static uint32_t vofa_send_tick_last;
static uint8_t vofa_send_time_valid;
//extern float sinf_e;
//extern float cosf_e;
extern float theta_e;

static void vofa_send_time_update(void)
{
    const uint32_t now_tick = HAL_GetTick();
    float dt;
    /* Reject long idle gaps before the 32-bit DWT counter can wrap fully. */
    if ((vofa_send_time_valid == 0U) ||
        ((uint32_t)(now_tick - vofa_send_tick_last) >= 20000U))
    {
        vofa_send_time_last = DWT->CYCCNT;
        vofa_send_interval_us = 0.0f;
        vofa_send_frequency_hz = 0.0f;
        vofa_send_time_valid = 1U;
    }
    else
    {
        dt = DWT_GetDeltaT(&vofa_send_time_last);
        vofa_send_interval_us = dt * 1000000.0f;
        vofa_send_frequency_hz = (dt > 0.0f) ? 1.0f / dt : 0.0f;
    }
    vofa_send_tick_last = now_tick;
}

static void vofa_pwm_reset(void)
{
    vofa_send_time_valid = 0U;
    vofa_send_interval_us = 0.0f;
    vofa_send_frequency_hz = 0.0f;
    pwm_capture_cancel();
    vofa_pwm_sent_samples = 0U;
    vofa_pwm_packet_samples = 0U;
    vofa_pwm_capture_request = 0U;
    vofa_pwm_auto_pending = 1U;
}

void vofa_init(void)
{
    vofa_last_send_cycle = DWT->CYCCNT;
    vofa_pwm_reset();
}

void vofa_usb_host_request(void)
{
    vofa_host_request_tick = HAL_GetTick();
    vofa_host_ready = 1U;
    vofa_host_request_pending = 1U;
}

void vofa_usb_disconnected(void)
{
    vofa_host_ready = 0U;
    vofa_host_request_pending = 0U;
}

static uint16_t vofa_packet_build(void)
{
    const uint32_t saved_primask = __get_PRIMASK();

    /* Copy currents and diagnostics together, without an ADC ISR splitting
       the frame across two control cycles. Restore IRQs before packing USB. */
    __disable_irq();
    /* Nearby reads, not a hardware-synchronized CNT/GPIO capture.
       LIN1/2/3 are physical PB13/14/15 driver-input levels. */
    const uint32_t low_side_levels = GPIOB->IDR;
    const float vofa_values[] =
    {
/*********************************************控制器*********************************************************/

//				/*电机电流环*/
//				pm.foc.iq_ref,
//				pm.foc.iq_fdb,	
//				pm.foc.id_ref,	
//				pm.foc.id_fdb,				
//				pm.foc.Uq_ref,
//				pm.foc.Ud_ref,
//				pm.speed.observer_torque_3th.load_torque_nm_hat/pm.motor.Kt,

//				
//			/*电机速度环*/
//			pm.speed.mechanical_rad_s_fdb,
//			pm.speed.mechanical_rad_s_ref,
//			foc_speed_pid.output,
//			foc_speed_pid.error,
//			foc_speed_pid.p_output,
//			foc_speed_pid.integral_output,
//			pm.speed.observer_torque_3th.load_torque_nm_hat/pm.motor.Kt,
//												
								
//			/*电机位置环*/
//			pm.pos.mechanical_angle_deg_ref,
//			pm.pos.mechanical_angle_deg_fdb,
//			pm.pos.mechanical_angle_rad_ref,
//			pm.pos.mechanical_angle_rad_fdb,
//			theta_e,
//			pm.encoder.mechanical_angle_rad_latest,
//			pm.foc.mechanical_angle_rad,
//			foc_position_pid.error*57.32f,
//			foc_position_pid.p_output,
//			foc_position_pid.integral_output,
//			foc_position_pid.output,
			
/*********************************************控制量*********************************************************/
					/*电机角度*/		
//				pm.foc.mechanical_angle_rad,
//				pm.foc.electrical_angle_rad,
//				pm.foc.electrical_angle_rad_camp,
//				pm.foc.sin_theta,
//				pm.foc.sin_theta_camp,
//				pm.foc.cos_theta,
//				pm.foc.cos_theta_camp,

			/*电机角速度*/
//			pm.speed.mechanical_rad_s,   /* CH4: mechanical speed (rad/s) */
			pm.speed.mechanical_rad_s_filtered,
//			pm.speed.observer_2nd.mechanical_rad_s_hat,
//			pm.speed.observer_3th.mechanical_rad_s_hat,
//			pm.speed.observer_torque_3th.mechanical_rad_s_hat,
//			pm.speed.observer_torque_3th.load_torque_nm_hat,
			pm.speed.mechanical_rad_s_ref,

//				pm.speed.observer_3th.mechanical_angle_rad_hat,
						
			/*反馈相电流*/
//				pm.foc.ia.nonrecon,   
//				pm.foc.ib.nonrecon,   
//				pm.foc.ic.nonrecon,   
//				pm.foc.ia.reconstructed,                  /* CH0: phase A current (A) */
//				pm.foc.ib.reconstructed,                  /* CH1: phase B current (A) */
//				pm.foc.ic.reconstructed,                  /* CH2: phase C current (A) */
			
//			pm.foc.i_alpha,              /* CH5: alpha-axis current (A) */
//			pm.foc.i_beta,               /* CH6: beta-axis current (A) */

//				pm.foc.iq_fdb,
//				pm.foc.d_fdb,	

/*********************************************辨识*********************************************************/

			/*反馈相电阻*/
//				pm.foc.Ud_ref/pm.foc.Is_filtered,
//				pm.foc.Ud_ref/pm.foc.id_fdb_filtered,
//				pm.foc.Ud_ref_filtered/pm.foc.id_ref,			
//				pm.foc.iq_fdb,
//				pm.foc.iq_fdb_filtered,			
//				pm.foc.id_fdb,	
//        pm.foc.id_fdb_filtered,										
//				pm.foc.Is_filtered,

				/*偏心补偿辨识*/
//				foc_open_loop_angle_measure.mechanical_angle_deg_ref,    // 强拖机械角度指令
//				foc_open_loop_angle_measure.mechanical_angle_deg_fdb,    // 编码器连续机械角度
//                foc_open_loop_angle_measure.mechanical_angle_deg_fdb_ecc, //补偿后连续机械角度
//                foc_open_loop_angle_measure.correction_deg, //查表补偿量
//                foc_open_loop_angle_measure.mechanical_angle_deg_error,   	//指令 − 补偿前角度
//                foc_open_loop_angle_measure.mechanical_angle_deg_error_ecc, //指令 − 补偿后角度
//				foc_open_loop_angle_measure.mechanical_angle_deg_error, //实时角度误差=CH0-CH1，未处理
//				eccentricity_measure.offset_b_deg,                /* CH3: 整圈配对误差的平均偏移b */
//				eccentricity_measure.paired_error_deg_display,    /* CH4: 抵消摩擦、减去b，再经过FIR的误差 */
//				eccentricity_measure.paired_error_deg_before_fir_display, /* CH5: 抵消摩擦、减去b后，FIR前的误差 */
//				eccentricity_measure.paired_angle_deg_display,    /* CH6: CH4、CH5对应的机械角度 */
//				(float)eccentricity_measure.done,                 /* CH7: 完成标志:0=采集或计算中，1=完成并回放 */
//				pm.speed.mechanical_rad_s_fdb,										/* CH8*/

				eccentricity_compensation.mechanical_angle_deg_before, //偏心补偿前	/* CH9*/
				eccentricity_compensation.mechanical_angle_deg_after,	 //偏心补偿后	/* CH10*/
//				eccentricity_compensation.correction_deg, 			//补偿表						/* CH11*/
//				(float)eccentricity_compensation.ready,					//表准备完成标志		/* CH12*/
//					(float)eccentricity_compensation.flash_saved, 	//Flash 保存成功标志 不太准 建议看下一个/* CH13*/
//				((eccentricity_compensation.flash_saved == 1U) && (eccentricity_compensation.flash_loaded == 0U))	,
//				eccentricity_compensation.save_pending,
//				eccentricity_compensation.flash_status,



//			/*J_B_Tf*/
//			motor_identification.J,
//			motor_identification.B,
//			motor_identification.Tf,
//			motor_identification.stage,
//			pm.speed.observer_2nd.mechanical_rad_s_hat,
//			pm.speed.mechanical_rad_s_ref,
//			pm.speed.mechanical_rad_s_fdb,
//			motor_identification.acceleration.acceleration_mean_rad_s2,
//			
//			pm.foc.iq_fdb,
//			pm.foc.id_fdb,	
//			
//			motor_identification.acceleration.iq_mean_a,
//			motor_identification.acceleration.acceleration_mean_rad_s2,
//			motor_identification.acceleration.speed_mean_rad_s,

//			motor_identification.deceleration.iq_mean_a,
//			motor_identification.deceleration.acceleration_mean_rad_s2,
//			motor_identification.deceleration.speed_mean_rad_s,
//			
//			pm.motor.J, pm.motor.B, pm.motor.Tf,
//			motor_identification.result_ready, //辨识计算完成
//			motor_identification.flash_saved,  //结果已应用并成功保存到 Flash
			
			/*
|  |  |
|motor_identification.stage
| 0  | 等待开始 |
| 1  | 等待速度在 25±2 rad/s 内持续稳定 1 秒 |
| 2  | 在 25 rad/s 匀速段，采集平均速度和反馈 iq，持续 1 秒 |
| 3  | 等待速度在 50±2 rad/s 内持续稳定 1 秒 |
| 4  | 在 50 rad/s 匀速段采集 1 秒，随后计算 B、Tf |
| 5  | 返回 25 rad/s，等待稳定 |
| 6  | 加速至 50 rad/s，采集反馈速度 30→45 rad/s 区间的数据 |
| 7  | 在 50 rad/s 等待稳定 |
| 8  | 减速至 25 rad/s，采集反馈速度 45→30 rad/s 区间的数据，随后计算 J |
| 9  | 辨识完成，随后关闭 MOS 并保存结果 |
| 10 | 辨识失败，关闭 MOS |
*/

//				/*电机角度零点偏移采样*/
//				(float)pm.encoder_foc_sample.raw,
////				(float)calibration.sampling,
//				(float)calibration.done,
//					calibration.failed,
////				(float)calibration.sample_count,
//				(float)calibration.mechanical_zero_raw,
//					calibration.flash_saved,
//				(float)calibration.mechanical_zero_angle_rad

				/*三相校准*/
//				calibration.done,
//				calibration.phase_reversed,
//				calibration.electrical_direction,
//				calibration.current_direction,
//				calibration.flash_saved,
/*********************************************采集数据*********************************************************/
//			
//			/*目标相电流*/
//				pm.foc.iq_ref,	
//				pm.foc.id_ref,
			
//				pm.foc.i_a_ref,                  /* CH0: phase A current (A) */
//        pm.foc.i_b_ref,                  /* CH1: phase B current (A) */
//        pm.foc.i_c_ref,  
			
			/*原始电流值*/
//				(float)ADC1->JDR1, //ia
//				(float)ADC1->JDR2, //ib
//				(float)ADC2->JDR1, //ic
//				(float)ADC2->JDR2, //vbus

			/*电流采样时间*/
//			pm.adc_measure.sample_time_us,
//			pm.adc_measure.elapsed_ticks,
//			pm.adc_measure.timer_count,
//			pm.adc_measure.timer_counting_down,
			
			/*相电流偏移*/
//				 pm.foc.ia.offset,
//				 pm.foc.ib.offset,
//				 pm.foc.ic.offset,

				/*电流采样开关管*/
//        (float)TIM1->CNT,          
//        (float)((low_side_levels & LIN1_Pin) != 0U), 
//        (float)((low_side_levels & LIN2_Pin) != 0U), 
//        (float)((low_side_levels & LIN3_Pin) != 0U), 
					
//			/*反馈相电压*/
//			voltage_sample.U_ng,  /* CH13: estimated motor neutral (V) */
//			voltage_sample.Uan,
//			voltage_sample.Ubn,
//			voltage_sample.Ucn,
//			voltage_sample.vbus_v,
////			/*反馈端电压*/
//			voltage_sample.Uag.value,
//			voltage_sample.Ubg.value,
//			voltage_sample.Ucg.value,
//		
//			voltage_sample.Uag.average,   
//      voltage_sample.Ubg.average,   
//      voltage_sample.Ucg.average,
//			
//			voltage_sample.Ualpha_fdb,
//			voltage_sample.Ubeta_fdb,

//			voltage_sample.Uq_fdb,
//			voltage_sample.Ud_fdb,
			
			/*目标相电压*/
//			pm.foc.u_alpha,                 
//      pm.foc.u_beta,                  

//				pm.foc.Uq_ref,
//				pm.foc.Ud_ref,
//						
////			/*马鞍波*/
//        (float)pm.svpwm.ccr1,        /* CH9: PWM compare 1 */
//        (float)pm.svpwm.ccr2,        /* CH10: PWM compare 2 */
//        (float)pm.svpwm.ccr3,        /* CH11: PWM compare 3 */


			
//        (float)pm.adc_measure.adc1_jeos_at_callback, /* CH5: ADC1 JEOS in callback */
//        (float)pm.adc_measure.adc2_jeos_at_callback, /* CH6: ADC2 JEOS in callback */
//        (float)pm.adc_measure.adc2_jeos_not_set_count, /* CH7: ADC2 JEOS=0 count */

				/*电流环*/
//				foc_iq_pid.p_output,
//				foc_iq_pid.integral_output,
//				foc_id_pid.p_output,
//				foc_id_pid.integral_output

				/*电流采样及其重构*/
//        (float)pm.current_sample.window_a,       /* CH6: A low-side ticks */
//        (float)pm.current_sample.window_b,       /* CH7: B low-side ticks */
//        (float)pm.current_sample.window_c,       /* CH8: C low-side ticks */
//        (float)pm.current_sample.window_a_valid, /* CH9: A duration valid */
//        (float)pm.current_sample.window_b_valid, /* CH10: B duration valid */
//        (float)pm.current_sample.window_c_valid, /* CH11: C duration valid */
//        (float)pm.current_sample.reconstructed_phase, /* CH12: 0=none,1=A,2=B,3=C */
        /* CH13: all measured sources retained AFTER reconstruction have
           enough ON time. Exclude only the phase actually reconstructed.
           This checks duration, not ADC trigger placement or accuracy. */
//			(float)(((pm.current_sample.window_a_valid != 0U) ||
//							 (pm.current_sample.reconstructed_phase == PMSM_PHASE_A)) &&
//							((pm.current_sample.window_b_valid != 0U) ||
//							 (pm.current_sample.reconstructed_phase == PMSM_PHASE_B)) &&
//							((pm.current_sample.window_c_valid != 0U) ||
//							 (pm.current_sample.reconstructed_phase == PMSM_PHASE_C))),

//        pm.current_sample.i_sum_before_reconstruction /* CH17: A+B+C before reconstruction (A) */
//				pm.foc.ia.reconstructed+pm.foc.ib.reconstructed+pm.foc.ic.reconstructed,
/*********************************************编码器*********************************************************/

			/*编码器计数报错*/
//				pm.encoder.parity_error_count,
//				pm.encoder.sensor_error_count,
//				pm.encoder.busy_skip_count ,
//				pm.encoder.encoder_sample_sequence,
//				pm.foc.mechanical_angle_rad,
//		  	pm.                                                                                                                    foc.electrical_angle_rad,
//			 (float)pm.encoder_foc_sample.raw,
//			calibration.current_direction,
//			 sinf_e,
//			 cosf_e,
//			 pm.foc.sin_theta, 
//			 pm.foc.cos_theta,
//			 sinf_e-pm.foc.sin_theta,
//			 cosf_e-pm.foc.cos_theta,

    };
    uint16_t channel_count;
    uint16_t i;
    uint16_t offset;
    uint16_t packet_size;

    __set_PRIMASK(saved_primask);

    channel_count = (uint16_t)(sizeof(vofa_values) / sizeof(vofa_values[0]));
    if (channel_count > VOFA_MAX_CHANNEL_COUNT)
    {
        return 0U;
    }

    if (sizeof(float) != 4U)
    {
        return 0U;
    }

    offset = 0U;
    for (i = 0U; i < channel_count; i++)
    {
        memcpy(&vofa_tx_buffer[offset], &vofa_values[i], sizeof(float));
        offset = (uint16_t)(offset + sizeof(float));
    }

    /* JustFloat frame delimiter: 0x0000807F, sent little-endian. */
    vofa_tx_buffer[offset++] = 0x00U;
    vofa_tx_buffer[offset++] = 0x00U;
    vofa_tx_buffer[offset++] = 0x80U;
    vofa_tx_buffer[offset] = 0x7FU;
    packet_size = (uint16_t)(offset + 1U);

    return packet_size;
}

static uint16_t vofa_pwm_packet_build(uint16_t *sample_count)
{
    uint16_t offset = 0U;
    uint16_t count = 0U;
    uint32_t index;

    *sample_count = 0U;
    if (sizeof(float) != 4U)
    {
        return 0U;
    }

    /* Board default has PWM disabled. Automatically wait for the key to
       enable it; an explicit Keil request can also capture the OFF state. */
    if ((vofa_pwm_auto_pending != 0U) &&
        (foc_pwm_output_is_enabled() != 0U))
    {
        vofa_pwm_capture_request = 1U;
    }

    /* Reuse the capture buffer only after the last USB packet completes.
       READY excludes errors: automatic replay never retries a failed DMA. */
    if ((vofa_pwm_auto_repeat != 0U) &&
        (pwm_capture.state == PWM_CAPTURE_READY) &&
        (vofa_pwm_sent_samples >= PWM_CAPTURE_SAMPLE_COUNT))
    {
        vofa_pwm_capture_request = 1U;
    }
    if (vofa_pwm_capture_request != 0U)
    {
        vofa_pwm_auto_pending = 0U;
        /* Queue a request made during acquisition/replay until it finishes. */
        if ((pwm_capture.state != PWM_CAPTURE_RUNNING) &&
            ((pwm_capture.state != PWM_CAPTURE_READY) ||
             (vofa_pwm_sent_samples >= PWM_CAPTURE_SAMPLE_COUNT)))
        {
            vofa_pwm_capture_request = 0U;
            vofa_pwm_sent_samples = 0U;
            (void)pwm_capture_start();
            return 0U;
        }
    }

    if (pwm_capture.state != PWM_CAPTURE_READY)
    {
        return 0U;
    }

    index = vofa_pwm_sent_samples;
    while ((index < PWM_CAPTURE_SAMPLE_COUNT) &&
           (count < VOFA_PWM_FRAMES_PER_PACKET))
    {
        const uint16_t pins = pwm_capture_sample_get(index);
        const float values[] =
        {
            (float)((pins >> 13U) & 1U), /* CH0: physical LIN1 / PB13 */
            (float)((pins >> 14U) & 1U), /* CH1: physical LIN2 / PB14 */
            (float)((pins >> 15U) & 1U), /* CH2: physical LIN3 / PB15 */
            /* Live at replay time, NOT sampled together with the GPIOs. */
            pm.foc.ia.reconstructed,                 /* CH3: live phase A current (A) */
            pm.foc.ib.reconstructed,                 /* CH4: live phase B current (A) */
            pm.foc.ic.reconstructed,                 /* CH5: live phase C current (A) */
            (float)pwm_capture.completed_count, /* CH6: capture ID */
            (float)index,               /* CH7: point in record, 0..3999 */
        };

        /* Keep edits to this channel list within the allocated USB buffer. */
        if ((sizeof(values) > VOFA_MAX_CHANNEL_COUNT * sizeof(float)) ||
            ((sizeof(values) + VOFA_JUSTFLOAT_TAIL_SIZE) >
             (sizeof(vofa_tx_buffer) - offset)))
        {
            return 0U;
        }
        memcpy(&vofa_tx_buffer[offset], values, sizeof(values));
        offset = (uint16_t)(offset + sizeof(values));
        vofa_tx_buffer[offset++] = 0x00U;
        vofa_tx_buffer[offset++] = 0x00U;
        vofa_tx_buffer[offset++] = 0x80U;
        vofa_tx_buffer[offset++] = 0x7FU;
        index++;
        count++;
    }
    *sample_count = count;
    return offset;
}

void vofa_process(void)
{
    USBD_CDC_HandleTypeDef *hcdc;
    uint32_t now_cycle;
    uint16_t packet_size;
    uint16_t capture_samples = 0U;

    /* Poll DMA once per foreground pass, without waiting for completion. */
    pwm_capture_process();
    now_cycle = DWT->CYCCNT;
    if ((uint32_t)(now_cycle - vofa_last_send_cycle) <
        VOFA_SEND_PERIOD_CYCLES)
    {
        return;
    }
    /* Advance the interval even if USB is busy and this attempt is skipped. */
    vofa_last_send_cycle = now_cycle;

    /* USB deinit can free pClassData. Protect both the check and all class
       accesses below against USB IRQs; FOC/ADC interrupts remain enabled. */
    HAL_NVIC_DisableIRQ(USB_LP_IRQn);
    if ((hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) ||
        (hUsbDeviceFS.pClassData == NULL) ||
        (vofa_host_ready == 0U) ||
        ((uint32_t)(HAL_GetTick() - vofa_host_request_tick) <
         VOFA_HOST_SETTLE_TIME_MS))
    {
        HAL_NVIC_EnableIRQ(USB_LP_IRQn);
        if (vofa_session_active != 0U)
        {
            vofa_pwm_reset();
            vofa_session_active = 0U;
        }
        return;
    }

    hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (vofa_host_request_pending != 0U)
    {
        /* Discard a transfer left pending by an earlier port session. */
        if (hcdc->TxState != 0U)
        {
            if ((hUsbDeviceFS.pData == NULL) ||
                (HAL_PCD_EP_Abort((PCD_HandleTypeDef *)hUsbDeviceFS.pData,
                                  CDC_IN_EP) != HAL_OK))
            {
                HAL_NVIC_EnableIRQ(USB_LP_IRQn);
                return;
            }
            hcdc->TxState = 0U;
        }
        vofa_host_request_pending = 0U;
        /* A reopened host receives a new record, never a partial old one. */
        vofa_pwm_reset();
        vofa_session_active = 1U;
    }

    if (hcdc->TxState != 0U)
    {
        /* A closed host port can leave its last IN transfer pending forever. */
        if ((uint32_t)(now_cycle - vofa_tx_start_cycle) >=
            VOFA_TX_TIMEOUT_CYCLES)
        {
            if ((hcdc->TxState != 0U) && (hUsbDeviceFS.pData != NULL) &&
                (HAL_PCD_EP_Abort((PCD_HandleTypeDef *)hUsbDeviceFS.pData,
                                  CDC_IN_EP) == HAL_OK))
            {
                hcdc->TxState = 0U;
                vofa_host_ready = 0U;
            }
        }
        HAL_NVIC_EnableIRQ(USB_LP_IRQn);
        return;
    }
    HAL_NVIC_EnableIRQ(USB_LP_IRQn);

    /* Reaching here with TxState == 0 acknowledges the previous packet.
       USB busy/failure never advances the frozen-record replay cursor. */
    if (vofa_pwm_packet_samples != 0U)
    {
        vofa_pwm_sent_samples += vofa_pwm_packet_samples;
        vofa_pwm_packet_samples = 0U;
    }

    if (vofa_active_mode != vofa_mode)
    {
        vofa_pwm_reset();
        vofa_active_mode = vofa_mode;
    }

    if (vofa_active_mode == VOFA_MODE_PWM_CAPTURE)
    {
        packet_size = vofa_pwm_packet_build(&capture_samples);
    }
    else if (vofa_active_mode == VOFA_MODE_TELEMETRY)
    {
        packet_size = vofa_packet_build();
    }
    else
    {
        return;
    }
    if (packet_size == 0U)
    {
        return;
    }

    /* USB may disconnect while a packet is built. Guard the final class
       pointer check/submission against USB deinit; ADC interrupts stay on. */
    HAL_NVIC_DisableIRQ(USB_LP_IRQn);
    if ((hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) &&
        (hUsbDeviceFS.pClassData == hcdc) &&
        (vofa_host_ready != 0U) && (vofa_host_request_pending == 0U))
    {
        vofa_tx_start_cycle = now_cycle;
        if (CDC_Transmit_FS(vofa_tx_buffer, packet_size) == USBD_OK)
        {
            vofa_send_time_update();
            vofa_pwm_packet_samples = capture_samples;
        }
    }
    HAL_NVIC_EnableIRQ(USB_LP_IRQn);
}

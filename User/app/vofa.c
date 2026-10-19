#include "vofa.h"

#include "common.h"
#include "encoder_zero_calibration.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "pid.h"
#include "pwm_capture.h"
#include "voltage_sample.h"

#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

#define VOFA_SEND_PERIOD_US       50U
#define VOFA_SEND_PERIOD_CYCLES \
    ((DWT_CPU_FREQ_HZ / 1000000UL) * VOFA_SEND_PERIOD_US)
#define VOFA_HOST_SETTLE_TIME_MS  10U
#define VOFA_TX_TIMEOUT_MS         500U
#define VOFA_TX_TIMEOUT_CYCLES \
    ((DWT_CPU_FREQ_HZ / 1000UL) * VOFA_TX_TIMEOUT_MS)
#define VOFA_JUSTFLOAT_TAIL_SIZE  4U
#define VOFA_MAX_CHANNEL_COUNT    18U

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

static void vofa_pwm_reset(void)
{
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
    const float vofa_values[] =
    {
        pm.foc.i_a,                  /* CH0: phase A current (A) */
        pm.foc.i_b,                  /* CH1: phase B current (A) */
        pm.foc.i_c,                  /* CH2: phase C current (A) */
//			voltage_sample.v_an_v,
//			voltage_sample.v_bn_v,
//			voltage_sample.v_cn_v,
//			voltage_sample.vbus_v,
//			voltage_sample.v_alpha,
//			voltage_sample.v_beta,
			voltage_sample.v_q,
			voltage_sample.v_d,
			//        pm.foc.u_alpha,                  /* CH3: d-axis current (A) */
//        pm.foc.u_beta,                  /* CH4: q-axis current (A) */
//			pm.foc.i_a_ref,                  /* CH0: phase A current (A) */
//        pm.foc.i_b_ref,                  /* CH1: phase B current (A) */
//        pm.foc.i_c_ref,  
			pm.foc.i_q,
//			pm.foc.i_q_ref,
			pm.foc.i_d,			
//			pm.foc.i_d_ref,
			pm.foc.u_d,
			pm.foc.u_q,
			
//			pm.foc.i_alpha,              /* CH5: alpha-axis current (A) */
//			pm.foc.i_beta,               /* CH6: beta-axis current (A) */

//			(float)ADC1->JDR1,
//				(float)ADC1->JDR2,
//					(float)ADC1->JDR3,
//						pm.adc.ia_off,
//					pm.adc.ib_off,
//					pm.adc.ic_off
//        (float)pm.svpwm.ccr1,        /* CH9: PWM compare 1 */
//        (float)pm.svpwm.ccr2,        /* CH10: PWM compare 2 */
//        (float)pm.svpwm.ccr3,        /* CH11: PWM compare 3 */
//			
//				pm.foc.mechanical_angle_rad, /* CH12: mechanical angle (rad) */
//				pm.foc.electrical_angle_rad,
			pm.speed.mechanical_rad_s,   /* CH12: mechanical speed (rad/s) */
			voltage_sample.v_neutral_v,  /* CH13: estimated motor neutral (V) */
			voltage_sample.v_an_v,       /* CH14: physical MA to neutral (V) */
			voltage_sample.v_bn_v,       /* CH15: physical MB to neutral (V) */
			voltage_sample.v_cn_v,       /* CH16: physical MC to neutral (V) */
//			foc_iq_pid.p_output,
//				foc_iq_pid.integral_output,
//				foc_id_pid.p_output,
//				foc_id_pid.integral_output

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
//        pm.current_sample.i_a_before_reconstruction,   /* CH14: A before reconstruction (A) */
//        pm.current_sample.i_b_before_reconstruction,   /* CH15: B before reconstruction (A) */
//        pm.current_sample.i_c_before_reconstruction,   /* CH16: C before reconstruction (A) */
//        pm.current_sample.i_sum_before_reconstruction /* CH17: A+B+C before reconstruction (A) */
//				pm.foc.i_a+pm.foc.i_b+pm.foc.i_c,
//				/*У׼*/
//				(float)pm.encoder_foc_sample.raw,
//				(float)calibration.sampling,
//				(float)calibration.done,
//				(float)calibration.sample_count,
//				(float)calibration.mechanical_zero_raw
//				(float)calibration.mechanical_zero_angle_rad
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
            pm.foc.i_a,                 /* CH3: live phase A current (A) */
            pm.foc.i_b,                 /* CH4: live phase B current (A) */
            pm.foc.i_c,                 /* CH5: live phase C current (A) */
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
            vofa_pwm_packet_samples = capture_samples;
        }
    }
    HAL_NVIC_EnableIRQ(USB_LP_IRQn);
}

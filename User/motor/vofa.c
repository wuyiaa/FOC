#include "vofa.h"

#include "common.h"
#include "encoder_zero_calibration.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

#define VOFA_SEND_PERIOD_US       50U
#define VOFA_SEND_PERIOD_CYCLES \
    ((DWT_CPU_FREQ_HZ / 1000000UL) * VOFA_SEND_PERIOD_US)
#define VOFA_JUSTFLOAT_TAIL_SIZE  4U
#define VOFA_MAX_CHANNEL_COUNT    15U

/*
 * Edit vofa_values[] in vofa_packet_build() to change the channel list/order.
 * Use (float)variable for integer or enum values.
 * Configure the same channel names/order in VOFA+ using the JustFloat parser.
 */
#define VOFA_BUFFER_SIZE \
    (VOFA_MAX_CHANNEL_COUNT * sizeof(float) + VOFA_JUSTFLOAT_TAIL_SIZE)

/* CDC retains the Tx buffer pointer until the USB IN transfer completes. */
static uint8_t vofa_tx_buffer[VOFA_BUFFER_SIZE];
static uint32_t vofa_last_send_cycle;

void vofa_init(void)
{
    vofa_last_send_cycle = DWT->CYCCNT;
}

static uint16_t vofa_packet_build(void)
{
    const float vofa_values[] =
    {
        pm.foc.i_a,                  /* CH0: phase A current (A) */
        pm.foc.i_b,                  /* CH1: phase B current (A) */
        pm.foc.i_c,                  /* CH2: phase C current (A) */
//        pm.foc.u_alpha,                  /* CH3: d-axis current (A) */
//        pm.foc.u_beta,                  /* CH4: q-axis current (A) */
//			pm.foc.i_a_ref,                  /* CH0: phase A current (A) */
//        pm.foc.i_b_ref,                  /* CH1: phase B current (A) */
//        pm.foc.i_c_ref,  
			pm.foc.i_q,
			pm.foc.i_d,
			pm.foc.i_q_ref,
			pm.foc.i_d_ref,
			
//			(float)ADC1->JDR1,
//				(float)ADC1->JDR2,
//					(float)ADC1->JDR3,
//						pm.adc.ia_off,
//					pm.adc.ib_off,
//					pm.adc.ic_off
//        pm.foc.electrical_angle_rad, /* CH5: electrical angle (rad) */
//        pm.foc.u_d,                  /* CH6: d-axis voltage command (V) */
//        pm.foc.u_q,                  /* CH7: q-axis voltage command (V) */
//        pm.foc.u_alpha,              /* CH8: alpha voltage command (V) */
//        pm.foc.u_beta,               /* CH9: beta voltage command (V) */
        (float)pm.svpwm.ccr1,        /* CH10: phase A PWM compare */
        (float)pm.svpwm.ccr2,        /* CH11: phase B PWM compare */
        (float)pm.svpwm.ccr3,         /* CH12: phase C PWM compare */
//			
				pm.foc.mechanical_angle_rad,
				pm.foc.electrical_angle_rad,
			
			
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

void vofa_process(void)
{
    USBD_CDC_HandleTypeDef *hcdc;
    uint32_t now_cycle;
    uint16_t packet_size;

    now_cycle = DWT->CYCCNT;
    if ((uint32_t)(now_cycle - vofa_last_send_cycle) <
        VOFA_SEND_PERIOD_CYCLES)
    {
        return;
    }
    vofa_last_send_cycle = now_cycle;

    /* CDC_Transmit_FS dereferences pClassData, so wait for USB enumeration. */
    if ((hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) ||
        (hUsbDeviceFS.pClassData == NULL))
    {
        return;
    }

    hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (hcdc->TxState != 0U)
    {
        /* Keep the static packet buffer unchanged while USB owns it. */
        return;
    }

    packet_size = vofa_packet_build();
    if (packet_size == 0U)
    {
        return;
    }

    /* Non-blocking: a busy CDC endpoint simply skips this sample frame. */
    (void)CDC_Transmit_FS(vofa_tx_buffer, packet_size);
}

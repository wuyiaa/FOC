#include "voltage_sample.h"

/* Keep the existing ADC2 ranks: ADSPE, VA, VB, VC, NTC2. */
#define VOLTAGE_REGULAR_CHANNEL_COUNT  5U
#define VOLTAGE_DMA_COUNT              (2U * VOLTAGE_REGULAR_CHANNEL_COUNT)
#define VOLTAGE_VA_INDEX               1U
#define VOLTAGE_VB_INDEX               2U
#define VOLTAGE_VC_INDEX               3U

volatile voltage_sample_t voltage_sample;
__ALIGN_BEGIN static volatile uint16_t voltage_adc2_dma[VOLTAGE_DMA_COUNT] __ALIGN_END;

HAL_StatusTypeDef voltage_sample_start(void)
{
    HAL_StatusTypeDef status;

    status = HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    if (status != HAL_OK)
    {
        return status;
    }

    /* PC5 / ADC2_IN11: arm the TIM1_CC4 falling-edge bus-voltage trigger.
       No extra injected IRQ: ADC1's three-rank callback collects this result. */
    status = HAL_ADCEx_InjectedStart(&hadc2);
    if (status != HAL_OK)
    {
        return status;
    }

    status = HAL_ADC_Start_DMA(&hadc2, (uint32_t *)voltage_adc2_dma,
                              VOLTAGE_DMA_COUNT);
    if (status != HAL_OK)
    {
        (void)HAL_ADCEx_InjectedStop(&hadc2);
        return status;
    }

    /* Continuous scans are faster than the FOC loop. Poll completed blocks
       there instead of generating a DMA interrupt for every half/full block.
       Transfer-error interrupts remain enabled. */
    __HAL_DMA_DISABLE_IT(hadc2.DMA_Handle, DMA_IT_HT | DMA_IT_TC);
    return HAL_OK;
}

static void voltage_sample_phase_update(void)
{
    DMA_HandleTypeDef *dma = hadc2.DMA_Handle;
    uint32_t remaining_before;
    uint32_t remaining_after;
    uint32_t offset;
    uint16_t va;
    uint16_t vb;
    uint16_t vc;

    remaining_before = __HAL_DMA_GET_COUNTER(dma);
    if ((remaining_before == 0U) || (remaining_before > VOLTAGE_DMA_COUNT))
    {
        return;
    }

    if (remaining_before > VOLTAGE_REGULAR_CHANNEL_COUNT)
    {
        /* DMA is writing the first scan. The second scan is usable only
           after the very first full-buffer completion (TC stays latched). */
        if (__HAL_DMA_GET_FLAG(dma, __HAL_DMA_GET_TC_FLAG_INDEX(dma)) == RESET)
        {
            return;
        }
        offset = VOLTAGE_REGULAR_CHANNEL_COUNT;
    }
    else
    {
        /* DMA is writing the second scan; the first scan is complete. */
        offset = 0U;
    }

    va = voltage_adc2_dma[offset + VOLTAGE_VA_INDEX];
    vb = voltage_adc2_dma[offset + VOLTAGE_VB_INDEX];
    vc = voltage_adc2_dma[offset + VOLTAGE_VC_INDEX];
    __DMB();
    remaining_after = __HAL_DMA_GET_COUNTER(dma);

    /* This bounded copy runs in the highest-priority ADC ISR. Reject it if
       DMA crossed a block boundary while the three values were copied. */
    if ((remaining_after == 0U) || (remaining_after > remaining_before) ||
        ((remaining_before > VOLTAGE_REGULAR_CHANNEL_COUNT) !=
         (remaining_after > VOLTAGE_REGULAR_CHANNEL_COUNT)))
    {
        return;
    }

    voltage_sample.va_raw = va;
    voltage_sample.vb_raw = vb;
    voltage_sample.vc_raw = vc;
    voltage_sample.va_v = (float)va * VOLTAGE_ADC_V_PER_COUNT;
    voltage_sample.vb_v = (float)vb * VOLTAGE_ADC_V_PER_COUNT;
    voltage_sample.vc_v = (float)vc * VOLTAGE_ADC_V_PER_COUNT;
    voltage_sample.phase_update_count++;
    voltage_sample.phase_valid = 1U;
}

void voltage_sample_update(void)
{
    uint16_t raw;

    voltage_sample.bus_fresh = 0U;
    voltage_sample.adc_error_code = HAL_ADC_GetError(&hadc2);
    if (voltage_sample.adc_error_code != HAL_ADC_ERROR_NONE)
    {
        voltage_sample.phase_valid = 0U;
        voltage_sample.bus_valid = 0U;
        return;
    }

    /* The ADC2 result is normally ready before ADC1's three-rank sequence.
       Check JEOS explicitly; never wait inside the current-loop interrupt. */
    if (__HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_JEOS) != RESET)
    {
        raw = (uint16_t)hadc2.Instance->JDR1;
        __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
        voltage_sample.vbus_raw = raw;
        voltage_sample.vbus_v = (float)raw * VOLTAGE_ADC_V_PER_COUNT;
        voltage_sample.bus_update_count++;
        voltage_sample.bus_valid = 1U;
        voltage_sample.bus_fresh = 1U;
    }
    else
    {
        /* Keep the previous value, but expose the missed update in Watch. */
        voltage_sample.bus_not_ready_count++;
    }

    voltage_sample_phase_update();
}

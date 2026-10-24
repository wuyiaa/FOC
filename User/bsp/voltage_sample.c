#include "voltage_sample.h"

#include <math.h>

/* ADC2 regular ranks: ADSPE, VA, VB, VC, NTC2. */
#define VOLTAGE_REGULAR_CHANNEL_COUNT  5U
#define VOLTAGE_DMA_COUNT              (2U * VOLTAGE_REGULAR_CHANNEL_COUNT)
#define VOLTAGE_VA_INDEX               1U
#define VOLTAGE_VB_INDEX               2U
#define VOLTAGE_VC_INDEX               3U

volatile voltage_sample_t voltage_sample;
__ALIGN_BEGIN static volatile uint16_t voltage_adc2_dma[VOLTAGE_DMA_COUNT] __ALIGN_END;

static void voltage_sample_clarke(float va, float vb, float vc,
                                  float *v_alpha, float *v_beta)
{
    /* Inputs are winding-to-neutral voltages in logical A/B/C order. */
    *v_alpha = (2.0f * va - vb - vc) * (1.0f / 3.0f);
    *v_beta = (vb - vc) * 0.577350269f;
}

static void voltage_sample_park(float v_alpha, float v_beta,
                                float electrical_angle_rad,
                                float *v_d, float *v_q)
{
    const float sin_theta = sinf(electrical_angle_rad);
    const float cos_theta = cosf(electrical_angle_rad);

    /* Match the current Park transform's electrical-angle convention. */
    *v_d = v_alpha * cos_theta + v_beta * sin_theta;
    *v_q = v_beta * cos_theta - v_alpha * sin_theta;
}

void voltage_sample_transform_update(float electrical_angle_rad,
                                     uint8_t phase_reversed,
                                     uint8_t angle_valid)
{
    static uint32_t last_phase_update_count;
    uint32_t saved_primask;
    uint32_t phase_update_count;
    uint8_t phase_valid;
    float va;
    float vb;
    float vc;
    float v_neutral;
    float v_an;
    float v_bn;
    float v_cn;
    float v_alpha;
    float v_beta;
    float v_d;
    float v_q;

    if (voltage_sample.phase_valid == 0U)
    {
        voltage_sample.transform_valid = 0U;
        return;
    }
    if (angle_valid == 0U)
    {
        voltage_sample.transform_valid = 0U;
    }
    if (voltage_sample.phase_update_count == last_phase_update_count)
    {
        return;
    }

    /* The ADC ISR publishes all three phase values. Copy one whole set. */
    saved_primask = __get_PRIMASK();
    __disable_irq();
    phase_update_count = voltage_sample.phase_update_count;
    phase_valid = voltage_sample.phase_valid;
    va = voltage_sample.Uag.value;
    vb = voltage_sample.Ubg.value;
    vc = voltage_sample.Ucg.value;
    __set_PRIMASK(saved_primask);

    if (phase_valid == 0U)
    {
        voltage_sample.transform_valid = 0U;
        return;
    }
    if (phase_update_count == last_phase_update_count)
    {
        return;
    }

    /* For a floating, balanced three-phase winding, the mean of the three
       bridge terminal voltages estimates the motor neutral potential. */
    v_neutral = (va + vb + vc) * (1.0f / 3.0f);
    v_an = va - v_neutral;
    v_bn = vb - v_neutral;
    v_cn = vc - v_neutral;

    voltage_sample.U_ng = v_neutral;
    voltage_sample.Uan = v_an;
    voltage_sample.Ubn = v_bn;
    voltage_sample.Ucn = v_cn;

    /* The physical winding values above keep MA/MB/MC order. The transform
       uses the same calibrated logical phase order as the current loop. */
    if (phase_reversed != 0U)
    {
        const float swap = v_an;
        v_an = v_cn;
        v_cn = swap;
    }

    voltage_sample_clarke(v_an, v_bn, v_cn, &v_alpha, &v_beta);

    voltage_sample.Ualpha_fdb = v_alpha;
    voltage_sample.Ubeta_fdb = v_beta;
    if (angle_valid == 0U)
    {
        voltage_sample.transform_valid = 0U;
        last_phase_update_count = phase_update_count;
        return;
    }

    voltage_sample_park(v_alpha, v_beta, electrical_angle_rad, &v_d, &v_q);
    voltage_sample.Ud_fdb = v_d;
    voltage_sample.Uq_fdb = v_q;
    voltage_sample.transform_update_count++;
    voltage_sample.transform_valid = 1U;
    last_phase_update_count = phase_update_count;
}

HAL_StatusTypeDef voltage_sample_start(void)
{
    HAL_StatusTypeDef status;

    status = HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Arm the ADC2 injected slave before starting its regular DMA and before
       ADC1 is armed. ADC1's TIM1 CC4 trigger samples IC with IA, then VBUS. */
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

/* Average accepted snapshots only; startup uses the available sample count. */
static void voltage_sample_average_update(uint16_t va, uint16_t vb, uint16_t vc)
{
    static uint16_t history[VOLTAGE_AVERAGE_SAMPLE_COUNT][3];
    static uint32_t sums[3];
    static uint32_t index;
    static uint32_t count;
    const uint16_t raw[3] = {va, vb, vc};
    uint32_t phase;
    float scale;

    for (phase = 0U; phase < 3U; phase++)
    {
        sums[phase] -= history[index][phase];
        history[index][phase] = raw[phase];
        sums[phase] += raw[phase];
    }
    index = (index + 1U) % VOLTAGE_AVERAGE_SAMPLE_COUNT;
    if (count < VOLTAGE_AVERAGE_SAMPLE_COUNT)
    {
        count++;
    }
    scale = VOLTAGE_ADC_V_PER_COUNT / (float)count;
    voltage_sample.Uag.average = (float)sums[0] * scale;
    voltage_sample.Ubg.average = (float)sums[1] * scale;
    voltage_sample.Ucg.average = (float)sums[2] * scale;
}

static uint8_t voltage_sample_regular_update(void)
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
        return 0U;
    }

    if (remaining_before > VOLTAGE_REGULAR_CHANNEL_COUNT)
    {
        /* DMA is writing the first scan. The second scan is usable only
           after the very first full-buffer completion (TC stays latched). */
        if (__HAL_DMA_GET_FLAG(dma, __HAL_DMA_GET_TC_FLAG_INDEX(dma)) == RESET)
        {
            return 0U;
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

    /* Reject this set if DMA crossed a block boundary during the copy. */
    if ((remaining_after == 0U) || (remaining_after > remaining_before) ||
        ((remaining_before > VOLTAGE_REGULAR_CHANNEL_COUNT) !=
         (remaining_after > VOLTAGE_REGULAR_CHANNEL_COUNT)))
    {
        return 0U;
    }

    voltage_sample.Uag.raw = va;
    voltage_sample.Ubg.raw = vb;
    voltage_sample.Ucg.raw = vc;
    voltage_sample.Uag.value = (float)va * VOLTAGE_ADC_V_PER_COUNT;
    voltage_sample.Ubg.value = (float)vb * VOLTAGE_ADC_V_PER_COUNT;
    voltage_sample.Ucg.value = (float)vc * VOLTAGE_ADC_V_PER_COUNT;
    voltage_sample_average_update(va, vb, vc);
    voltage_sample.phase_update_count++;
    voltage_sample.phase_valid = 1U;
    return 1U;
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
        voltage_sample.transform_valid = 0U;
        return;
    }

    /* Consume the latest completed ADC2 sequence, which may be from the
       previous PWM period, without waiting in the current-loop interrupt. */
    if (__HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_JEOS) != RESET)
    {
        raw = (uint16_t)hadc2.Instance->JDR2;
        __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
        voltage_sample.vbus_raw = raw;
        voltage_sample.vbus_v = (float)raw * VOLTAGE_ADC_V_PER_COUNT;
        voltage_sample.bus_update_count++;
        voltage_sample.bus_valid = 1U;
        voltage_sample.bus_fresh = 1U;
    }
    else
    {
        /* Keep the previous value when no new completed sequence is available. */
        voltage_sample.bus_not_ready_count++;
    }

    (void)voltage_sample_regular_update();
}

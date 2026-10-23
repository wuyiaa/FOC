#include "common.h"
#include "encoder_zero_calibration.h"
#include "phase_auto_calibration.h"

#include <math.h>

#if ((CURRENT_RECONSTRUCTION_MODE != CURRENT_RECONSTRUCTION_MAX_CURRENT) && \
     (CURRENT_RECONSTRUCTION_MODE != CURRENT_RECONSTRUCTION_MIN_WINDOW))
#error "Invalid CURRENT_RECONSTRUCTION_MODE"
#endif

static float foc_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8_t current_window_is_valid(const pmsm_current_sample_t *sample)
{
    if ((sample->window_a_valid == 0U) ||
        (sample->window_b_valid == 0U) ||
        (sample->window_c_valid == 0U))
    {
        return 0U;
    }

    return 1U;
}

static uint16_t current_window_get(uint16_t pwm_cmp)
{
    if (pwm_cmp >= PWM_PERIOD_TICKS)
    {
        return 0U;
    }

    /* Center-aligned PWM1: the low side conducts while CNT > CCR,
       on both the up-count and down-count portions of a PWM period.
       Nominal duration only: no deadtime, settling or trigger correction. */
    return (uint16_t)(2U * (PWM_PERIOD_TICKS - pwm_cmp));
}

static pmsm_phase_t max_current_phase_get(const pmsm_foc_t *foc)
{
    pmsm_phase_t phase = PMSM_PHASE_A;
    float current_max = foc_abs(foc->ia.nonrecon);
    float current_abs = foc_abs(foc->ib.nonrecon);

    if (current_abs > current_max)
    {
        current_max = current_abs;
        phase = PMSM_PHASE_B;
    }

    current_abs = foc_abs(foc->ic.nonrecon);
    if (current_abs > current_max)
    {
        phase = PMSM_PHASE_C;
    }

    return phase;
}

#if (CURRENT_RECONSTRUCTION_MODE == CURRENT_RECONSTRUCTION_MIN_WINDOW)
static pmsm_phase_t min_window_phase_get(const pmsm_current_sample_t *sample)
{
    pmsm_phase_t phase = PMSM_PHASE_A;
    uint16_t window_min = sample->window_a;

    if (sample->window_b < window_min)
    {
        window_min = sample->window_b;
        phase = PMSM_PHASE_B;
    }

    if (sample->window_c < window_min)
    {
        phase = PMSM_PHASE_C;
    }

    return phase;
}
#endif

static void current_phase_reconstruct(pmsm_foc_t *foc, pmsm_phase_t phase)
{
    switch (phase)
    {
        case PMSM_PHASE_A:
            foc->ia.reconstructed = -(foc->ib.nonrecon + foc->ic.nonrecon);
            break;

        case PMSM_PHASE_B:
            foc->ib.reconstructed = -(foc->ia.nonrecon + foc->ic.nonrecon);
            break;

        case PMSM_PHASE_C:
            foc->ic.reconstructed = -(foc->ia.nonrecon + foc->ib.nonrecon);
            break;

        case PMSM_PHASE_NONE:
        default:
            break;
    }
}

void foc_current_sample_process(pmsm_t *pm)
{
    pmsm_current_sample_t *sample = &pm->current_sample;

    sample->i_sum_before_reconstruction = pm->foc.ia.nonrecon +
        pm->foc.ib.nonrecon + pm->foc.ic.nonrecon;
	
    sample->pwm_cmp_a = (uint16_t)TIM1->CCR1;
    sample->pwm_cmp_b = (uint16_t)TIM1->CCR2;
    sample->pwm_cmp_c = (uint16_t)TIM1->CCR3;

    /* Window checks and reconstructed_phase use physical A/B/C. */

    sample->window_a = current_window_get(sample->pwm_cmp_a);
    sample->window_b = current_window_get(sample->pwm_cmp_b);
    sample->window_c = current_window_get(sample->pwm_cmp_c);

    sample->window_a_valid = (uint8_t)(sample->window_a >= ADC_SAMPLE_TOTAL_TICKS);
    sample->window_b_valid = (uint8_t)(sample->window_b >= ADC_SAMPLE_TOTAL_TICKS);
    sample->window_c_valid = (uint8_t)(sample->window_c >= ADC_SAMPLE_TOTAL_TICKS);
//    sample->window_valid = current_window_is_valid(sample);
    sample->full_clarke = 1U;
    sample->reconstructed_phase = PMSM_PHASE_NONE;

    if (sample->window_valid == 0U)
    {
        sample->reconstructed_phase = max_current_phase_get(&pm->foc);
        sample->full_clarke = 0U;
    }
#if (CURRENT_RECONSTRUCTION_MODE == CURRENT_RECONSTRUCTION_MIN_WINDOW)
//		if(sample->window_valid)//因为有三相都有效的时候，如果只是无效的时候启用，容易三相都有效的时候畸形
//		{
			sample->reconstructed_phase = min_window_phase_get(sample);
			sample->full_clarke = 0U;
//		}			
#endif
//sample->full_clarke = 1U;
    current_phase_reconstruct(&pm->foc, sample->reconstructed_phase);
}

void clarke_transform(pmsm_foc_t *foc, uint8_t full_clarke)
{
    /* Map physical currents to logical phases only at the control boundary. */
    const float ia = (calibration.phase_reversed != 0U) ?
        foc->ic.reconstructed : foc->ia.reconstructed;
    const float ib = foc->ib.reconstructed;
    const float ic = (calibration.phase_reversed != 0U) ?
        foc->ia.reconstructed : foc->ic.reconstructed;
    if (full_clarke != 0U)
    {
        foc->i_alpha = ia * TWO_BY_THREE - ib * ONE_BY_THREE - ic * ONE_BY_THREE;
        foc->i_beta = ib * ONE_BY_SQRT3 - ic * ONE_BY_SQRT3;
    }
    else
    {
        foc->i_alpha = ia;
        foc->i_beta = ia * ONE_BY_SQRT3 + ib * TWO_BY_SQRT3;
    }
}

void inverse_clarke_transform(pmsm_foc_t *foc)
{
    foc->i_a_ref = foc->i_alpha;
    foc->i_b_ref = -0.5f * foc->i_alpha + HALF_SQRT3 * foc->i_beta;
    foc->i_c_ref = -0.5f * foc->i_alpha - HALF_SQRT3 * foc->i_beta;
}

float theta_e;
static uint8_t foc_electrical_angle_get(pmsm_t *pm,
                                       float *sin_theta,
                                       float *cos_theta)
{
    uint32_t corrected_mechanical_count;
    uint32_t electrical_count;

    if (pm->encoder_foc_sample.valid == 0U)
    {
        return 0U;
    }
    /* Keep the DMA-published raw angle untouched; compensate only for FOC. */
    corrected_mechanical_count =
        ((uint32_t)pm->encoder_foc_sample.raw +
         (uint32_t)AS5047P_DATA_MASK + 1U -
         (uint32_t)calibration.mechanical_zero_raw) &
        (uint32_t)AS5047P_DATA_MASK;
    electrical_count =
        (corrected_mechanical_count * (uint32_t)pm->motor.pn) &
        AS5047P_DATA_MASK;
		
		pm->foc.mechanical_angle_rad = MOTOR_MECHANICAL_DRV *
        (float)corrected_mechanical_count * AS5047P_RAD_PER_COUNT;
    /* Both Park and inverse Park use electrical zero during zero calibration. */
    if (encoder_zero_calibration_requested != 0U)
    {
        pm->foc.electrical_angle_rad = 0.0f;
    }
    else
    {
        pm->foc.electrical_angle_rad = (float)calibration.electrical_direction *
            (float)electrical_count * AS5047P_RAD_PER_COUNT;
    }
//theta_e +=0.00005*10;
//pm->foc.electrical_angle_rad=theta_e;
    *sin_theta = sinf(pm->foc.electrical_angle_rad);
    *cos_theta = cosf(pm->foc.electrical_angle_rad);

    return 1U;
}

void park_transform(pmsm_t *pm)
{
    float sin_theta;
    float cos_theta;

    if (foc_electrical_angle_get(pm, &sin_theta, &cos_theta) == 0U)
    {
        return;
    }

    pm->foc.id_fdb = pm->foc.i_alpha * cos_theta
                + pm->foc.i_beta * sin_theta;
    pm->foc.iq_fdb = pm->foc.i_beta * cos_theta
                - pm->foc.i_alpha * sin_theta;
}

void foc_current_calc(pmsm_t *pm)
{
    clarke_transform(&pm->foc, pm->current_sample.full_clarke);
    park_transform(pm);
}

void inverse_park_transform(pmsm_t *pm)
{
    float sin_theta;
    float cos_theta;

    if (foc_electrical_angle_get(pm, &sin_theta, &cos_theta) == 0U)
    {
        return;
    }

//		pm->foc.Ud_ref = 0;
//		pm->foc.Uq_ref = 0.66;
		
    /* Inverse of the Park convention used above. */
    pm->foc.u_alpha = pm->foc.Ud_ref * cos_theta
                    - pm->foc.Uq_ref * sin_theta;
    pm->foc.u_beta  = pm->foc.Ud_ref * sin_theta
                    + pm->foc.Uq_ref * cos_theta;
//		pm->foc.u_alpha = 2.5f;//2.5f; 5.0f
//		pm->foc.u_beta  = 4.33f;//4.33f 0.0f
}

static uint16_t svpwm_ccr_from_time(float phase_time, float period)
{
    float duty;

    duty = phase_time / period;
    if (duty < 0.0f)
    {
        duty = 0.0f;
    }
    else if (duty > 1.0f)
    {
        duty = 1.0f;
    }

    return (uint16_t)(duty * (float)PWM_PERIOD_TICKS);
}

void svpwm_calc(pmsm_t *pm, float v_dc, float v_min)
{
    static uint8_t sector_last = 2U;
    float u_alpha = pm->foc.u_alpha;
    float u_beta = pm->foc.u_beta;
    float period = PWM_PERIOD_S;
    float vref2 = u_alpha * u_alpha + u_beta * u_beta;
    float u1;
    float u2;
    float u3;
    float x;
    float y;
    float z;
    float k;
    float t1 = 0.0f;
    float t2 = 0.0f;
    float t0;
    float ta;
    float tb;
    float tc;
    float scale;
    uint8_t a = 0U;
    uint8_t b = 0U;
    uint8_t c = 0U;
    uint8_t n;
    uint8_t sector = 0U;

    pm->svpwm.vref2 = vref2;

    /* Preserve the Simulink invalid-input behavior: outputs remain zero. */
    pm->svpwm.ccr1 = 0U;
    pm->svpwm.ccr2 = 0U;
    pm->svpwm.ccr3 = 0U;
    pm->svpwm.sector = 0U;
    if ((v_dc <= 1.0e-12f) || (period <= 1.0e-12f))
    {
        return;
    }

    if (vref2 < (v_min * v_min))
    {
        pm->svpwm.ccr1 = (uint16_t)(PWM_PERIOD_TICKS >> 1U);
        pm->svpwm.ccr2 = (uint16_t)(PWM_PERIOD_TICKS >> 1U);
        pm->svpwm.ccr3 = (uint16_t)(PWM_PERIOD_TICKS >> 1U);
        pm->svpwm.sector = sector_last;
        return;
    }

    /* Sector bits and lookup mapping are kept identical to the supplied model. */
    u1 = u_beta;
    u2 = HALF_SQRT3 * u_alpha - 0.5f * u_beta;
    u3 = -HALF_SQRT3 * u_alpha - 0.5f * u_beta;
    if (u1 > 0.0f)
    {
        a = 1U;
    }
    if (u2 > 0.0f)
    {
        b = 1U;
    }
    if (u3 > 0.0f)
    {
        c = 1U;
    }
    n = (uint8_t)(a + (uint8_t)(2U * b) + (uint8_t)(4U * c));

    switch (n)
    {
        case 3U: sector = 1U; break;
        case 1U: sector = 2U; break;
        case 5U: sector = 3U; break;
        case 4U: sector = 4U; break;
        case 6U: sector = 5U; break;
        case 2U: sector = 6U; break;
        default: sector = 0U; break;
    }

    if (sector == 0U)
    {
        pm->svpwm.ccr1 = (uint16_t)(PWM_PERIOD_TICKS >> 1U);
        pm->svpwm.ccr2 = (uint16_t)(PWM_PERIOD_TICKS >> 1U);
        pm->svpwm.ccr3 = (uint16_t)(PWM_PERIOD_TICKS >> 1U);
        return;
    }

    k = SQRT3 * period / v_dc;
    x = k * u_beta;
    y = k * (HALF_SQRT3 * u_alpha + 0.5f * u_beta);
    z = k * (-HALF_SQRT3 * u_alpha + 0.5f * u_beta);

    switch (sector)
    {
        case 1U: t1 = -z; t2 = x;  break;
        case 2U: t1 = y;  t2 = z;  break;
        case 3U: t1 = x;  t2 = -y; break;
        case 4U: t1 = z;  t2 = -x; break;
        case 5U: t1 = -y; t2 = -z; break;
        case 6U: t1 = -x; t2 = y;  break;
        default: break;
    }

    if (t1 < 0.0f)
    {
        t1 = 0.0f;
    }
    if (t2 < 0.0f)
    {
        t2 = 0.0f;
    }

    if ((t1 + t2) > period)
    {
        scale = period / (t1 + t2);
        t1 *= scale;
        t2 *= scale;
    }

    t0 = period - t1 - t2;
    switch (sector)
    {
        case 1U: ta = t1 + t2 + t0 * 0.5f; tb = t2 + t0 * 0.5f; tc = t0 * 0.5f; break;
        case 2U: ta = t1 + t0 * 0.5f; tb = t1 + t2 + t0 * 0.5f; tc = t0 * 0.5f; break;
        case 3U: ta = t0 * 0.5f; tb = t1 + t2 + t0 * 0.5f; tc = t2 + t0 * 0.5f; break;
        case 4U: ta = t0 * 0.5f; tb = t1 + t0 * 0.5f; tc = t1 + t2 + t0 * 0.5f; break;
        case 5U: ta = t2 + t0 * 0.5f; tb = t0 * 0.5f; tc = t1 + t2 + t0 * 0.5f; break;
        case 6U: ta = t1 + t2 + t0 * 0.5f; tb = t0 * 0.5f; tc = t1 + t0 * 0.5f; break;
        default: ta = period * 0.5f; tb = period * 0.5f; tc = period * 0.5f; break;
    }

    pm->svpwm.ccr1 = svpwm_ccr_from_time(ta, period);
    pm->svpwm.ccr2 = svpwm_ccr_from_time(tb, period);
    pm->svpwm.ccr3 = svpwm_ccr_from_time(tc, period);
    pm->svpwm.sector = sector;
    sector_last = sector;
}

void foc_voltage_calc(pmsm_t *pm, float v_dc, float v_min)
{
#if (FOC_PHASE_AUTO_CALIBRATION_MODE != 0U)
    foc_phase_auto_calibration_step(pm, v_dc, v_min);
#elif (FOC_ENCODER_ZERO_CALIBRATION_MODE != 0U)
    foc_encoder_zero_calibration_step(pm, v_dc, v_min);
#else
    if (pm->encoder_foc_sample.valid == 0U)
    {
        return;
    }

    inverse_park_transform(pm);
		inverse_clarke_transform(&pm->foc);
    svpwm_calc(pm, v_dc, v_min);
#endif
}

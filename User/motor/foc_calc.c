#include "common.h"

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
    if ((sample->window_a < ADC_SAMPLE_TOTAL_TICKS) ||
        (sample->window_b < ADC_SAMPLE_TOTAL_TICKS) ||
        (sample->window_c < ADC_SAMPLE_TOTAL_TICKS))
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

    if (pwm_cmp <= PWM_HALF_PERIOD_TICKS)
    {
        return pwm_cmp;
    }

    return (uint16_t)(PWM_PERIOD_TICKS - pwm_cmp);
}

static pmsm_phase_t max_current_phase_get(const pmsm_foc_t *foc)
{
    pmsm_phase_t phase = PMSM_PHASE_A;
    float current_max = foc_abs(foc->i_a);
    float current_abs = foc_abs(foc->i_b);

    if (current_abs > current_max)
    {
        current_max = current_abs;
        phase = PMSM_PHASE_B;
    }

    current_abs = foc_abs(foc->i_c);
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
            foc->i_a = -(foc->i_b + foc->i_c);
            break;

        case PMSM_PHASE_B:
            foc->i_b = -(foc->i_a + foc->i_c);
            break;

        case PMSM_PHASE_C:
            foc->i_c = -(foc->i_a + foc->i_b);
            break;

        case PMSM_PHASE_NONE:
        default:
            break;
    }
}

void foc_current_sample_process(pmsm_t *pm)
{
    pmsm_current_sample_t *sample = &pm->current_sample;

    sample->pwm_cmp_a = (uint16_t)TIM1->CCR1;
    sample->pwm_cmp_b = (uint16_t)TIM1->CCR2;
    sample->pwm_cmp_c = (uint16_t)TIM1->CCR3;

    sample->window_a = current_window_get(sample->pwm_cmp_a);
    sample->window_b = current_window_get(sample->pwm_cmp_b);
    sample->window_c = current_window_get(sample->pwm_cmp_c);

    sample->window_valid = current_window_is_valid(sample);
    sample->full_clarke = 1U;
    sample->reconstructed_phase = PMSM_PHASE_NONE;

    if (sample->window_valid == 0U)
    {
        sample->reconstructed_phase = max_current_phase_get(&pm->foc);
        sample->full_clarke = 0U;
    }
#if (CURRENT_RECONSTRUCTION_MODE == CURRENT_RECONSTRUCTION_MIN_WINDOW)
    else
    {
        sample->reconstructed_phase = min_window_phase_get(sample);
        sample->full_clarke = 0U;
    }
#endif

    current_phase_reconstruct(&pm->foc, sample->reconstructed_phase);
}

void clarke_transform(pmsm_foc_t *foc, uint8_t full_clarke)
{
    if (full_clarke != 0U)
    {
        foc->i_alpha = foc->i_a * TWO_BY_THREE
                     - foc->i_b * ONE_BY_THREE
                     - foc->i_c * ONE_BY_THREE;
        foc->i_beta = foc->i_b * ONE_BY_SQRT3
                    - foc->i_c * ONE_BY_SQRT3;
    }
    else
    {
        foc->i_alpha = foc->i_a;
        foc->i_beta = foc->i_a * ONE_BY_SQRT3
                    + foc->i_b * TWO_BY_SQRT3;
    }
}

void foc_current_calc(pmsm_t *pm)
{
    clarke_transform(&pm->foc, pm->current_sample.full_clarke);
}

#include "common.h"
#include "encoder_zero_calibration.h"

#include <string.h>


pmsm_t pm;

static uint8_t foc_pwm_channels_started;

HAL_StatusTypeDef foc_pwm_output_enable(pmsm_t *pm)
{
    if ((pm == NULL) ||
        (pm->encoder.fault != 0U) ||
        (pm->foc.current_limit_fault != 0U))
    {
        return HAL_ERROR;
    }

    if (foc_pwm_channels_started != 0U)
    {
        __HAL_TIM_MOE_ENABLE(&htim1);
        return HAL_OK;
    }

    /* Start from a balanced zero vector until the first valid FOC update. */
    pm->svpwm.ccr1 = PWM_HALF_PERIOD_TICKS;
    pm->svpwm.ccr2 = PWM_HALF_PERIOD_TICKS;
    pm->svpwm.ccr3 = PWM_HALF_PERIOD_TICKS;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pm->svpwm.ccr1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pm->svpwm.ccr2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pm->svpwm.ccr3);

    /* Latch all three new compare values on the same timer update event. */
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_2);
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_3);

    if ((HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) ||
        (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) ||
        (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) ||
        (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK))
    {
        /* Keep the gate driver outputs disabled if startup partially fails. */
        __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
        return HAL_ERROR;
    }

    foc_pwm_channels_started = 1U;
    return HAL_OK;
}

void foc_pwm_output_disable(void)
{
    /* Keep TIM1 running: CH4 still generates the ADC injected trigger. */
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
}

void foc_pwm_output_update(const pmsm_t *pm)
{
    if ((pm == NULL) ||
        (foc_pwm_channels_started == 0U) ||
        (pm->encoder.fault != 0U) ||
        (pm->foc.current_limit_fault != 0U))
    {
        return;
    }

    /* CCR preload applies the three phase duties together at a timer update. */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pm->svpwm.ccr1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pm->svpwm.ccr2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pm->svpwm.ccr3);
}

void pmsm_board_init(void)
{
    pm.board.v_ref = 3.3f;
    pm.board.v_adc = 4096.0f;

    /* SCH_axdr_v1.2: 5 mOhm shunt and current amplifier gain of 20. */
    pm.board.i_res = 0.005f;
    pm.board.i_op = 20.0f;

     pm.board.i_ratio = pm.board.v_ref / (pm.board.v_adc* pm.board.i_res* pm.board.i_op);
    pm.board.i_max = pm.board.v_adc * pm.board.i_ratio * 0.5f;
//	  float ia_positive_max =
//      (4095.0f - pm.adc.ia_off) * pm.board.i_ratio;

//  float ia_negative_max =
//      pm.adc.ia_off * pm.board.i_ratio;

}

void pmsm_motor_init(void)
{
    /* Motor parameters use SI units: H, Ohm and pole pairs. */
    pm.motor.lq = 0.005698f;
    pm.motor.ld = 0.002846f;
    pm.motor.rs = 6.617f;
    pm.motor.pn = 11U;
}

void pmsm_init(void)
{
    memset(&pm, 0, sizeof(pm));
#if (FOC_ENCODER_ZERO_CALIBRATION_MODE != 0U)
    foc_encoder_zero_calibration_init();
#endif
    /* Restore the last calibrated offset for normal FOC operation. */
    (void)encoder_zero_flash_load(&calibration.mechanical_zero_raw);
    pmsm_board_init();
    pmsm_motor_init();
    as5047p_init(&pm.encoder);
    foc_get_curr_off();
}

void foc_get_curr_off(void)
{
    float sum_a = 0.0f;
    float sum_b = 0.0f;
    float sum_c = 0.0f;
    uint32_t i;

    for (i = 0U; i < 1000; i++)
    {
        HAL_Delay(1U);
        sum_a += (float)ADC1->JDR3;
        sum_b += (float)ADC1->JDR2;
        sum_c += (float)ADC1->JDR1;
    }

    pm.adc.ia_off = sum_a * 0.001f;
    pm.adc.ib_off = sum_b * 0.001f;
    pm.adc.ic_off = sum_c * 0.001f;
    pm.adc.offset_ready = 1U;
}

void foc_adc_sample(pmsm_t *pm)
{
    pm->adc.ia = (uint16_t)ADC1->JDR3;
    pm->adc.ib = (uint16_t)ADC1->JDR2;
    pm->adc.ic = (uint16_t)ADC1->JDR1;

    pm->foc.i_a = ((float)pm->adc.ia - pm->adc.ia_off) * pm->board.i_ratio;
    pm->foc.i_b = ((float)pm->adc.ib - pm->adc.ib_off) * pm->board.i_ratio;
    pm->foc.i_c = ((float)pm->adc.ic - pm->adc.ic_off) * pm->board.i_ratio;
}

#include "common.h"
#include "encoder_zero_calibration.h"

#include <string.h>


pmsm_t pm;

static uint8_t foc_pwm_channels_started;
/* ADC channel offsets stay in physical A/B/C order across phase changes. */
static float physical_current_offset[3];

HAL_StatusTypeDef foc_pwm_output_enable(pmsm_t *pm)
{
    if ((pm == NULL) ||
        (pm->encoder.fault != 0U) ||
        (pm->foc.current_limit_fault != 0U))
    {
        return HAL_ERROR;
    }

#if ((FOC_ENCODER_ZERO_CALIBRATION_MODE != 0U) || \
     (FOC_PHASE_AUTO_CALIBRATION_MODE != 0U))
    if ((calibration.done != 0U) || (calibration.failed != 0U))
    {
        return HAL_ERROR;
    }
#endif

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

uint8_t foc_pwm_output_is_enabled(void)
{
    /* Starting CH4 for ADC triggering also sets MOE in HAL, even before
       the motor PWM channels CH1-CH3 have been started. */
    return (uint8_t)((foc_pwm_channels_started != 0U) &&
                     ((htim1.Instance->BDTR & TIM_BDTR_MOE) != 0U));
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

    /* Reverse means 123 <-> 321: swap A/C, never a cyclic 120-degree shift. */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                          (calibration.phase_reversed != 0U) ?
                          pm->svpwm.ccr3 : pm->svpwm.ccr1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pm->svpwm.ccr2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
                          (calibration.phase_reversed != 0U) ?
                          pm->svpwm.ccr1 : pm->svpwm.ccr3);
//		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pm->svpwm.ccr1);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pm->svpwm.ccr2);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pm->svpwm.ccr3);
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
//      (4095.0f - pm.foc.ia.offset) * pm.board.i_ratio;

//  float ia_negative_max =
//      pm.foc.ia.offset * pm.board.i_ratio;

}

void pmsm_motor_init(void)
{
    /* Motor parameters use SI units: H, Ohm and pole pairs. */
    pm.motor.lq = FOC_MOTOR_LQ_H;
    pm.motor.ld = FOC_MOTOR_LD_H;
    pm.motor.rs = FOC_MOTOR_RS_OHM;
    pm.motor.phif = 0.014f;
    pm.motor.inertia_kg_m2 = 0.008f;
    pm.motor.speed_damping_ratio = 1.0f;
    pm.motor.pn = FOC_MOTOR_POLE_PAIRS;
}

void pmsm_init(void)
{
    memset(&pm, 0, sizeof(pm));
    calibration.current_direction = 1;
    calibration.phase_reversed = MOTOR_PHASE_REVERSED_DEFAULT;
    calibration.electrical_direction = MOTOR_ELECTRICAL_DIRECTION_DEFAULT;
#if (FOC_PHASE_AUTO_CALIBRATION_MODE != 0U)
    (void)encoder_zero_flash_load(&calibration);
    foc_encoder_zero_calibration_init();
#elif (FOC_ENCODER_ZERO_CALIBRATION_MODE != 0U)
    /* Load the saved zero; fixed phase/direction are selected below. */
    (void)encoder_zero_flash_load(&calibration);
    foc_encoder_zero_calibration_init();
#else
    /* Restore the last calibrated offset for normal FOC operation. */
    (void)encoder_zero_flash_load(&calibration);
#endif
//#if (FOC_PHASE_AUTO_CALIBRATION_MODE == 0U)
//    /* Keep the Flash zero offset, but use the selected phase order and angle
//       direction even if the saved record contains older values. */
//    calibration.phase_reversed = MOTOR_PHASE_REVERSED_DEFAULT;
//    calibration.electrical_direction = MOTOR_ELECTRICAL_DIRECTION_DEFAULT;
//#endif
    pmsm_board_init();
    pmsm_motor_init();
    foc_current_pid_init();
    foc_speed_pid_init(&pm);
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
        sum_a += (float)ADC1->JDR1;
        sum_b += (float)ADC1->JDR2;
        sum_c += (float)ADC2->JDR1;
//			  sum_a += (float)ADC1->JDR2;
//        sum_b += (float)ADC1->JDR1;
//        sum_c += (float)ADC1->JDR3;
    }

    physical_current_offset[0] = sum_a * 0.001f;
    physical_current_offset[1] = sum_b * 0.001f;
    physical_current_offset[2] = sum_c * 0.001f;
    pm.foc.ia.offset = (calibration.phase_reversed != 0U) ?
        physical_current_offset[2] : physical_current_offset[0];
    pm.foc.ib.offset = physical_current_offset[1];
    pm.foc.ic.offset = (calibration.phase_reversed != 0U) ?
        physical_current_offset[0] : physical_current_offset[2];
    pm.adc.offset_ready = 1U;
}

static void phase_current_convert(pmsm_phase_current_t *current, float ratio,
                                  int8_t direction)
{
    current->with_offset = (float)current->raw * ratio;
    current->nonrecon = ((float)current->raw - current->offset) * ratio *
                        (float)direction;
    current->reconstructed = current->nonrecon;
}

void foc_adc_sample(pmsm_t *pm)
{
    /* Map ADC channels and their offsets to logical phases together. */
    pm->foc.ia.raw = (calibration.phase_reversed != 0U) ?
        (uint16_t)ADC2->JDR1 : (uint16_t)ADC1->JDR1;
    pm->foc.ib.raw = (uint16_t)ADC1->JDR2;
    pm->foc.ic.raw = (calibration.phase_reversed != 0U) ?
        (uint16_t)ADC1->JDR1 : (uint16_t)ADC2->JDR1;
    pm->foc.ia.offset = (calibration.phase_reversed != 0U) ?
        physical_current_offset[2] : physical_current_offset[0];
    pm->foc.ib.offset = physical_current_offset[1];
    pm->foc.ic.offset = (calibration.phase_reversed != 0U) ?
        physical_current_offset[0] : physical_current_offset[2];
    phase_current_convert(&pm->foc.ia, pm->board.i_ratio, calibration.current_direction);
    phase_current_convert(&pm->foc.ib, pm->board.i_ratio, calibration.current_direction);
    phase_current_convert(&pm->foc.ic, pm->board.i_ratio, calibration.current_direction);
}

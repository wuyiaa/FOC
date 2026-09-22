#include "common.h"

#include <string.h>


pmsm_t pm;

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

void pmsm_init(void)
{
    memset(&pm, 0, sizeof(pm));
    pmsm_board_init();
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

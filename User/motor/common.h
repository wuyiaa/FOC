#ifndef USER_MOTOR_COMMON_H
#define USER_MOTOR_COMMON_H

#include "main.h"
#include "adc.h"
#include "tim.h"

typedef struct
{
    uint16_t ia;
    uint16_t ib;
    uint16_t ic;

    float ia_off;
    float ib_off;
    float ic_off;
} pmsm_adc_val_t;

typedef struct
{
    float v_ref;
    float v_adc;
    float i_res;
    float i_op;
    float i_ratio;
    float i_max;
} pmsm_board_t;

typedef struct
{
    float i_a;
    float i_b;
    float i_c;
} pmsm_foc_t;

typedef struct
{
    pmsm_board_t board;
    pmsm_adc_val_t adc;
    pmsm_foc_t foc;
} pmsm_t;

extern pmsm_t pm;

void pmsm_board_init(void);
void pmsm_init(void);
void foc_get_curr_off(void);
void foc_adc_sample(pmsm_t *pm);

#endif /* USER_MOTOR_COMMON_H */

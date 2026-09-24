#ifndef USER_MOTOR_ENCODER_ZERO_CALIBRATION_H
#define USER_MOTOR_ENCODER_ZERO_CALIBRATION_H

#include "common.h"

extern volatile pmsm_encoder_zero_cal_t calibration;

HAL_StatusTypeDef encoder_zero_flash_load(volatile uint16_t *raw_offset);
HAL_StatusTypeDef encoder_zero_flash_save(uint16_t raw_offset);

#if (FOC_ENCODER_ZERO_CALIBRATION_MODE != 0U)
void foc_encoder_zero_calibration_init(void);
void foc_encoder_zero_calibration_step(pmsm_t *pm, float v_dc, float v_min);
#endif

#endif /* USER_MOTOR_ENCODER_ZERO_CALIBRATION_H */

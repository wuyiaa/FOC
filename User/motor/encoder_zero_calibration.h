#ifndef USER_MOTOR_ENCODER_ZERO_CALIBRATION_H
#define USER_MOTOR_ENCODER_ZERO_CALIBRATION_H

#include "common.h"

extern volatile pmsm_encoder_zero_cal_t calibration;
/* 1: run zero calibration; cleared when complete or aborted by a fault. */
extern volatile uint8_t encoder_zero_calibration_requested;

HAL_StatusTypeDef encoder_zero_flash_load(volatile pmsm_encoder_zero_cal_t *config);
HAL_StatusTypeDef encoder_zero_flash_save(const volatile pmsm_encoder_zero_cal_t *config);

/* Used by flash_save to retain the latest complete encoder record. */
uint8_t encoder_zero_flash_record_valid(uint64_t record);

#if ((FOC_ENCODER_ZERO_CALIBRATION_MODE != 0U) || \
     (FOC_PHASE_AUTO_CALIBRATION_MODE != 0U))
void foc_encoder_zero_calibration_init(void);

void foc_encoder_zero_calibration_step(pmsm_t *pm, float v_dc, float v_min);
#endif

#endif /* USER_MOTOR_ENCODER_ZERO_CALIBRATION_H */

#ifndef USER_MOTOR_PHASE_AUTO_CALIBRATION_H
#define USER_MOTOR_PHASE_AUTO_CALIBRATION_H

#include "common.h"

#if (FOC_PHASE_AUTO_CALIBRATION_MODE != 0U)
void foc_phase_auto_calibration_step(pmsm_t *pm, float v_dc, float v_min);
#endif

#endif /* USER_MOTOR_PHASE_AUTO_CALIBRATION_H */

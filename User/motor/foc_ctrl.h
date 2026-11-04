#ifndef USER_MOTOR_FOC_CTRL_H
#define USER_MOTOR_FOC_CTRL_H

#include "common.h"

/* Current-loop feedback filter parameters. */
#define ID_FILTER_CUTOFF_HZ 1.0f
#define ID_FILTER_ALPHA \
    ((TWO_PI * ID_FILTER_CUTOFF_HZ * PWM_PERIOD_S) / \
     (1.0f + TWO_PI * ID_FILTER_CUTOFF_HZ * PWM_PERIOD_S))

#endif /* USER_MOTOR_FOC_CTRL_H */

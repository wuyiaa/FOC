#ifndef USER_MOTOR_MOTOR_SPEED_H
#define USER_MOTOR_MOTOR_SPEED_H

#include <stdint.h>

#include "as5047p.h"

typedef struct
{
    volatile float mechanical_rad_s_ref; /* Speed target, rad/s. */
    float mechanical_rad_s;
    float mechanical_rad_s_filtered; /* First-order low-pass output, rad/s. */
    float electrical_rad_s;
	  float electrical_rad_s_filtered;
    uint16_t last_raw;
    //uint32_t last_sample_cycle;
    //uint32_t last_sequence;
    uint8_t initialized;
} motor_speed_t;

/* Called by TIM7 every 500 us; the first sample initializes last_raw. */
void motor_speed_update(motor_speed_t *speed,
                        const as5047p_sample_t *sample,
                        uint8_t pole_pairs);

#endif /* USER_MOTOR_MOTOR_SPEED_H */

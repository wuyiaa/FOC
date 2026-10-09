#ifndef USER_MOTOR_MOTOR_SPEED_H
#define USER_MOTOR_MOTOR_SPEED_H

#include <stdint.h>

#include "as5047p.h"

typedef struct
{
    float mechanical_rad_s;
    float electrical_rad_s;
    uint16_t last_raw;
    uint32_t last_sample_cycle;
    uint32_t last_sequence;
    uint8_t initialized;
} motor_speed_t;

void motor_speed_update(motor_speed_t *speed,
                        const as5047p_sample_t *sample,
                        uint8_t pole_pairs,
                        uint32_t now_cycle);

#endif /* USER_MOTOR_MOTOR_SPEED_H */

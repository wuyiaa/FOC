#ifndef USER_MOTOR_MOTOR_SPEED_H
#define USER_MOTOR_MOTOR_SPEED_H

#include <stdint.h>

#include "as5047p.h"

//#define MOTOR_SPEED_WINDOW_CYCLES  (DWT_CPU_FREQ_HZ / 1000UL)
//#define MOTOR_SPEED_STALE_CYCLES   (DWT_CPU_FREQ_HZ / 100UL)
#define MOTOR_SPEED_UPDATE_PERIOD_S  0.0005f
#define MOTOR_SPEED_FILTER_CUTOFF_HZ 100.0f
#define MOTOR_SPEED_COUNTS_PER_REV (AS5047P_DATA_MASK + 1U)
#define MOTOR_SPEED_HALF_REV       (MOTOR_SPEED_COUNTS_PER_REV / 2U)
#define MOTOR_SPEED_FILTER_ALPHA \
    ((TWO_PI * MOTOR_SPEED_FILTER_CUTOFF_HZ * MOTOR_SPEED_UPDATE_PERIOD_S) / \
     (1.0f + TWO_PI * MOTOR_SPEED_FILTER_CUTOFF_HZ * MOTOR_SPEED_UPDATE_PERIOD_S))

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

/* Clear feedback/history; preserve mechanical_rad_s_ref. */
void motor_speed_reset(motor_speed_t *speed);

/* Called by TIM7 every 500 us; the first sample initializes last_raw. */
void motor_speed_update(motor_speed_t *speed,
                        const as5047p_sample_t *sample,
                        uint8_t pole_pairs);

#endif /* USER_MOTOR_MOTOR_SPEED_H */

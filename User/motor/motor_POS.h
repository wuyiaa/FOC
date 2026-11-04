#ifndef USER_MOTOR_POS_H
#define USER_MOTOR_POS_H

#define MOTOR_POS_UPDATE_PERIOD_S  0.001f

typedef struct
{
    volatile float mechanical_angle_rad_ref;
    float mechanical_angle_rad_fdb;
} motor_pos_t;

/* Called by the TIM3 update callback every 1 ms (1 kHz). */
void motor_pos_update(void);

#endif /* USER_MOTOR_POS_H */

#ifndef USER_MOTOR_PID_H
#define USER_MOTOR_PID_H

#include <stdint.h>

#define PID_PARAMETER_COUNT  6U

typedef struct
{
    float kp;
    float ki;
    float kd;
    float output_limit;
    float integral_limit;
    float deadband;

    float integral_output;
    float previous_error;
    float output;
} pid_controller_t;

/* Parameter order: Kp, Ki, Kd, output limit, integral limit, deadband. */
void pid_init(pid_controller_t *pid, const float parameters[PID_PARAMETER_COUNT]);
float pid_calculate(pid_controller_t *pid,
                    float target,
                    float feedback,
                    float dt);

#endif /* USER_MOTOR_PID_H */

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
		float p_output;
    float previous_error;
    float output;
} pid_controller_t;

extern pid_controller_t foc_id_pid;
extern pid_controller_t foc_iq_pid;
/* Keil Watch: [0] Kp, [1] Ki, [2] Kd, [3..5] limits/deadband.
 * Read by the current loop each cycle. Bandwidth changes replace Kp/Ki;
 * manual gain edits remain active until the bandwidth changes again. */
extern volatile float foc_id_pid_parameters[PID_PARAMETER_COUNT];
extern volatile float foc_iq_pid_parameters[PID_PARAMETER_COUNT];
/* Parameter order: Kp, Ki, Kd, output limit, integral limit, deadband. */
void pid_init(pid_controller_t *pid, const volatile float parameters[PID_PARAMETER_COUNT]);
float pid_calculate(pid_controller_t *pid,
                    float target,
                    float feedback,
                    float dt);

#endif /* USER_MOTOR_PID_H */

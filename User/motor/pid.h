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
	
		float error;
    float integral_output;
		float p_output;
    float previous_error;
    volatile float output; /* Speed output is published by TIM7 and read by FOC. */
} pid_controller_t;

extern pid_controller_t foc_id_pid;
extern pid_controller_t foc_iq_pid;
extern pid_controller_t foc_speed_pid;
/* Keil Watch: [0] Kp, [1] Ki, [2] Kd, [3..5] limits/deadband.
 * [3] is the shared voltage-circle radius: measured VBUS * fixed margin / sqrt(3).
 * [3] and [4] are refreshed from measured VBUS each cycle.
 * Read by the current loop each cycle. Bandwidth changes replace Kp/Ki;
 * manual gain edits remain active until the bandwidth changes again. */
extern volatile float foc_id_pid_parameters[PID_PARAMETER_COUNT];
extern volatile float foc_iq_pid_parameters[PID_PARAMETER_COUNT];
/* Speed PID: Kp, Ki, Kd, current limit (A), integral limit (A), deadband (rad/s).
 * Gains follow current-loop bandwidth / 10, motor inertia and damping ratio. */
extern volatile float foc_speed_pid_parameters[PID_PARAMETER_COUNT];
/* Parameter order: Kp, Ki, Kd, output limit, integral limit, deadband. */
void pid_init(pid_controller_t *pid, volatile float parameters[PID_PARAMETER_COUNT]);

#endif /* USER_MOTOR_PID_H */

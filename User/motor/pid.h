#ifndef USER_MOTOR_PID_H
#define USER_MOTOR_PID_H

#include <stdint.h>

#include "common.h"

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
    volatile float output; /* Published between the position, speed and current loops. */
} pid_controller_t;

extern pid_controller_t foc_id_pid;
extern pid_controller_t foc_iq_pid;
extern pid_controller_t foc_speed_pid;
extern pid_controller_t foc_position_pid;
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
/* Position PID: Kp, Ki, Kd, speed limit (rad/s), integral limit (rad/s),
 * angle-error deadband (rad). Gains use continuous-time units. */
extern volatile float foc_position_pid_parameters[PID_PARAMETER_COUNT];
/* Parameter order: Kp, Ki, Kd, output limit, integral limit, deadband. */
void pid_init(pid_controller_t *pid, volatile float parameters[PID_PARAMETER_COUNT]);
/* Clear controller history and output while retaining gains and limits. */
void pid_reset(pid_controller_t *pid);
/* Clear dq feedforward terms and final voltage references. */
void foc_feedforward_reset(pmsm_t *pm);

void foc_current_pid_init(void);
/* 0: PI only; nonzero: PI + feedforward. Both use circular voltage limiting. */
void foc_current_pid_update(pmsm_t *pm, float dt, uint8_t feedforward_enable);
void foc_speed_pid_init(pmsm_t *pm);
void foc_speed_pid_update(pmsm_t *pm, float dt);
void foc_position_pid_init(pmsm_t *pm);
void foc_position_pid_update(pmsm_t *pm, float dt);

#endif /* USER_MOTOR_PID_H */

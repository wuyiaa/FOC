#include "pid.h"

#include "common.h"

#include <math.h>
#include <stddef.h>

/*
 * Separate editable parameter arrays; order: Kp, Ki, Kd, output limit,
 * integral limit, current-error deadband. PID output/limits are in volts.
 * Gains use fc = fpwm / 20 = 1 kHz, Kp = 2*pi*fc*L,
 * Ki = 2*pi*fc*Rs, and Kd = 0.
 */
static const float foc_id_pid_parameters[PID_PARAMETER_COUNT] =
{//17.8819
    3.1415926*2*1000*0.002846f, 3.1415926*2*1000*6.617f, 0.0f,
    24.0*0.9*0.57735f, 24.0*0.9*0.57735*0.5f, 0.0f
};

static const float foc_iq_pid_parameters[PID_PARAMETER_COUNT] =
{
    3.1415926*2*1000*0.005698f, 3.1415926*2*1000*6.617f, 0.0f,
    24.0*0.9*0.57735f, 24.0*0.9*0.57735*0.5f, 0.0f
};

static pid_controller_t foc_id_pid;
static pid_controller_t foc_iq_pid;

static float pid_clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }

    return value;
}

void pid_init(pid_controller_t *pid, const float parameters[PID_PARAMETER_COUNT])
{
    if ((pid == NULL) || (parameters == NULL))
    {
        return;
    }

    pid->kp = parameters[0];
    pid->ki = parameters[1];
    pid->kd = parameters[2];
    pid->output_limit = fabsf(parameters[3]);
    pid->integral_limit = fabsf(parameters[4]);
    pid->deadband = fabsf(parameters[5]);
    pid->integral_output = 0.0f;
    pid->previous_error = 0.0f;
    pid->output = 0.0f;
}

float pid_calculate(pid_controller_t *pid,
                    float target,
                    float feedback,
                    float dt)
{
    float error;
    float proportional;
    float derivative;
    float integral_candidate;
    float unconstrained_output;
    uint8_t saturation_driven_further;

    if ((pid == NULL) || (dt <= 0.0f))
    {
        return 0.0f;
    }

    error = target - feedback;
    if (fabsf(error) <= pid->deadband)
    {
        error = 0.0f;
    }

    proportional = pid->kp * error;
    derivative = pid->kd * (error - pid->previous_error) / dt;
    integral_candidate = pid_clamp(
        pid->integral_output + (pid->ki * error * dt),
        pid->integral_limit);
    unconstrained_output = proportional + integral_candidate + derivative;

    saturation_driven_further = (uint8_t)(
        ((unconstrained_output > pid->output_limit) && (error > 0.0f)) ||
        ((unconstrained_output < -pid->output_limit) && (error < 0.0f)));
    if (saturation_driven_further == 0U)
    {
        pid->integral_output = integral_candidate;
    }

    pid->previous_error = error;
    pid->output = pid_clamp(
        proportional + pid->integral_output + derivative,
        pid->output_limit);

    return pid->output;
}

void foc_current_pid_init(void)
{
    pid_init(&foc_id_pid, foc_id_pid_parameters);
    pid_init(&foc_iq_pid, foc_iq_pid_parameters);
}

void foc_current_pid_update(pmsm_t *pm, float dt)
{
    if ((pm == NULL) ||
        (pm->encoder_foc_sample.valid == 0U) ||
        (pm->encoder.fault != 0U) ||
        (pm->foc.current_limit_fault != 0U))
    {
        return;
    }

    pm->foc.u_d = pid_calculate(&foc_id_pid,
                                pm->foc.i_d_ref,
                                pm->foc.i_d,
                                dt);
    pm->foc.u_q = pid_calculate(&foc_iq_pid,
                                pm->foc.i_q_ref,
                                pm->foc.i_q,
                                dt);
}

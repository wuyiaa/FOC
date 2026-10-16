#include "pid.h"

#include "common.h"

#include <math.h>
#include <stddef.h>

/*
 * Parameter order: Kp, Ki, Kd, output limit, integral limit, deadband.
 * PID output/limits are in volts. Kp = 2*pi*fc*L, Ki = 2*pi*fc*Rs.
 * Keil Watch can edit the bandwidth and both parameter arrays at runtime.
 * Bandwidth changes recalculate Kp/Ki once; direct edits persist otherwise.
 */
#define FOC_CURRENT_OUTPUT_LIMIT_V \
    (FOC_DC_BUS_VOLTAGE_V * FOC_CURRENT_OUTPUT_UTILIZATION * ONE_BY_SQRT3)

volatile float FOC_CURRENT_LOOP_BANDWIDTH_DEFAULT_HZ = 350.0f;
static float applied_bandwidth_hz = -1.0f;

volatile float foc_id_pid_parameters[PID_PARAMETER_COUNT] =
{
    0.0f, /* Kp is calculated from the live bandwidth during PID initialization. */
    0.0f, /* Ki is calculated at the same time. */
    FOC_CURRENT_KD_V_S_PER_A,
    FOC_CURRENT_OUTPUT_LIMIT_V,
    FOC_CURRENT_OUTPUT_LIMIT_V * FOC_CURRENT_INTEGRAL_LIMIT_RATIO,
    FOC_CURRENT_DEADBAND_A
};

volatile float foc_iq_pid_parameters[PID_PARAMETER_COUNT] =
{
    0.0f, /* Kp is calculated from the live bandwidth during PID initialization. */
    0.0f, /* Ki is calculated at the same time. */
    FOC_CURRENT_KD_V_S_PER_A,
    FOC_CURRENT_OUTPUT_LIMIT_V,
    FOC_CURRENT_OUTPUT_LIMIT_V * FOC_CURRENT_INTEGRAL_LIMIT_RATIO,
    FOC_CURRENT_DEADBAND_A
};

pid_controller_t foc_id_pid;
pid_controller_t foc_iq_pid;

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

static void pid_load_parameters(pid_controller_t *pid,
                                const volatile float parameters[PID_PARAMETER_COUNT])
{
    pid->kp = parameters[0];
    pid->ki = parameters[1];
    pid->kd = parameters[2];
    pid->output_limit = fabsf(parameters[3]);
    pid->integral_limit = fabsf(parameters[4]);
    pid->deadband = fabsf(parameters[5]);
}

static void foc_current_pid_parameters_update(void)
{
    const float bandwidth_hz = FOC_CURRENT_LOOP_BANDWIDTH_DEFAULT_HZ;

    if (isfinite(bandwidth_hz) && (bandwidth_hz >= 0.0f) &&
        (bandwidth_hz != applied_bandwidth_hz))
    {
        const float omega = TWO_PI * bandwidth_hz;

        foc_id_pid_parameters[0] = omega * FOC_MOTOR_LD_H;
        /* Preserve the existing D-axis integral-off / Q-axis integral-on tuning. */
        foc_id_pid_parameters[1] = omega * FOC_MOTOR_RS_OHM * 0.0f;
        foc_iq_pid_parameters[0] = omega * FOC_MOTOR_LQ_H;
        foc_iq_pid_parameters[1] = omega * FOC_MOTOR_RS_OHM;
        applied_bandwidth_hz = bandwidth_hz;
    }

    /* Refresh gains/limits without resetting the controller history. */
    pid_load_parameters(&foc_id_pid, foc_id_pid_parameters);
    pid_load_parameters(&foc_iq_pid, foc_iq_pid_parameters);
}

void pid_init(pid_controller_t *pid, const volatile float parameters[PID_PARAMETER_COUNT])
{
    if ((pid == NULL) || (parameters == NULL))
    {
        return;
    }

    pid_load_parameters(pid, parameters);
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
		pid->p_output = proportional;
    pid->output = pid_clamp(
        proportional + pid->integral_output + derivative,
        pid->output_limit);

    return pid->output;
}

void foc_current_pid_init(void)
{
    pid_init(&foc_id_pid, foc_id_pid_parameters);
    pid_init(&foc_iq_pid, foc_iq_pid_parameters);
    foc_current_pid_parameters_update();
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

    foc_current_pid_parameters_update();

    pm->foc.u_d = pid_calculate(&foc_id_pid,
                                pm->foc.i_d_ref,
                                pm->foc.i_d,
                                dt);
    pm->foc.u_q = pid_calculate(&foc_iq_pid,
                                pm->foc.i_q_ref,
                                pm->foc.i_q,
                                dt);
}

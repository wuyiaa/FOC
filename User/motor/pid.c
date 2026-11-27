#include "pid.h"

#include "common.h"
#include "voltage_sample.h"
#include "encoder_zero_calibration.h"
#include <math.h>
#include <stddef.h>

/*
 * Parameter order: Kp, Ki, Kd, output limit, integral limit, deadband.
 * PID output/limits are in volts. Kp = 2*pi*fc*L, Ki = 2*pi*fc*Rs.
 * Both output limits hold the shared voltage-circle radius from live VBUS.
 * Keil Watch can edit the bandwidth and both parameter arrays at runtime.
 * Bandwidth changes recalculate Kp/Ki once; voltage/integral limits update each cycle.
 */
volatile float FOC_CURRENT_LOOP_BANDWIDTH_DEFAULT_HZ = 300.0f;
static float applied_bandwidth_hz = -1.0f;

pid_controller_t foc_id_pid;
pid_controller_t foc_iq_pid;
pid_controller_t foc_speed_pid;
pid_controller_t foc_position_pid;

ldq_identification_point_t ldq_identification_point_1;

#if 0
#if (Ldq_Identification != 0U)
typedef struct
{
    float current_sin_lpf_a;
    float current_cos_lpf_a;
    float voltage_cos_lpf_v;
    float voltage_sin_lpf_v;
} ldq_hfi_axis_state_t;

static float ldq_hfi_phase_rad;
static ldq_hfi_axis_state_t ldq_hfi_d_state;
static ldq_hfi_axis_state_t ldq_hfi_q_state;
static uint32_t ldq_hfi_settle_sample_count;
static uint32_t ldq_hfi_average_sample_count;
static float ldq_hfi_ld_sum_h;
static float ldq_hfi_lq_sum_h;

static float ldq_hfi_axis_update(ldq_hfi_axis_state_t *state,
                                 float current_feedback_a,
                                 float voltage_axis_v,
                                 float sin_phase,
                                 float cos_phase,
                                 float lpf_alpha,
                                 float omega_h_rad_s)
{
    float current_amplitude_squared_a;

    state->current_sin_lpf_a += lpf_alpha *
        (2.0f * current_feedback_a * sin_phase - state->current_sin_lpf_a);
    state->current_cos_lpf_a += lpf_alpha *
        (2.0f * current_feedback_a * cos_phase - state->current_cos_lpf_a);
    state->voltage_cos_lpf_v += lpf_alpha *
        (2.0f * voltage_axis_v * cos_phase - state->voltage_cos_lpf_v);
    state->voltage_sin_lpf_v += lpf_alpha *
        (2.0f * voltage_axis_v * sin_phase - state->voltage_sin_lpf_v);

    current_amplitude_squared_a =
        state->current_sin_lpf_a * state->current_sin_lpf_a +
        state->current_cos_lpf_a * state->current_cos_lpf_a;
    if (current_amplitude_squared_a > 1.0e-12f)
    {
        return (state->voltage_cos_lpf_v * state->current_sin_lpf_a -
                state->voltage_sin_lpf_v * state->current_cos_lpf_a) /
            (omega_h_rad_s * current_amplitude_squared_a);
    }

    return 0.0f;
}
#endif
#endif

#if (Ldq_Identification != 0U)


static uint32_t ldq_identification_wait_count;
static uint32_t ldq_identification_capture_count;
static float ldq_identification_sum_id_a;
static float ldq_identification_sum_iq_a;
static float ldq_identification_sum_omega_e_rad_s;
static float ldq_identification_sum_ud_v;
static float ldq_identification_sum_uq_v;

static void ldq_identification_stage_reset(void)
{
    ldq_identification_wait_count = 0U;
    ldq_identification_capture_count = 0U;
    ldq_identification_sum_id_a = 0.0f;
    ldq_identification_sum_iq_a = 0.0f;
    ldq_identification_sum_omega_e_rad_s = 0.0f;
    ldq_identification_sum_ud_v = 0.0f;
    ldq_identification_sum_uq_v = 0.0f;
}

static void ldq_identification_capture_point(
    ldq_identification_point_t *point, uint32_t sample_count)
{
    const float inverse_count = 1.0f / (float)sample_count;

    point->id_a = ldq_identification_sum_id_a * inverse_count;
    point->iq_a = ldq_identification_sum_iq_a * inverse_count;
    point->omega_e_rad_s =
        ldq_identification_sum_omega_e_rad_s * inverse_count;
    point->ud_v = ldq_identification_sum_ud_v * inverse_count;
    point->uq_v = ldq_identification_sum_uq_v * inverse_count;
}

static uint8_t ldq_identification_calculate(
    pmsm_t *pm, const ldq_identification_point_t *p1,
    const ldq_identification_point_t *p2)
{
    const float det_rs_lq =
        p1->id_a * (-p2->omega_e_rad_s * p2->iq_a) -
        p2->id_a * (-p1->omega_e_rad_s * p1->iq_a);
    const float det_ld_phif = p1->omega_e_rad_s * p2->omega_e_rad_s *
        (p1->id_a - p2->id_a);
    float rs;
    float lq;
    float q1_residual;
    float q2_residual;

    if ((fabsf(det_rs_lq) <= 1.0e-9f) ||
        (fabsf(det_ld_phif) <= 1.0e-6f))
    {
        return 0U;
    }

    rs = (p1->ud_v * (-p2->omega_e_rad_s * p2->iq_a) -
          p2->ud_v * (-p1->omega_e_rad_s * p1->iq_a)) / det_rs_lq;
    lq = (p1->id_a * p2->ud_v - p2->id_a * p1->ud_v) / det_rs_lq;

    q1_residual = p1->uq_v - rs * p1->iq_a;
    q2_residual = p2->uq_v - rs * p2->iq_a;

    pm->motor.rs_identified_ohm = rs;
    pm->motor.lq_identified_h = lq;
    pm->motor.ld_identified_h =
        (q1_residual * p2->omega_e_rad_s -
         q2_residual * p1->omega_e_rad_s) / det_ld_phif;
    pm->motor.phif_identified_wb =
        (p1->omega_e_rad_s * p1->id_a * q2_residual -
         p2->omega_e_rad_s * p2->id_a * q1_residual) / det_ld_phif;
    return 1U;
}
#endif

volatile float foc_id_pid_parameters[PID_PARAMETER_COUNT] =
{
    0.0f, /* Kp is calculated from the live bandwidth during PID initialization. */
    0.0f, /* Ki is calculated at the same time. */
    0.0f,
    13.2f,//FOC_CURRENT_OUTPUT_LIMIT_V,
    6.1f,//FOC_CURRENT_OUTPUT_LIMIT_V * FOC_CURRENT_INTEGRAL_LIMIT_RATIO,
    FOC_CURRENT_DEADBAND_A
};

volatile float foc_iq_pid_parameters[PID_PARAMETER_COUNT] =
{
    0.0f, /* Kp is calculated from the live bandwidth during PID initialization. */
    0.0f, /* Ki is calculated at the same time. */
    0.0f,
    13.2f,//FOC_CURRENT_OUTPUT_LIMIT_V,
    6.1f,//FOC_CURRENT_OUTPUT_LIMIT_V * FOC_CURRENT_INTEGRAL_LIMIT_RATIO,
    FOC_CURRENT_DEADBAND_A
};

volatile float foc_speed_pid_parameters[PID_PARAMETER_COUNT] =
{
    0.1f, /* 0.015f   0.1f*/
    0.13f, /* 0.13 */
    0.0f,
    5.0f, /* Maximum positive/negative iq reference, A. */
    0.5f, /* Integral limit, A. *///0.5f
    0.0f  /* Speed-error deadband, rad/s. */
};

volatile float foc_position_pid_parameters[PID_PARAMETER_COUNT] =
{
    5.0f,  /* Kp: (rad/s)/rad. */
    0.0f,  /* Ki: dt is applied by pid_axis_calculate. */
    0.0f,  /* Kd: unfiltered angle feedback; disabled initially. */
    70.0f, /* Speed output limit, rad/s. */
    35.0f, /* Integral limit, rad/s. */
    0.0f   /* Angle-error deadband, rad. */
};

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
                                volatile float parameters[PID_PARAMETER_COUNT])
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
    const float limit = voltage_sample.vbus_v * FOC_CURRENT_VOLTAGE_MARGIN * ONE_BY_SQRT3;

    if (isfinite(bandwidth_hz) && (bandwidth_hz >= 0.0f) &&
        (bandwidth_hz != applied_bandwidth_hz))
    {
        const float omega = TWO_PI * bandwidth_hz;

        foc_id_pid_parameters[0] = omega * pm.motor.ld;
        foc_id_pid_parameters[1] = omega * pm.motor.rs;
        foc_iq_pid_parameters[0] = omega * pm.motor.lq;
        foc_iq_pid_parameters[1] = omega * pm.motor.rs;
        applied_bandwidth_hz = bandwidth_hz;
    }

    foc_id_pid_parameters[3] = limit;
    foc_iq_pid_parameters[3] = limit;
    foc_id_pid_parameters[4] = limit * FOC_CURRENT_INTEGRAL_LIMIT_RATIO;
    foc_iq_pid_parameters[4] = limit * FOC_CURRENT_INTEGRAL_LIMIT_RATIO;

    /* Refresh gains/limits without resetting the controller history. */
    pid_load_parameters(&foc_id_pid, foc_id_pid_parameters);
    pid_load_parameters(&foc_iq_pid, foc_iq_pid_parameters);
}

void pid_reset(pid_controller_t *pid)
{
    pid->error = 0.0f;
    pid->integral_output = 0.0f;
    pid->p_output = 0.0f;
    pid->previous_error = 0.0f;
    pid->output = 0.0f;
}

void foc_feedforward_reset(pmsm_t *pm)
{
    pm->foc.Ud_ff = 0.0f;
    pm->foc.Uq_ff = 0.0f;
    pm->foc.Ud_ref = 0.0f;
    pm->foc.Uq_ref = 0.0f;
}

void pid_init(pid_controller_t *pid, volatile float parameters[PID_PARAMETER_COUNT])
{
    if ((pid == NULL) || (parameters == NULL))
    {
        return;
    }

    pid_load_parameters(pid, parameters);
    pid_reset(pid);
}

/* Return the raw single-axis output; the caller applies its output limit. */
static float pid_axis_calculate(pid_controller_t *pid,
                    float target,
                    float feedback,
                    float dt)
{
    float proportional;
    float derivative;

		pid->error = target - feedback;
    if (fabsf(pid->error) <= pid->deadband)
    {
        pid->error = 0.0f;
    }

    proportional = pid->kp * pid->error;
    derivative = pid->kd * (pid->error - pid->previous_error) / dt;
    pid->integral_output = pid_clamp(
        pid->integral_output + (pid->ki * pid->error * dt),
        pid->integral_limit);

    pid->previous_error = pid->error;
    pid->p_output = proportional;
    return proportional + pid->integral_output + derivative;
}

static float foc_current_voltage_scale(float ud, float uq, float limit)
{
    const float magnitude_squared = ud * ud + uq * uq;

    if (limit <= 0.0f)
    {
        return 0.0f;
    }
    if (magnitude_squared > limit * limit)
    {
        return limit / sqrtf(magnitude_squared);
    }
    return 1.0f;
}

/* Called only when the final dq voltage is saturated. */
static void foc_current_anti_windup_update(float ud, float uq,
                                          float old_id_integral,
                                          float old_iq_integral)
{
    const float delta_id_integral = foc_id_pid.integral_output - old_id_integral;
    const float delta_iq_integral = foc_iq_pid.integral_output - old_iq_integral;

    /* Reject outward integral increments; allow increments that unwind. */
    if (ud * delta_id_integral > 0.0f)
    {
        foc_id_pid.integral_output = old_id_integral;
        foc_id_pid.output -= delta_id_integral;
    }
    if (uq * delta_iq_integral > 0.0f)
    {
        foc_iq_pid.integral_output = old_iq_integral;
        foc_iq_pid.output -= delta_iq_integral;
    }
}

static void foc_current_output_limit_update(pmsm_t *pm, float ud, float uq,
                                            float old_id_integral,
                                            float old_iq_integral)
{
    float limit;
    float scale;

    limit = foc_id_pid.output_limit;
    scale = foc_current_voltage_scale(ud, uq, limit);

    if (scale < 1.0f)
    {
        foc_current_anti_windup_update(ud, uq, old_id_integral, old_iq_integral);
        ud = foc_id_pid.output + pm->foc.Ud_ff;
        uq = foc_iq_pid.output + pm->foc.Uq_ff;
        scale = foc_current_voltage_scale(ud, uq, limit);
    }

    pm->foc.Ud_ref = ud * scale;
    pm->foc.Uq_ref = uq * scale;
}

/* One dq step: calculate both axes, then circular-limit the final voltage.
   Ud_ff/Uq_ff are zero in PI-only mode. PID outputs retain the PI terms. */
static void pid_calculate_step(pmsm_t *pm, float dt)
{
    const float old_id_integral = foc_id_pid.integral_output;
    const float old_iq_integral = foc_iq_pid.integral_output;
    float ud;
    float uq;

    foc_id_pid.output = pid_axis_calculate(&foc_id_pid, pm->foc.id_ref, pm->foc.id_fdb, dt);
    foc_iq_pid.output = pid_axis_calculate(&foc_iq_pid, pm->foc.iq_ref, pm->foc.iq_fdb, dt);

    ud = foc_id_pid.output + pm->foc.Ud_ff;
    uq = foc_iq_pid.output + pm->foc.Uq_ff;
    foc_current_output_limit_update(pm, ud, uq, old_id_integral, old_iq_integral);
}

void foc_current_pid_init(void)
{
    pid_init(&foc_id_pid, foc_id_pid_parameters);
    pid_init(&foc_iq_pid, foc_iq_pid_parameters);
    foc_current_pid_parameters_update();
}

void foc_current_pid_update(pmsm_t *pm, float dt, uint8_t feedforward_enable)
{
    float omega_e;
#if 0
#if (Ldq_Identification != 0U)
    float sin_phase;
    float cos_phase;
    float omega_h_rad_s;
    float lpf_alpha;
    float ld_raw_h;
    float lq_raw_h;
#endif
#endif

    if ((pm == NULL) || (dt <= 0.0f))
    {
        return;
    }

    if (foc_pwm_output_is_enabled() == 0U)
    {
        pid_reset(&foc_id_pid);
        pid_reset(&foc_iq_pid);
        foc_feedforward_reset(pm);
        return;
    }

    if ((pm->encoder.fault != 0U) ||
        (pm->foc.current_limit_fault != 0U))
    {
        return;
    }

    foc_current_pid_parameters_update();

#if 0
#if (Ldq_Identification != 0U)
    omega_h_rad_s = TWO_PI * LDQ_HFI_FREQUENCY_HZ;
    sin_phase = sinf(ldq_hfi_phase_rad);
    cos_phase = cosf(ldq_hfi_phase_rad);
    lpf_alpha = TWO_PI * LDQ_HFI_LPF_CUTOFF_HZ * dt;
//    pm->foc.id_ref = LDQ_HFI_CURRENT_AMPLITUDE_A * sin_phase;
//    pm->foc.iq_ref = LDQ_HFI_CURRENT_AMPLITUDE_A * sin_phase;
#endif
#endif

#if (Ldq_Identification != 0U)
    if (pm->motor.ldq_identification_stage == 0U)
    {
        pm->motor.ldq_identification_stage = 1U;
        ldq_identification_stage_reset();
    }

    if (pm->motor.ldq_identification_stage == 1U)
    {
        pm->foc.id_ref = 0.0f;
        pm->foc.iq_ref = 0.15f;//0.15
    }
    else if (pm->motor.ldq_identification_stage >= 2U)
    {
        pm->foc.id_ref = -0.1f;
        pm->foc.iq_ref = 0.2f;
    }
#endif

    if (feedforward_enable != 0U)
    {
        /* Speed uses the physical direction convention; FF must follow theta_e.
           MOTOR_SPEED_DIRECTION is +/-1, so division removes that convention. */
        omega_e = pm->speed.electrical_rad_s *
            (float)calibration.electrical_direction / MOTOR_SPEED_DIRECTION;
        pm->foc.Ud_ff = -omega_e * pm->motor.lq * pm->foc.iq_fdb;
        pm->foc.Uq_ff = omega_e *
            (pm->motor.ld * pm->foc.id_fdb + pm->motor.phif);
    }
    else
    {
        pm->foc.Ud_ff = 0.0f;
        pm->foc.Uq_ff = 0.0f;
    }
    pid_calculate_step(pm, dt);
#if 0
#if (Ldq_Identification != 0U)
    ld_raw_h = ldq_hfi_axis_update(
        &ldq_hfi_d_state, pm->foc.id_fdb, pm->foc.Ud_ref,
        sin_phase, cos_phase, lpf_alpha, omega_h_rad_s);
    lq_raw_h = ldq_hfi_axis_update(
        &ldq_hfi_q_state, pm->foc.iq_fdb, pm->foc.Uq_ref,
        sin_phase, cos_phase, lpf_alpha, omega_h_rad_s);

    if (pm->motor.ldq_identification_average_done == 0U)
    {
        const uint32_t settle_samples =
            (uint32_t)(LDQ_HFI_SETTLE_TIME_S / PWM_PERIOD_S);
        const uint32_t average_samples =
            (uint32_t)(LDQ_HFI_RESULT_AVERAGE_TIME_S / PWM_PERIOD_S);

        if (ldq_hfi_settle_sample_count < settle_samples)
        {
            ldq_hfi_settle_sample_count++;
            pm->motor.ld_identified_h = ld_raw_h;
            pm->motor.lq_identified_h = lq_raw_h;
        }
        else
        {
            ldq_hfi_ld_sum_h += ld_raw_h;
            ldq_hfi_lq_sum_h += lq_raw_h;
            ldq_hfi_average_sample_count++;

            pm->motor.ld_identified_h = ldq_hfi_ld_sum_h /
                (float)ldq_hfi_average_sample_count;
            pm->motor.lq_identified_h = ldq_hfi_lq_sum_h /
                (float)ldq_hfi_average_sample_count;

            if (ldq_hfi_average_sample_count >= average_samples)
            {
                pm->motor.ldq_identification_average_done = 1U;
            }
        }
    }

    ldq_hfi_phase_rad += omega_h_rad_s * dt;
    if (ldq_hfi_phase_rad >= TWO_PI)
    {
        ldq_hfi_phase_rad -= TWO_PI;
    }
#endif
#endif

#if (Ldq_Identification != 0U)
    if (pm->motor.ldq_identification_done == 0U)
    {
        const uint32_t settle_samples =
            (uint32_t)(LDQ_IDENT_SETTLE_TIME_S / dt);
        const uint32_t capture_samples =
            (uint32_t)(LDQ_IDENT_CAPTURE_TIME_S / dt);

        if (ldq_identification_wait_count < settle_samples)
        {
            ldq_identification_wait_count++;
        }
        else
        {
            const float omega_ident_rad_s = pm->speed.electrical_rad_s *
                (float)calibration.electrical_direction /
                MOTOR_SPEED_DIRECTION;

            ldq_identification_sum_id_a += pm->foc.id_fdb;
            ldq_identification_sum_iq_a += pm->foc.iq_fdb;
            ldq_identification_sum_omega_e_rad_s += omega_ident_rad_s;
            ldq_identification_sum_ud_v += pm->foc.Ud_ref;
            ldq_identification_sum_uq_v += pm->foc.Uq_ref;
            ldq_identification_capture_count++;

            if (ldq_identification_capture_count >= capture_samples)
            {
                if (pm->motor.ldq_identification_stage == 1U)
                {
                    ldq_identification_capture_point(
                        &ldq_identification_point_1,
                        ldq_identification_capture_count);
                    pm->motor.ldq_identification_stage = 2U;
                    ldq_identification_stage_reset();
                }
                else
                {
                    ldq_identification_point_t point_2;

                    ldq_identification_capture_point(
                        &point_2, ldq_identification_capture_count);
                    if (ldq_identification_calculate(
                            pm, &ldq_identification_point_1, &point_2) != 0U)
                    {
                        pm->motor.ldq_identification_stage = 3U;
                        pm->motor.ldq_identification_done = 1U;
                    }
                }
            }
        }
    }
#endif
}
//float spd_p;
//float spd_i;
static void foc_speed_pid_parameters_update(const pmsm_t *pm)
{
    const float omega = TWO_PI * FOC_CURRENT_LOOP_BANDWIDTH_DEFAULT_HZ * 0.1f;

    foc_speed_pid_parameters[0] = 2.0f * pm->motor.speed_damping_ratio *
        omega * pm->motor.J / pm->motor.Kt;
    foc_speed_pid_parameters[1] = omega * omega *
        pm->motor.J / pm->motor.Kt;
////	foc_speed_pid_parameters[0] = spd_p;
////	foc_speed_pid_parameters[1] = spd_i;
		foc_speed_pid_parameters[4] = pm->speed.observer_torque_3th.load_torque_nm_hat / pm->motor.Kt * 10.0f;
    pid_load_parameters(&foc_speed_pid, foc_speed_pid_parameters);
}

void foc_speed_pid_init(pmsm_t *pm)
{   
    foc_speed_pid_parameters_update(pm);
    pid_init(&foc_speed_pid, foc_speed_pid_parameters);
}

void foc_speed_pid_update(pmsm_t *pm, float dt)
{
    const float old_integral = foc_speed_pid.integral_output;
    float output;
    float delta_integral;

    if (foc_pwm_output_is_enabled() == 0U)
    {
        pid_reset(&foc_speed_pid);
        return;
    }

    foc_speed_pid_parameters_update(pm);
    output = pid_axis_calculate(&foc_speed_pid,
        pm->speed.mechanical_rad_s_ref,
        pm->speed.mechanical_rad_s_fdb, dt);
    delta_integral = foc_speed_pid.integral_output - old_integral;

    /* Reject integral growth that pushes an already saturated output outward. */
    if ((fabsf(output) > foc_speed_pid.output_limit) &&
        (output * delta_integral > 0.0f))
    {
        foc_speed_pid.integral_output = old_integral;
        output -= delta_integral;
    }

    /* Publish only the limited current target: ADC may preempt TIM7 here. */
    foc_speed_pid.output = pid_clamp(output, foc_speed_pid.output_limit);
}

static void foc_position_pid_parameters_update(void)
{
    pid_load_parameters(&foc_position_pid, foc_position_pid_parameters);
}

void foc_position_pid_init(pmsm_t *pm)
{
    pid_init(&foc_position_pid, foc_position_pid_parameters);
}

void foc_position_pid_update(pmsm_t *pm, float dt)
{
    const float old_integral = foc_position_pid.integral_output;
    float error;
    float output;
    float delta_integral;

	 	pm->pos.mechanical_angle_rad_ref = pm->pos.mechanical_angle_deg_ref * (TWO_PI / 360.0f);

    error = pm->pos.mechanical_angle_rad_ref -
            pm->pos.mechanical_angle_rad_fdb;

    if (foc_pwm_output_is_enabled() == 0U)
    {
        pid_reset(&foc_position_pid);
        return;
    }

    /* Single-turn shortest-path error in [-pi, pi). */
    while (error >= 0.5f * TWO_PI)
    {
        error -= TWO_PI;
    }
    while (error < -0.5f * TWO_PI)
    {
        error += TWO_PI;
    }

    foc_position_pid_parameters_update();
    output = pid_axis_calculate(&foc_position_pid, error, 0.0f, dt);
    delta_integral = foc_position_pid.integral_output - old_integral;

    /* Use the same conditional integration as the speed controller. */
    if ((fabsf(output) > foc_position_pid.output_limit) &&
        (output * delta_integral > 0.0f))
    {
        foc_position_pid.integral_output = old_integral;
        output -= delta_integral;
    }

    /* Output follows the calibrated mechanical-angle direction. */
    foc_position_pid.output = pid_clamp(output, foc_position_pid.output_limit);
}

#include "motor_POS.h"
#include "common.h"
#include "pid.h"

#include <math.h>

void motor_pos_update(void)
{
    /* Use the calibrated mechanical angle directly, without filtering. */
    pm.pos.mechanical_angle_rad_fdb = pm.foc.mechanical_angle_rad;
    pm.pos.mechanical_angle_deg_fdb =
        pm.pos.mechanical_angle_rad_fdb * (360.0f / TWO_PI);

    /* Wrap the degree display only; keep the radian control feedback intact. */
    while (pm.pos.mechanical_angle_deg_fdb < 0.0f)
    {
        pm.pos.mechanical_angle_deg_fdb += 360.0f;
    }
    while (pm.pos.mechanical_angle_deg_fdb >= 360.0f)
    {
        pm.pos.mechanical_angle_deg_fdb -= 360.0f;
    }
}

void motor_pos_mit_reference_update(float dt)
{
    float target_delta;

    if (dt <= 0.0f)
    {
        return;
    }

    pm.pos.mechanical_angle_rad_ref =
        pm.pos.mechanical_angle_deg_ref * (TWO_PI / 360.0f);

    if (foc_pwm_output_is_enabled() == 0U)
    {
        pm.pos.mechanical_angle_rad_ref_previous =
            pm.pos.mechanical_angle_rad_ref;
        pm.pos.mechanical_angle_rad_ref_speed = 0.0f;
        pm.pos.mit_reference_initialized = 0U;
        return;
    }

    if (pm.pos.mit_reference_initialized == 0U)
    {
        pm.pos.mechanical_angle_rad_ref_previous =
            pm.pos.mechanical_angle_rad_ref;
        pm.pos.mechanical_angle_rad_ref_speed = 0.0f;
        pm.pos.mit_reference_initialized = 1U;
        return;
    }

    target_delta = pm.pos.mechanical_angle_rad_ref -
                   pm.pos.mechanical_angle_rad_ref_previous;
    while (target_delta >= 0.5f * TWO_PI)
    {
        target_delta -= TWO_PI;
    }
    while (target_delta < -0.5f * TWO_PI)
    {
        target_delta += TWO_PI;
    }
    pm.pos.mechanical_angle_rad_ref_speed = target_delta / dt;
    pm.pos.mechanical_angle_rad_ref_previous =
        pm.pos.mechanical_angle_rad_ref;
}

   


float Mit_position_control_current_ref(const pmsm_t *pm)
{
    const float h = 12.0f;
    const float tn = 1 / (TWO_PI*FOC_CURRENT_LOOP_BANDWIDTH_DEFAULT_HZ);
    float angle_error;
    float iq_ref;
    float current_limit;
		float kp;//0.1
		float kd;//0.01
		float fdb_gain = 1.0f;
	
    if (pm == NULL)
    {
        return 0.0f;
    }

    angle_error = pm->pos.mechanical_angle_rad_ref -
                  pm->foc.mechanical_angle_rad;
    while (angle_error >= 0.5f * TWO_PI)
    {
        angle_error -= TWO_PI;
    }
    while (angle_error < -0.5f * TWO_PI)
    {
        angle_error += TWO_PI;
    }

    kp = (h + 1.0f) * pm->motor.J /
         (3.0f * pm->motor.phif * (float)pm->motor.pn * h * h * tn * tn);
    kd = h * tn * kp;
    iq_ref = kp * angle_error +
             kd * (pm->pos.mechanical_angle_rad_ref_speed -
                   fdb_gain *
                   pm->speed.observer_torque_3th.mechanical_rad_s_hat) 
            + pm->speed.observer_torque_3th.load_torque_nm_hat / pm->motor.Kt;
             
//pm->speed.observer_torque_3th.mechanical_rad_s_hat
		//pm->speed.mechanical_rad_s_filtered   
		//pm->speed.observer_2nd.mechanical_rad_s_hat
    current_limit = fabsf(foc_speed_pid_parameters[3]);
    if (iq_ref > current_limit)
    {
        iq_ref = current_limit;
    }
    else if (iq_ref < -current_limit)
    {
        iq_ref = -current_limit;
    }

    return iq_ref;
}

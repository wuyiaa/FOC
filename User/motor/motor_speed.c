#include "motor_speed.h"
#include "foc_ctrl.h"
#include "motor_POS.h"

#include <stddef.h>
#include <math.h>

#include "motor_config.h"
#include "common.h"

		 
void motor_speed_reset(motor_speed_t *speed)
{
    static const speed_torque_Luenberger_3th_t empty_torque_3th = {0};
    speed->initialized = 0U;
    speed->last_raw = 0U;
    speed->mechanical_rad_s = 0.0f;
    speed->electrical_rad_s = 0.0f;
    speed->mechanical_rad_s_filtered = 0.0f;
    speed->electrical_rad_s_filtered = 0.0f;
    speed->observer_2nd.mechanical_angle_rad_hat = 0.0f;
    speed->observer_2nd.mechanical_rad_s_hat = 0.0f;
    speed->observer_2nd.electrical_rad_s_hat = 0.0f;
    speed->observer_2nd.angle_error_rad = 0.0f;
    speed->observer_2nd.initialized = 0U;
    speed->observer_3th.mechanical_angle_rad_hat = 0.0f;
    speed->observer_3th.mechanical_rad_s_hat = 0.0f;
    speed->observer_3th.mechanical_rad_s2_hat = 0.0f;
    speed->observer_3th.angle_error_rad = 0.0f;
    speed->observer_3th.initialized = 0U;
    speed->observer_torque_3th = empty_torque_3th;
}
/* Independent second-order observer; no fmodf is needed. */
static float speed_lungberger_2nd_wrap_pi(float angle_rad)
{
    while (angle_rad >= 0.5f * TWO_PI)
    {
        angle_rad -= TWO_PI;
    }
    while (angle_rad < -0.5f * TWO_PI)
    {
        angle_rad += TWO_PI;
    }
    return angle_rad;
}

 float speed_lungberger_2nd_omega_rad_s = 300.0f;
 /* Correction-only Euler observer, matching the MATLAB model term multiplied by 0.
    Both state updates use the same previous angle, speed and wrapped error. */
void speed_lungberger_2nd(speed_lungberger_2nd_t *observer,
                                         float mechanical_angle_rad,
                                         uint8_t pole_pairs)
 {
  //   const float speed_lungberger_2nd_omega_rad_s = TWO_PI * MOTOR_SPEED_LUENBERGER_2ND_CUTOFF_HZ;
     const float l1 = 2.0f * MOTOR_SPEED_LUENBERGER_2ND_DAMPING_RATIO * speed_lungberger_2nd_omega_rad_s -
                      MOTOR_SPEED_LUENBERGER_2ND_B_OVER_J;
     const float l2 = speed_lungberger_2nd_omega_rad_s * speed_lungberger_2nd_omega_rad_s - MOTOR_SPEED_LUENBERGER_2ND_B_OVER_J * l1;
     float angle_error;
     float next_angle;
     float next_speed;

     mechanical_angle_rad = speed_lungberger_2nd_wrap_pi(mechanical_angle_rad);
     if (observer->initialized == 0U)
     {
         observer->mechanical_angle_rad_hat = mechanical_angle_rad;
         observer->mechanical_rad_s_hat = 0.0f;
         observer->electrical_rad_s_hat = 0.0f;
         observer->angle_error_rad = 0.0f;
         observer->initialized = 1U;
         return;
     }

     angle_error = speed_lungberger_2nd_wrap_pi(
         mechanical_angle_rad - observer->mechanical_angle_rad_hat);
     next_angle = observer->mechanical_angle_rad_hat + MOTOR_SPEED_UPDATE_PERIOD_S *
         (observer->mechanical_rad_s_hat + l1 * angle_error);
     next_speed = observer->mechanical_rad_s_hat + MOTOR_SPEED_UPDATE_PERIOD_S *
         l2 * angle_error;
     observer->angle_error_rad = angle_error;
     observer->mechanical_angle_rad_hat = speed_lungberger_2nd_wrap_pi(next_angle);
     observer->mechanical_rad_s_hat = next_speed;
     observer->electrical_rad_s_hat = next_speed * (float)pole_pairs;
 }


/* Local to this observer: no fmodf and no dependency on the second-order state. */
static float speed_Luenberger_3th_wrap_pi(float angle_rad)
{
    while (angle_rad >= 0.5f * TWO_PI)
    {
        angle_rad -= TWO_PI;
    }
    while (angle_rad < -0.5f * TWO_PI)
    {
        angle_rad += TWO_PI;
    }
    return angle_rad;
}
float speed_Luenberger_3th_omega_rad_s = 300.0f;
void speed_Luenberger_3th(speed_Luenberger_3th_t *observer,
                            float mechanical_angle_rad,
                            float dt,
                            float pole_frequency_hz)
{
   // const float speed_Luenberger_3th_omega_rad_s = TWO_PI * pole_frequency_hz;
    const float l1 = 3.0f * speed_Luenberger_3th_omega_rad_s;
    const float l2 = 3.0f * speed_Luenberger_3th_omega_rad_s * speed_Luenberger_3th_omega_rad_s;
    const float l3 = speed_Luenberger_3th_omega_rad_s * speed_Luenberger_3th_omega_rad_s * speed_Luenberger_3th_omega_rad_s;
    float angle_error;
    float next_angle;
    float next_speed;
    float next_acceleration;

    if (observer->initialized == 0U)
    {
        observer->mechanical_angle_rad_hat =
            speed_Luenberger_3th_wrap_pi(mechanical_angle_rad);
        observer->mechanical_rad_s_hat = 0.0f;
        observer->mechanical_rad_s2_hat = 0.0f;
        observer->angle_error_rad = 0.0f;
        observer->initialized = 1U;
        return;
    }

    angle_error = speed_Luenberger_3th_wrap_pi(
        mechanical_angle_rad - observer->mechanical_angle_rad_hat);

    /* Calculate all next states before writing any of the current states. */
    next_angle = observer->mechanical_angle_rad_hat + dt *
        (observer->mechanical_rad_s_hat + l1 * angle_error);
    next_speed = observer->mechanical_rad_s_hat + dt *
        (observer->mechanical_rad_s2_hat + l2 * angle_error);
    next_acceleration = observer->mechanical_rad_s2_hat + dt * l3 * angle_error;

    observer->mechanical_angle_rad_hat =
        speed_Luenberger_3th_wrap_pi(next_angle);
    observer->mechanical_rad_s_hat = next_speed;
    observer->mechanical_rad_s2_hat = next_acceleration;
    observer->angle_error_rad = angle_error;
}

/* Kept independent of both existing observers; no fmodf is needed. */
static float speed_torque_Luenberger_3th_wrap_pi(float angle_rad)
{
    while (angle_rad >= 0.5f * TWO_PI)
    {
        angle_rad -= TWO_PI;
    }
    while (angle_rad < -0.5f * TWO_PI)
    {
        angle_rad += TWO_PI;
    }
    return angle_rad;
}

void speed_torque_Luenberger_3th(
    speed_torque_Luenberger_3th_t *observer,
    float mechanical_angle_rad, float mechanical_rad_s, float iq_fdb,
    float dt, float wc_l, float J, float B, float Kt, uint8_t pole_pairs)
{
    const float l1 = 3.0f * wc_l - B / J;
    const float l2 = 3.0f * wc_l * wc_l - (B / J) * l1;
    const float l3 = -J * wc_l * wc_l * wc_l;
    float angle_error;
    float next_angle;
    float next_speed;
    float load_torque_rate;
    float next_load_torque;

    if (observer->initialized == 0U)
    {
        observer->mechanical_angle_rad_hat =
            speed_torque_Luenberger_3th_wrap_pi(mechanical_angle_rad);
        observer->mechanical_rad_s_hat = 0.0f;
        observer->electrical_rad_s_hat = 0.0f;
        observer->load_torque_nm_hat = 0.0f;
        observer->load_torque_rate_nm_s = 0.0f;
        observer->angle_error_rad = 0.0f;
        observer->speed_error_rad_s = 0.0f;
        observer->initialized = 1U;
        return;
    }

    angle_error = speed_torque_Luenberger_3th_wrap_pi(
        mechanical_angle_rad - observer->mechanical_angle_rad_hat);
    observer->speed_error_rad_s =
        mechanical_rad_s - observer->mechanical_rad_s_hat;

    /* Compute all next states from sample k before publishing sample k+1. */
    next_angle = observer->mechanical_angle_rad_hat + dt *
        (observer->mechanical_rad_s_hat + l1 * angle_error);
    next_speed = observer->mechanical_rad_s_hat + dt *
        ((Kt * iq_fdb - B * observer->mechanical_rad_s_hat -
          observer->load_torque_nm_hat) / J + l2 * angle_error);
    load_torque_rate = l3 * angle_error;
    next_load_torque = observer->load_torque_nm_hat + dt * load_torque_rate;

    observer->mechanical_angle_rad_hat =
        speed_torque_Luenberger_3th_wrap_pi(next_angle);
    observer->mechanical_rad_s_hat = next_speed;
    observer->electrical_rad_s_hat = next_speed * (float)pole_pairs;
    observer->load_torque_nm_hat = next_load_torque;
    observer->load_torque_rate_nm_s = load_torque_rate;
    observer->angle_error_rad = angle_error;
}

void motor_speed_update(motor_speed_t *speed,
                        const as5047p_sample_t *sample,
                        uint8_t pole_pairs)
{
    uint16_t raw_count;
    int32_t delta_count;

    if ((speed == NULL) || (sample == NULL))
    {
        return;
    }


    /* Only this aligned halfword is needed; ADC may publish a newer sample
       during the calculation without changing this local angle. */
    raw_count = sample->raw;

    if (speed->initialized == 0U)
    {
        speed->last_raw = raw_count;
        speed->initialized = 1U;
        return;
    }


    /* The shortest signed count difference handles the 0/16383 wrap. */
    delta_count = (int32_t)raw_count - (int32_t)speed->last_raw;
    if (delta_count > (int32_t)MOTOR_SPEED_HALF_REV)
    {
        delta_count -= (int32_t)MOTOR_SPEED_COUNTS_PER_REV;
    }
    else if (delta_count < -(int32_t)MOTOR_SPEED_HALF_REV)
    {
        delta_count += (int32_t)MOTOR_SPEED_COUNTS_PER_REV;
    }

    /* Physical CCW speed is positive, independent of FOC phase/angle setup. */
		
    speed->mechanical_rad_s = (float)delta_count * MOTOR_SPEED_DIRECTION *
        AS5047P_RAD_PER_COUNT / MOTOR_SPEED_UPDATE_PERIOD_S;
    speed->electrical_rad_s = speed->mechanical_rad_s * (float)pole_pairs;


		

    speed->last_raw = raw_count;
}

void speed_LPF_filtered(motor_speed_t *speed, uint8_t pole_pairs)
{
    speed->mechanical_rad_s_filtered += MOTOR_SPEED_LPF_ALPHA *
        (speed->mechanical_rad_s - speed->mechanical_rad_s_filtered);
    speed->electrical_rad_s_filtered =
        speed->mechanical_rad_s_filtered * (float)pole_pairs;
}

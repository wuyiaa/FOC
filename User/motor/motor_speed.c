#include "motor_speed.h"
#include "foc_ctrl.h"
#include "motor_POS.h"

#include <stddef.h>
#include <math.h>

//#include "dwt_time.h"
#include "motor_config.h"
#include "common.h"

		 
void motor_speed_reset(motor_speed_t *speed)
{
    speed->initialized = 0U;
    speed->last_raw = 0U;
    speed->mechanical_rad_s = 0.0f;
    speed->electrical_rad_s = 0.0f;
    speed->mechanical_rad_s_filtered = 0.0f;
    speed->electrical_rad_s_filtered = 0.0f;
    speed->mechanical_angle_rad_hat = 0.0f;
    speed->mechanical_rad_s_hat = 0.0f;
    speed->electrical_rad_s_hat = 0.0f;
    speed->observer_angle_error_rad = 0.0f;
    speed->observer_three.mechanical_angle_rad_hat = 0.0f;
    speed->observer_three.mechanical_rad_s_hat = 0.0f;
    speed->observer_three.mechanical_rad_s2_hat = 0.0f;
    speed->observer_three.angle_error_rad = 0.0f;
    speed->observer_three.initialized = 0U;
}
/* Second-order implementation retained for later use; currently disabled. */
 float omega = 300.0f;
 /* Correction-only Euler observer, matching the MATLAB model term multiplied by 0.
    Both state updates use the same previous angle, speed and wrapped error. */
void speed_lungberger_two(motor_speed_t *speed,
                                         float mechanical_angle_rad,
                                         uint8_t pole_pairs)
 {
  //   const float omega = TWO_PI * MOTOR_SPEED_OBSERVER_CUTOFF_HZ;
     const float l1 = 2.0f * MOTOR_SPEED_OBSERVER_DAMPING_RATIO * omega -
                      MOTOR_SPEED_OBSERVER_B_OVER_J;
     const float l2 = omega * omega - MOTOR_SPEED_OBSERVER_B_OVER_J * l1;
     float angle_error = mechanical_angle_rad - speed->mechanical_angle_rad_hat;
     float next_angle;
     float next_speed;

     if (angle_error > 0.5f * TWO_PI)
     {
         angle_error -= TWO_PI;
     }
     else if (angle_error < -0.5f * TWO_PI)
     {
         angle_error += TWO_PI;
     }

     next_angle = speed->mechanical_angle_rad_hat + MOTOR_SPEED_UPDATE_PERIOD_S *
         (speed->mechanical_rad_s_hat + l1 * angle_error);
     next_speed = speed->mechanical_rad_s_hat + MOTOR_SPEED_UPDATE_PERIOD_S *
         l2 * angle_error;
     while (next_angle >= TWO_PI)
     {
         next_angle -= TWO_PI;
     }
     while (next_angle < 0.0f)
     {
         next_angle += TWO_PI;
     }

     speed->observer_angle_error_rad = angle_error;
     speed->mechanical_angle_rad_hat = next_angle;
     speed->mechanical_rad_s_hat = next_speed;
     speed->electrical_rad_s_hat = next_speed * (float)pole_pairs;
 }


/* Local to this observer: no fmodf and no dependency on the second-order state. */
static float speed_Luenberger_three_wrap_pi(float angle_rad)
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
float omega_l = 300.0f;
void speed_Luenberger_three(speed_Luenberger_three_t *observer,
                            float mechanical_angle_rad,
                            float dt,
                            float pole_frequency_hz)
{
   // const float omega_l = TWO_PI * pole_frequency_hz;
    const float l1 = 3.0f * omega_l;
    const float l2 = 3.0f * omega_l * omega_l;
    const float l3 = omega_l * omega_l * omega_l;
    float angle_error;
    float next_angle;
    float next_speed;
    float next_acceleration;

    if (observer->initialized == 0U)
    {
        observer->mechanical_angle_rad_hat =
            speed_Luenberger_three_wrap_pi(mechanical_angle_rad);
        observer->mechanical_rad_s_hat = 0.0f;
        observer->mechanical_rad_s2_hat = 0.0f;
        observer->angle_error_rad = 0.0f;
        observer->initialized = 1U;
        return;
    }

    angle_error = speed_Luenberger_three_wrap_pi(
        mechanical_angle_rad - observer->mechanical_angle_rad_hat);

    /* Calculate all next states before writing any of the current states. */
    next_angle = observer->mechanical_angle_rad_hat + dt *
        (observer->mechanical_rad_s_hat + l1 * angle_error);
    next_speed = observer->mechanical_rad_s_hat + dt *
        (observer->mechanical_rad_s2_hat + l2 * angle_error);
    next_acceleration = observer->mechanical_rad_s2_hat + dt * l3 * angle_error;

    observer->mechanical_angle_rad_hat =
        speed_Luenberger_three_wrap_pi(next_angle);
    observer->mechanical_rad_s_hat = next_speed;
    observer->mechanical_rad_s2_hat = next_acceleration;
    observer->angle_error_rad = angle_error;
}

void motor_speed_update(motor_speed_t *speed,
                        const as5047p_sample_t *sample,
                        uint8_t pole_pairs)
{
    //uint32_t elapsed_cycles;
    uint16_t raw_count;
    int32_t delta_count;
    float mechanical_angle_rad;

    if ((speed == NULL) || (sample == NULL))
    {
        return;
    }

    //if ((speed->initialized != 0U) &&
    //    ((uint32_t)(now_cycle - speed->last_sample_cycle) >=
    //     MOTOR_SPEED_STALE_CYCLES))
    //{
    //    speed->mechanical_rad_s = 0.0f;
    //    speed->electrical_rad_s = 0.0f;
    //    speed->initialized = 0U;
    //    if (sample->sequence == speed->last_sequence)
    //    {
    //        return;
    //    }
    //}
    //if (sample->valid == 0U)
    //{
    //    return;
    //}

    /* Only this aligned halfword is needed; ADC may publish a newer sample
       during the calculation without changing this local angle. */
    raw_count = sample->raw;
    mechanical_angle_rad = (float)raw_count * AS5047P_RAD_PER_COUNT *
        MOTOR_SPEED_DIRECTION;
    if (mechanical_angle_rad < 0.0f)
    {
        mechanical_angle_rad += TWO_PI;
    }

    if (speed->initialized == 0U)
    {
        speed->last_raw = raw_count;
        speed->mechanical_angle_rad_hat = mechanical_angle_rad;
        speed->mechanical_rad_s_hat = 0.0f;
        speed->electrical_rad_s_hat = 0.0f;
        speed->observer_angle_error_rad = 0.0f;
        //speed->last_sample_cycle = sample->cycle;
        //speed->last_sequence = sample->sequence;
        speed->initialized = 1U;
        return;
    }

    //if (sample->sequence == speed->last_sequence)
    //{
    //    return;
    //}
    //elapsed_cycles = (uint32_t)(sample->cycle - speed->last_sample_cycle);
    //if (elapsed_cycles < MOTOR_SPEED_WINDOW_CYCLES)
    //{
    //    return;
    //}

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
    //speed->mechanical_rad_s =
    //    ((float)delta_count * MOTOR_SPEED_DIRECTION * AS5047P_RAD_PER_COUNT *
    //     (float)DWT_CPU_FREQ_HZ) / (float)elapsed_cycles;
		
    speed->mechanical_rad_s = (float)delta_count * MOTOR_SPEED_DIRECTION *
        AS5047P_RAD_PER_COUNT / MOTOR_SPEED_UPDATE_PERIOD_S;
    speed->electrical_rad_s = speed->mechanical_rad_s * (float)pole_pairs;

    speed->mechanical_rad_s_filtered += MOTOR_SPEED_FILTER_ALPHA *
        (speed->mechanical_rad_s - speed->mechanical_rad_s_filtered);
		speed->electrical_rad_s_filtered =  speed->mechanical_rad_s_filtered * (float)pole_pairs;
		

    speed->last_raw = raw_count;
    //speed->last_sample_cycle = sample->cycle;
    //speed->last_sequence = sample->sequence;
}

/* Mechanical parameter identification, independent of speed feedback updates. */
motor_identification_t motor_identification;

static void motor_ident_window_begin(motor_identification_window_t *window,
                                    float speed_rad_s, float iq_a)
{
    static const motor_identification_window_t empty = {0};
    *window = empty;
    window->speed_start_rad_s = speed_rad_s;
    window->speed_end_rad_s = speed_rad_s;
    window->last_iq_a = iq_a;
    window->started = 1U;
}

static void motor_ident_window_append(motor_identification_window_t *window,
                                     float speed_rad_s, float iq_a)
{
    window->speed_sum_rad_s += 0.5f * (window->speed_end_rad_s + speed_rad_s);
    window->iq_sum_a += 0.5f * (window->last_iq_a + iq_a);
    window->speed_end_rad_s = speed_rad_s;
    window->last_iq_a = iq_a;
    window->interval_count++;
}

static void motor_ident_window_finish(motor_identification_window_t *window)
{
    const float intervals = (float)window->interval_count;
    window->duration_s = intervals * MOTOR_SPEED_UPDATE_PERIOD_S;
    window->speed_mean_rad_s = window->speed_sum_rad_s / intervals;
    window->iq_mean_a = window->iq_sum_a / intervals;
    window->acceleration_mean_rad_s2 =
        (window->speed_end_rad_s - window->speed_start_rad_s) / window->duration_s;
}

static uint8_t motor_ident_settled(float speed_rad_s, float target_rad_s)
{
    if (fabsf(speed_rad_s - target_rad_s) <= MOTOR_IDENT_SPEED_TOLERANCE_RAD_S)
    {
        motor_identification.stable_count++;
    }
    else
    {
        motor_identification.stable_count = 0U;
    }
    if (motor_identification.stable_count >= MOTOR_IDENT_SETTLE_TICKS)
    {
        motor_identification.stable_count = 0U;
        return 1U;
    }
    return 0U;
}

static void motor_ident_results_calculate(void)
{
    const float kt = motor_identification.torque_constant_nm_a;
    const float steady_speed_difference =
        motor_identification.steady_high.speed_mean_rad_s -
        motor_identification.steady_low.speed_mean_rad_s;
    const float acceleration_difference =
        motor_identification.acceleration.acceleration_mean_rad_s2 -
        motor_identification.deceleration.acceleration_mean_rad_s2;
    const float dynamic_speed_difference =
        motor_identification.acceleration.speed_mean_rad_s -
        motor_identification.deceleration.speed_mean_rad_s;
    const float dynamic_torque_difference = kt *
        (motor_identification.acceleration.iq_mean_a -
         motor_identification.deceleration.iq_mean_a);

    /* Only reject undefined equations; keep signed/noisy results observable. */
    if (!(steady_speed_difference > 0.001f) ||
        !(motor_identification.acceleration.acceleration_mean_rad_s2 > 0.0f) ||
        !(motor_identification.deceleration.acceleration_mean_rad_s2 < 0.0f))
    {
        motor_identification.stage = MOTOR_IDENT_FAILED;
        return;
    }
    motor_identification.B = kt *
        (motor_identification.steady_high.iq_mean_a -
         motor_identification.steady_low.iq_mean_a) / steady_speed_difference;
    motor_identification.Tf = kt * motor_identification.steady_low.iq_mean_a -
        motor_identification.B * motor_identification.steady_low.speed_mean_rad_s;
    /* Same positive direction cancels Coulomb friction, but not B * speed. */
    motor_identification.J = (dynamic_torque_difference -
        motor_identification.B * dynamic_speed_difference) / acceleration_difference;

    if (isfinite(motor_identification.J) && isfinite(motor_identification.B) &&
        isfinite(motor_identification.Tf))
    {
        motor_identification.result_ready = 1U;
        motor_identification.stage = MOTOR_IDENT_DONE;
    }
    else
    {
        motor_identification.stage = MOTOR_IDENT_FAILED;
    }
}

void J_B_Tf_Indent(motor_speed_t *speed, float iq_fdb,
                  float torque_constant_nm_a, uint8_t output_enabled)
{
    const float speed_rad_s = speed->mechanical_rad_s_fdb;
    motor_identification_window_t *window;

    if (output_enabled == 0U)
    {
        /* Retain completed observations while paused; discard partial progress. */
        if ((motor_identification.stage != MOTOR_IDENT_DONE) &&
            (motor_identification.stage != MOTOR_IDENT_FAILED))
        {
            motor_identification.stage = MOTOR_IDENT_IDLE;
            motor_identification.stable_count = 0U;
        }
        speed->mechanical_rad_s_ref = 0.0f;
        return;
    }
    if ((motor_identification.stage == MOTOR_IDENT_IDLE) ||
        (motor_identification.stage == MOTOR_IDENT_DONE) ||
        (motor_identification.stage == MOTOR_IDENT_FAILED))
    {
        static const motor_identification_t empty = {MOTOR_IDENT_IDLE};
        motor_identification = empty;
        motor_identification.torque_constant_nm_a = torque_constant_nm_a;
        motor_identification.stage = (torque_constant_nm_a > 0.0f) ?
            MOTOR_IDENT_SETTLE_INITIAL : MOTOR_IDENT_FAILED;
    }

    switch (motor_identification.stage)
    {
        case MOTOR_IDENT_SETTLE_INITIAL:
            if (motor_ident_settled(speed_rad_s, MOTOR_IDENT_LOW_SPEED_RAD_S) != 0U)
            {
                motor_identification.stage = MOTOR_IDENT_ACCELERATE;
            }
            break;

        case MOTOR_IDENT_ACCELERATE:
            window = &motor_identification.acceleration;
            if (window->started != 0U)
            {
                motor_ident_window_append(window, speed_rad_s, iq_fdb);
            }
            else if ((speed_rad_s >= MOTOR_IDENT_WINDOW_LOW_RAD_S) &&
                     (speed_rad_s < MOTOR_IDENT_WINDOW_HIGH_RAD_S))
            {
                motor_ident_window_begin(window, speed_rad_s, iq_fdb);
            }
            if (speed_rad_s >= MOTOR_IDENT_WINDOW_HIGH_RAD_S)
            {
                if (window->interval_count != 0U)
                {
                    motor_ident_window_finish(window);
                    motor_identification.stage = MOTOR_IDENT_SETTLE_HIGH;
                }
                else
                {
                    motor_identification.stage = MOTOR_IDENT_FAILED;
                }
            }
            break;

        case MOTOR_IDENT_SETTLE_HIGH:
            if (motor_ident_settled(speed_rad_s, MOTOR_IDENT_HIGH_SPEED_RAD_S) != 0U)
            {
                motor_ident_window_begin(&motor_identification.steady_high,
                                        speed_rad_s, iq_fdb);
                motor_identification.stage = MOTOR_IDENT_AVERAGE_HIGH;
            }
            break;

        case MOTOR_IDENT_AVERAGE_HIGH:
            if (fabsf(speed_rad_s - MOTOR_IDENT_HIGH_SPEED_RAD_S) >
                MOTOR_IDENT_SPEED_TOLERANCE_RAD_S)
            {
                motor_identification.stage = MOTOR_IDENT_SETTLE_HIGH;
                break;
            }
            window = &motor_identification.steady_high;
            motor_ident_window_append(window, speed_rad_s, iq_fdb);
            if (window->interval_count >= MOTOR_IDENT_AVERAGE_TICKS)
            {
                motor_ident_window_finish(window);
                motor_identification.stage = MOTOR_IDENT_DECELERATE;
            }
            break;

        case MOTOR_IDENT_DECELERATE:
            window = &motor_identification.deceleration;
            if (window->started != 0U)
            {
                motor_ident_window_append(window, speed_rad_s, iq_fdb);
            }
            else if ((speed_rad_s <= MOTOR_IDENT_WINDOW_HIGH_RAD_S) &&
                     (speed_rad_s > MOTOR_IDENT_WINDOW_LOW_RAD_S))
            {
                motor_ident_window_begin(window, speed_rad_s, iq_fdb);
            }
            if (speed_rad_s <= MOTOR_IDENT_WINDOW_LOW_RAD_S)
            {
                if (window->interval_count != 0U)
                {
                    motor_ident_window_finish(window);
                    motor_identification.stage = MOTOR_IDENT_SETTLE_LOW;
                }
                else
                {
                    motor_identification.stage = MOTOR_IDENT_FAILED;
                }
            }
            break;

        case MOTOR_IDENT_SETTLE_LOW:
            if (motor_ident_settled(speed_rad_s, MOTOR_IDENT_LOW_SPEED_RAD_S) != 0U)
            {
                motor_ident_window_begin(&motor_identification.steady_low,
                                        speed_rad_s, iq_fdb);
                motor_identification.stage = MOTOR_IDENT_AVERAGE_LOW;
            }
            break;

        case MOTOR_IDENT_AVERAGE_LOW:
            if (fabsf(speed_rad_s - MOTOR_IDENT_LOW_SPEED_RAD_S) >
                MOTOR_IDENT_SPEED_TOLERANCE_RAD_S)
            {
                motor_identification.stage = MOTOR_IDENT_SETTLE_LOW;
                break;
            }
            window = &motor_identification.steady_low;
            motor_ident_window_append(window, speed_rad_s, iq_fdb);
            if (window->interval_count >= MOTOR_IDENT_AVERAGE_TICKS)
            {
                motor_ident_window_finish(window);
                motor_ident_results_calculate();
            }
            break;

        default: /* DONE/FAILED: retain observations until a new run. */
            break;
    }

    if ((motor_identification.stage == MOTOR_IDENT_DONE) ||
        (motor_identification.stage == MOTOR_IDENT_FAILED))
    {
        speed->mechanical_rad_s_ref = 0.0f;
    }
    else if ((motor_identification.stage == MOTOR_IDENT_ACCELERATE) ||
        (motor_identification.stage == MOTOR_IDENT_SETTLE_HIGH) ||
        (motor_identification.stage == MOTOR_IDENT_AVERAGE_HIGH))
    {
        speed->mechanical_rad_s_ref = MOTOR_IDENT_HIGH_SPEED_RAD_S;
    }
    else
    {
        speed->mechanical_rad_s_ref = MOTOR_IDENT_LOW_SPEED_RAD_S;
    }
}

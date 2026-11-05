#ifndef USER_MOTOR_MOTOR_SPEED_H
#define USER_MOTOR_MOTOR_SPEED_H

#include <stdint.h>

#include "as5047p.h"

//#define MOTOR_SPEED_WINDOW_CYCLES  (DWT_CPU_FREQ_HZ / 1000UL)
//#define MOTOR_SPEED_STALE_CYCLES   (DWT_CPU_FREQ_HZ / 100UL)
#define MOTOR_SPEED_UPDATE_PERIOD_S  0.0005f
#define MOTOR_SPEED_FILTER_CUTOFF_HZ 100.0f

/* Parallel Luenberger observer: fc_L, thegma_L and B/J. */
#define MOTOR_SPEED_OBSERVER_CUTOFF_HZ     100.0f
#define MOTOR_SPEED_OBSERVER_DAMPING_RATIO   1.0f
#define MOTOR_SPEED_OBSERVER_B_OVER_J        0.0f

/* Three repeated continuous poles at -2*pi*fc; this is not -3 dB bandwidth. */
#define MOTOR_SPEED_LUENBERGER_THREE_POLE_HZ 100.0f
#define MOTOR_SPEED_COUNTS_PER_REV (AS5047P_DATA_MASK + 1U)
#define MOTOR_SPEED_HALF_REV       (MOTOR_SPEED_COUNTS_PER_REV / 2U)
#define MOTOR_SPEED_FILTER_ALPHA \
    ((TWO_PI * MOTOR_SPEED_FILTER_CUTOFF_HZ * MOTOR_SPEED_UPDATE_PERIOD_S) / \
     (1.0f + TWO_PI * MOTOR_SPEED_FILTER_CUTOFF_HZ * MOTOR_SPEED_UPDATE_PERIOD_S))

/* Independent third-order observer states, all in mechanical units. */
typedef struct
{
    float mechanical_angle_rad_hat;
    float mechanical_rad_s_hat;
    float mechanical_rad_s2_hat;
    float angle_error_rad;
    uint8_t initialized;
} speed_Luenberger_three_t;

typedef struct
{
    volatile float mechanical_rad_s_ref; /* Speed target, rad/s. */
		volatile float mechanical_rad_s_fdb; /* Speed target, rad/s. */
    float mechanical_rad_s;
    float mechanical_rad_s_filtered; /* First-order low-pass output, rad/s. */
    float electrical_rad_s;
	  float electrical_rad_s_filtered;
    /* Observer uses mechanical angle/speed in MOTOR_SPEED_DIRECTION. */
    float mechanical_angle_rad_hat;
    float mechanical_rad_s_hat;
    float electrical_rad_s_hat;
    float observer_angle_error_rad;
    speed_Luenberger_three_t observer_three;
    uint16_t last_raw;
    //uint32_t last_sample_cycle;
    //uint32_t last_sequence;
    uint8_t initialized;
} motor_speed_t;

/* Mechanical angle/error use [-pi, pi). dt and pole_frequency_hz must be > 0.
   First sample aligns the angle and initializes speed/acceleration to zero. */
void speed_Luenberger_three(speed_Luenberger_three_t *observer,
                            float mechanical_angle_rad,
                            float dt,
                            float pole_frequency_hz);

/* Clear feedback/history; preserve mechanical_rad_s_ref. */
void motor_speed_reset(motor_speed_t *speed);

/* TIM7: difference/LPF feedback every 500 us, also while paused. */
void motor_speed_update(motor_speed_t *speed,
                        const as5047p_sample_t *sample,
                        uint8_t pole_pairs);


/* Positive, unloaded 25 -> 50 -> 25 rad/s identification. */
#define MOTOR_IDENT_LOW_SPEED_RAD_S          25.0f
#define MOTOR_IDENT_HIGH_SPEED_RAD_S         50.0f
#define MOTOR_IDENT_WINDOW_LOW_RAD_S         30.0f
#define MOTOR_IDENT_WINDOW_HIGH_RAD_S        45.0f
#define MOTOR_IDENT_SPEED_TOLERANCE_RAD_S      1.0f
#define MOTOR_IDENT_SETTLE_TIME_S              1.0f
#define MOTOR_IDENT_AVERAGE_TIME_S             1.0f
#define MOTOR_IDENT_SETTLE_TICKS \
    ((uint32_t)(MOTOR_IDENT_SETTLE_TIME_S / MOTOR_SPEED_UPDATE_PERIOD_S + 0.5f))
#define MOTOR_IDENT_AVERAGE_TICKS \
    ((uint32_t)(MOTOR_IDENT_AVERAGE_TIME_S / MOTOR_SPEED_UPDATE_PERIOD_S + 0.5f))

typedef enum
{
    MOTOR_IDENT_IDLE = 0,
    MOTOR_IDENT_SETTLE_INITIAL,
    MOTOR_IDENT_ACCELERATE,
    MOTOR_IDENT_SETTLE_HIGH,
    MOTOR_IDENT_AVERAGE_HIGH,
    MOTOR_IDENT_DECELERATE,
    MOTOR_IDENT_SETTLE_LOW,
    MOTOR_IDENT_AVERAGE_LOW,
    MOTOR_IDENT_DONE,
    MOTOR_IDENT_FAILED
} motor_identification_stage_t;

typedef struct
{
    float speed_start_rad_s;
    float speed_end_rad_s;
    float speed_mean_rad_s;
    float iq_mean_a;
    float acceleration_mean_rad_s2; /* Signed endpoint difference / duration. */
    float duration_s;
    /* Trapezoid sums over intervals: both means cover the same time window. */
    float speed_sum_rad_s;
    float iq_sum_a;
    float last_iq_a;
    uint32_t interval_count;
    uint8_t started;
} motor_identification_window_t;

typedef struct
{
    volatile motor_identification_stage_t stage;
    volatile uint8_t result_ready;
    float J;  /* kg*m^2; observation only, includes attached inertia. */
    float B;  /* N*m*s/rad. */
    float Tf; /* Sliding/Coulomb friction torque in N*m, positive rotation. */
    float torque_constant_nm_a;
    uint32_t stable_count;
    motor_identification_window_t acceleration;
    motor_identification_window_t deceleration;
    motor_identification_window_t steady_high;
    motor_identification_window_t steady_low;
} motor_identification_t;

extern motor_identification_t motor_identification;

/* Call every MOTOR_SPEED_UPDATE_PERIOD_S after feedback, before speed PI.
   Uses mechanical_rad_s_fdb and measured iq. Kt = 1.5 * pole_pairs * flux.
   Starts automatically with PWM on; PWM off cancels an unfinished run.
   Caller pauses/disables MOS at DONE/FAILED. A new motor start begins a new run.
   Does not update motor J/B/static friction, or depend on acceleration observers. */
void J_B_Tf_Indent(motor_speed_t *speed, float iq_fdb,
                  float torque_constant_nm_a, uint8_t output_enabled);

void speed_lungberger_two(motor_speed_t *speed,
                                         float mechanical_angle_rad,
                                         uint8_t pole_pairs);

#endif /* USER_MOTOR_MOTOR_SPEED_H */

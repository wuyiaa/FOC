#ifndef USER_MOTOR_B_J_TF_IDENTIFICATION_H
#define USER_MOTOR_B_J_TF_IDENTIFICATION_H

#include "common.h"

/* Positive, unloaded identification: steady B/Tf first, then ramp-based J. */
#define MOTOR_IDENT_LOW_SPEED_RAD_S          25.0f
#define MOTOR_IDENT_HIGH_SPEED_RAD_S         50.0f
#define MOTOR_IDENT_ACCELERATION_RAD_S2      2500.0f
#define MOTOR_IDENT_DECELERATION_RAD_S2     -2500.0f
#define MOTOR_IDENT_WINDOW_LOW_RAD_S         30.0f
#define MOTOR_IDENT_WINDOW_HIGH_RAD_S        45.0f
#define MOTOR_IDENT_SPEED_TOLERANCE_RAD_S      2.0f
#define MOTOR_IDENT_SETTLE_TIME_S              1.0f
#define MOTOR_IDENT_AVERAGE_TIME_S             1.0f
#define MOTOR_IDENT_SETTLE_TICKS \
    ((uint32_t)(MOTOR_IDENT_SETTLE_TIME_S / MOTOR_SPEED_UPDATE_PERIOD_S + 0.5f))
#define MOTOR_IDENT_AVERAGE_TICKS \
    ((uint32_t)(MOTOR_IDENT_AVERAGE_TIME_S / MOTOR_SPEED_UPDATE_PERIOD_S + 0.5f))

typedef enum
{
    MOTOR_IDENT_IDLE = 0,
    MOTOR_IDENT_SETTLE_INITIAL,          /* 1: settle at 25 rad/s for B/Tf. */
    MOTOR_IDENT_AVERAGE_LOW,             /* 2: average at 25 rad/s. */
    MOTOR_IDENT_SETTLE_HIGH,             /* 3: settle at 50 rad/s for B/Tf. */
    MOTOR_IDENT_AVERAGE_HIGH,            /* 4: average at 50, calculate B/Tf. */
    MOTOR_IDENT_RETURN_LOW,              /* 5: return/settle at 25 for J. */
    MOTOR_IDENT_ACCELERATE,              /* 6: +25 rad/s^2; capture 30 -> 45. */
    MOTOR_IDENT_SETTLE_BEFORE_DECELERATE, /* 7: settle at 50 before ramp down. */
    MOTOR_IDENT_DECELERATE,              /* 8: -25 rad/s^2; capture 45 -> 30. */
    MOTOR_IDENT_DONE,                    /* 9: complete; caller disables PWM. */
    MOTOR_IDENT_FAILED                  /* 10: invalid measurement. */
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
    uint8_t completed;
} motor_identification_window_t;

typedef struct
{
    volatile motor_identification_stage_t stage;
    volatile uint8_t result_ready;
    volatile uint8_t flash_loaded;
    volatile uint8_t flash_save_attempted;
    volatile uint8_t flash_saved;
    float J;  /* kg*m^2; includes attached inertia. */
    float B;  /* N*m*s/rad. */
    float Tf; /* Sliding/Coulomb friction torque in N*m, positive rotation. */
    float torque_constant_nm_a;
    uint32_t stable_count;
    motor_identification_window_t acceleration;
    motor_identification_window_t deceleration;
    motor_identification_window_t steady_high;
    motor_identification_window_t steady_low;
} motor_identification_t;

#define MOTOR_PARAMETERS_RECORD_WORDS 4U

/* J/B/Tf persistence in the shared calibration page. */
HAL_StatusTypeDef motor_parameters_flash_load(pmsm_motor_t *motor);
HAL_StatusTypeDef motor_parameters_flash_save(const pmsm_motor_t *motor);
/* Used by flash_save to retain the latest complete J/B/Tf record. */
uint8_t motor_parameters_flash_record_valid(const volatile uint64_t *record);

extern motor_identification_t motor_identification;
/* With identification compiled in, set to 1 to request a new run. */
extern volatile uint8_t motor_identification_requested;
/* TIM7 completion: apply, save and verify once while PWM is off. */
void motor_identification_save_process(void);

/* Call every MOTOR_SPEED_UPDATE_PERIOD_S after feedback, before speed PI.
   Uses mechanical_rad_s_fdb, measured iq and initialized motor Kt.
   First measure B/Tf at 25/50 rad/s, then return to 25 for J ramp windows.
   Reference acceleration is commanded; actual acceleration is endpoint delta/dt.
   Starts automatically with PWM on; PWM off cancels an unfinished run.
   Caller pauses/disables MOS at DONE/FAILED and clears the request.
   The TIM7 caller applies J/B/Tf and persists them after completion.
   Does not change static friction or depend on acceleration observers. */
void J_B_Tf_Indent(motor_speed_t *speed, float iq_fdb,
                  float torque_constant_nm_a, uint8_t output_enabled);

#endif /* USER_MOTOR_B_J_TF_IDENTIFICATION_H */

#ifndef USER_MOTOR_MOTOR_SPEED_H
#define USER_MOTOR_MOTOR_SPEED_H

#include <stdint.h>

#include "as5047p.h"

#define MOTOR_SPEED_UPDATE_PERIOD_S  0.0005f
#define MOTOR_SPEED_LPF_CUTOFF_HZ 100.0f

/* Parallel Luenberger observer: fc_L, thegma_L and B/J. */
#define MOTOR_SPEED_LUENBERGER_2ND_CUTOFF_HZ     100.0f
#define MOTOR_SPEED_LUENBERGER_2ND_DAMPING_RATIO   1.0f
#define MOTOR_SPEED_LUENBERGER_2ND_B_OVER_J        0.0f

/* Three repeated continuous poles at -2*pi*fc; this is not -3 dB bandwidth. */
#define MOTOR_SPEED_LUENBERGER_3TH_POLE_HZ 100.0f
/* Angular pole frequency wc_l in rad/s, not Hz. */
#define MOTOR_SPEED_TORQUE_LUENBERGER_3TH_POLE_RAD_S 50.0f
#define MOTOR_SPEED_COUNTS_PER_REV (AS5047P_DATA_MASK + 1U)
#define MOTOR_SPEED_HALF_REV       (MOTOR_SPEED_COUNTS_PER_REV / 2U)
#define MOTOR_SPEED_LPF_ALPHA \
    ((TWO_PI * MOTOR_SPEED_LPF_CUTOFF_HZ * MOTOR_SPEED_UPDATE_PERIOD_S) / \
     (1.0f + TWO_PI * MOTOR_SPEED_LPF_CUTOFF_HZ * MOTOR_SPEED_UPDATE_PERIOD_S))

/* Independent second-order observer states, in MOTOR_SPEED_DIRECTION. */
typedef struct
{
    float mechanical_angle_rad_hat;
    float mechanical_rad_s_hat;
    float electrical_rad_s_hat;
    float angle_error_rad;
    uint8_t initialized;
} speed_lungberger_2nd_t;

/* Independent third-order observer states, all in mechanical units. */
typedef struct
{
    float mechanical_angle_rad_hat;
    float mechanical_rad_s_hat;
    float mechanical_rad_s2_hat;
    float angle_error_rad;
    uint8_t initialized;
} speed_Luenberger_3th_t;

/* Independent mechanical model observer; TL includes friction/external load. */
typedef struct
{
    float mechanical_angle_rad_hat;
    float mechanical_rad_s_hat;
    float electrical_rad_s_hat;
    float load_torque_nm_hat;
    float load_torque_rate_nm_s;
    float angle_error_rad;
    float speed_error_rad_s; /* Diagnostic only; not used for correction. */
    uint8_t initialized;
} speed_torque_Luenberger_3th_t;

typedef struct
{
    volatile float mechanical_rad_s_ref; /* Speed target, rad/s. */
		volatile float mechanical_rad_s_fdb; /* Speed target, rad/s. */
    float mechanical_rad_s;
    float mechanical_rad_s_filtered; /* First-order low-pass output, rad/s. */
    float electrical_rad_s;
	  float electrical_rad_s_filtered;
    speed_lungberger_2nd_t observer_2nd;
    speed_Luenberger_3th_t observer_3th;
    speed_torque_Luenberger_3th_t observer_torque_3th;
    uint16_t last_raw;
    uint8_t initialized;
} motor_speed_t;

/* Mechanical angle/error use [-pi, pi). dt and pole_frequency_hz must be > 0.
   First sample aligns the angle and initializes speed/acceleration to zero. */
void speed_Luenberger_3th(speed_Luenberger_3th_t *observer,
                            float mechanical_angle_rad,
                            float dt,
                            float pole_frequency_hz);

/* Mechanical angle/error wrap to [-pi, pi). First sample aligns the angle.
   dt and J must be positive; wc_l is in rad/s. All Euler RHS use old states.
   Kt is the initialized motor torque constant in N*m/A.
   iq_fdb is zero while PWM is disabled. Outputs are observation only. */
void speed_torque_Luenberger_3th(
    speed_torque_Luenberger_3th_t *observer,
    float mechanical_angle_rad, float mechanical_rad_s, float iq_fdb,
    float dt, float wc_l, float J, float B, float Kt, uint8_t pole_pairs);

/* Clear feedback/history; preserve mechanical_rad_s_ref. */
void motor_speed_reset(motor_speed_t *speed);

/* TIM7: raw difference speed every 500 us, also while paused. */
void motor_speed_update(motor_speed_t *speed,
                        const as5047p_sample_t *sample,
                        uint8_t pole_pairs);


/* Call after motor_speed_update at the same 500 us period. */
void speed_LPF_filtered(motor_speed_t *speed, uint8_t pole_pairs);

/* Input angle uses MOTOR_SPEED_DIRECTION; caller converts calibration sign.
   Angle/state/error wrap to [-pi, pi). Initializes without motor_speed_update. */
void speed_lungberger_2nd(speed_lungberger_2nd_t *observer,
                                         float mechanical_angle_rad,
                                         uint8_t pole_pairs);

#endif /* USER_MOTOR_MOTOR_SPEED_H */

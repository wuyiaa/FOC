#ifndef USER_MOTOR_COMMON_H
#define USER_MOTOR_COMMON_H

#include "main.h"
#include "adc.h"
#include "tim.h"
#include "dwt_time.h"
#include "as5047p.h"
#include "motor_speed.h"
#include "motor_POS.h"
#include "motor_config.h"

/* Set to 1U to enable three-phase PWM automatically after initialization. */
#ifndef FOC_PWM_OUTPUT_DEFAULT_ENABLED
#define FOC_PWM_OUTPUT_DEFAULT_ENABLED       0U
#endif

/* Encoder zero-only calibration. */
#ifndef FOC_ENCODER_ZERO_CALIBRATION_MODE
#define FOC_ENCODER_ZERO_CALIBRATION_MODE    0U
#endif

/* Independent switch for automatic phase-order and electrical-direction calibration. */
#ifndef FOC_PHASE_AUTO_CALIBRATION_MODE
#define FOC_PHASE_AUTO_CALIBRATION_MODE      0U
#endif

#define ENCODER_ZERO_ALIGN_TIME_S            1U
#define ENCODER_ZERO_ALIGN_VOLTAGE_V         2.0f
#define ENCODER_ZERO_SAMPLE_TIME_MS          100U

#if ((FOC_ENCODER_ZERO_CALIBRATION_MODE != 0U) && \
     (FOC_PWM_OUTPUT_DEFAULT_ENABLED == 0U))
#error "Enable FOC PWM explicitly to run encoder zero calibration"
#endif

#if ((FOC_PHASE_AUTO_CALIBRATION_MODE != 0U) && \
     (FOC_PWM_OUTPUT_DEFAULT_ENABLED == 0U))
#error "Enable FOC PWM explicitly to run phase auto calibration"
#endif

#if ((FOC_ENCODER_ZERO_CALIBRATION_MODE != 0U) && \
     (FOC_PHASE_AUTO_CALIBRATION_MODE != 0U))
#error "Select either encoder zero calibration or phase auto calibration"
#endif

/* TIM1 PWM and ADC current-sampling window, expressed in timer ticks. */
#define PWM_PERIOD_TICKS                       4000U
#define PWM_HALF_PERIOD_TICKS                  (PWM_PERIOD_TICKS >> 1U)
/* Minimum nominal low-side ON duration over one complete PWM period.
   This duration check does not verify the ADC trigger position. */
#define ADC_SAMPLE_TOTAL_TICKS                  950U

/* Current reconstruction modes selected at compile time. */
#define CURRENT_RECONSTRUCTION_MAX_CURRENT        1U
#define CURRENT_RECONSTRUCTION_MIN_WINDOW         2U

#ifndef CURRENT_RECONSTRUCTION_MODE
#define CURRENT_RECONSTRUCTION_MODE  CURRENT_RECONSTRUCTION_MIN_WINDOW
#endif

/* Fixed coefficients used by the amplitude-invariant Clarke transform. */
#define ONE_BY_THREE                  0.333333333f
#define TWO_BY_THREE                  0.666666667f
#define ONE_BY_SQRT3                  0.577350269f
#define TWO_BY_SQRT3                  1.154700538f
#define TWO_PI                        6.283185307f
#define SQRT3                         1.732050808f
#define HALF_SQRT3                    0.866025404f

/* TIM1: 160 MHz timer clock, center-aligned, ARR=4000 -> 20 kHz. */
#define PWM_PERIOD_S                  0.00005f

typedef enum
{
    PMSM_PHASE_NONE = 0,
    PMSM_PHASE_A,
    PMSM_PHASE_B,
    PMSM_PHASE_C
} pmsm_phase_t;

typedef struct
{
    volatile uint8_t offset_ready;
} pmsm_adc_val_t;

typedef struct
{
    float v_ref;
    float v_adc;
    float i_res;
    float i_op;
    float i_ratio;
    float i_max;
} pmsm_board_t;

typedef struct
{
    float lq;
    float ld;
    float rs;
    float phif; /* Permanent-magnet flux linkage, Wb. */
    float inertia_kg_m2;      /* Rotor and load inertia referred to the motor. */
    float speed_damping_ratio; /* Dimensionless speed-loop damping ratio. */
    uint8_t pn;
} pmsm_motor_t;

typedef struct
{
    uint16_t raw;          /* ADC counts, mapped to the logical phase. */
    float offset;         /* Zero offset in ADC counts. */
    float with_offset;    /* Converted to A, including bias. */
    float nonrecon;       /* Offset removed, polarity corrected, A. */
    float reconstructed;  /* Reconstruction output; otherwise nonrecon. */
} pmsm_phase_current_t;

typedef struct
{
    /* Logical A/B/C, mapped from physical ADC channels before conversion. */
    pmsm_phase_current_t ia;
    pmsm_phase_current_t ib;
    pmsm_phase_current_t ic;
	
		float i_a_ref;
    float i_b_ref;
    float i_c_ref;

    float i_alpha;
    float i_beta;
    float id_ref;
    float iq_ref;
    float id_fdb;
	  float id_fdb_filtered;
    float iq_fdb;
	  float iq_fdb_filtered;
    float Ud_ref;
	  float Ud_ref_filtered;
    float Uq_ref;
    float Ud_ff;
    float Uq_ff;
    float u_alpha;
    float u_beta;
    float electrical_angle_rad;
		float electrical_angle_rad_camp;
    float sin_theta;
		float sin_theta_camp;
    float cos_theta;
		float cos_theta_camp;
    uint8_t angle_valid;
	  float mechanical_angle_rad;
    volatile uint8_t current_limit_fault;
		float Is_filtered;
} pmsm_foc_t;

typedef struct
{
    uint16_t ccr1;
    uint16_t ccr2;
    uint16_t ccr3;
    uint8_t sector;
    float vref2;
		float vref2_min;
} pmsm_svpwm_t;

typedef struct
{
    uint16_t pwm_cmp_a;
    uint16_t pwm_cmp_b;
    uint16_t pwm_cmp_c;

    /* Nominal low-side ON duration: 2 * (ARR - CCR), in timer ticks. */
    uint16_t window_a;
    uint16_t window_b;
    uint16_t window_c;

    /* Per-phase duration >= ADC_SAMPLE_TOTAL_TICKS; 1 means long enough. */
    uint8_t window_a_valid;
    uint8_t window_b_valid;
    uint8_t window_c_valid;
    uint8_t window_valid; /* 1 only when all three duration checks pass. */
    uint8_t full_clarke;

    /* Calibrated phase currents before reconstruction, from this ADC sequence (A). */
    float i_sum_before_reconstruction;

    pmsm_phase_t reconstructed_phase;
} pmsm_current_sample_t;

typedef struct
{
    volatile uint32_t elapsed_ticks;
    volatile uint32_t timer_count;
    volatile uint8_t timer_counting_down;
    /* Trigger-to-callback latency modulo one PWM period, in microseconds. */
    volatile float sample_time_us;
    uint8_t active;
    uint8_t done;
    volatile uint8_t adc1_jeos_at_callback;
    volatile uint8_t adc2_jeos_at_callback;
    volatile uint32_t adc2_jeos_not_set_count;
} pmsm_adc_measure_t;

typedef struct
{
    volatile uint32_t start_phase;
    volatile uint32_t end_phase;
    volatile uint32_t elapsed_ticks;
    /* Valid for callback intervals shorter than one TIM1 PWM period. */
    volatile float foc_time_us;
    volatile uint8_t done;
} pmsm_foc_measure_t;

typedef struct
{
    volatile uint32_t align_start_cycle;
    volatile uint32_t sample_start_cycle;
    volatile uint32_t last_sample_sequence;
    volatile uint32_t sample_count;
    volatile uint16_t last_raw;
    volatile float sample_start_angle_rad;
    volatile float unwrapped_angle_offset_rad;
    volatile float angle_offset_sum_rad;
    volatile uint16_t mechanical_zero_raw;
    volatile float mechanical_zero_angle_rad;
    volatile uint8_t started;
    volatile uint8_t sampling;
    volatile uint8_t done;
    volatile uint8_t flash_save_attempted;
    volatile uint8_t flash_saved;
    volatile uint8_t phase_reversed;
    volatile int8_t electrical_direction;
    volatile uint8_t state;
    volatile uint8_t failed;
    volatile uint8_t direction_retried;
    volatile uint8_t phase_retried;
    volatile uint32_t drive_start_cycle;
    volatile uint8_t drive_observation_started;
    volatile uint16_t drive_last_raw;
    /* Current polarity: persisted with zero/phase; +1 keeps sign, -1 reverses. */
    int8_t current_direction;
    uint8_t current_polarity_required;
    /* 0=idle, 1=settling, 2=averaging, 3=valid, 4=failed. */
    uint8_t current_polarity_status;
    uint32_t current_polarity_start_cycle;
    uint32_t current_polarity_count;
    float current_polarity_sum[3];
    float current_polarity_mean[3]; /* Logical A/B/C, before correction. */
    volatile int32_t drive_delta_counts;
} pmsm_encoder_zero_cal_t;

typedef struct
{
    pmsm_board_t board;
    pmsm_motor_t motor;
    as5047p_t encoder;
    as5047p_sample_t encoder_foc_sample;
    motor_speed_t speed;
    motor_pos_t pos;
    pmsm_adc_val_t adc;
    pmsm_foc_t foc;
    pmsm_svpwm_t svpwm;
    pmsm_current_sample_t current_sample;
    pmsm_adc_measure_t adc_measure;
    pmsm_foc_measure_t foc_measure;
} pmsm_t;

extern pmsm_t pm;

void pmsm_board_init(void);
void pmsm_motor_init(void);
void pmsm_init(void);
void foc_get_curr_off(void);
void foc_adc_sample(pmsm_t *pm);
void foc_current_sample_process(pmsm_t *pm);
void foc_angle_reset(pmsm_t *pm);
void foc_electrical_angle_update(pmsm_t *pm);
void clarke_transform(pmsm_foc_t *foc, uint8_t full_clarke);
void inverse_clarke_transform(pmsm_foc_t *foc);
void park_transform(pmsm_t *pm);
void foc_current_calc(pmsm_t *pm);
void inverse_park_transform(pmsm_t *pm);
void svpwm_calc(pmsm_t *pm, float v_dc, float v_min);
void foc_voltage_calc(pmsm_t *pm, float v_dc, float v_min);
void output_pwm_init(pmsm_t *pm);
HAL_StatusTypeDef foc_pwm_output_enable(pmsm_t *pm);
void foc_pwm_output_disable(void);
uint8_t foc_pwm_output_is_enabled(void);
void foc_pwm_output_update(const pmsm_t *pm);

#endif /* USER_MOTOR_COMMON_H */

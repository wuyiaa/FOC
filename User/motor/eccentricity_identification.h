#ifndef USER_ECCENTRICITY_IDENTIFICATION_H
#define USER_ECCENTRICITY_IDENTIFICATION_H

#include "common.h"

/* One saved point per 200 us; acquisition/FOC still run every 50 us. */
#define ECCENTRICITY_RECORD_INTERVAL 4U
#define ECCENTRICITY_MAX_SAMPLES 7000U
#define ECCENTRICITY_LUT_SIZE 256U
#define ECCENTRICITY_FLASH_RECORD_WORDS (ECCENTRICITY_LUT_SIZE / 2U + 2U)

typedef struct
{
    /* Forward data is no longer needed after pairing. Reuse its RAM for FIR. */
    union
    {
        struct
        {
            float angle_deg[ECCENTRICITY_MAX_SAMPLES];
            float error_deg[ECCENTRICITY_MAX_SAMPLES];
        } forward;
        struct
        {
            /* Descending uniform grid: angle[i] = 360*(N-1-i)/N deg. */
            float uniform_error_deg[ECCENTRICITY_MAX_SAMPLES];
            float filtered_error_deg[ECCENTRICITY_MAX_SAMPLES];
        } fir;
    } curve;
    /* Original reverse-order paired errors. After stage 3 these have b
       removed; preserve them as an unfiltered reference for comparison. */
    float paired_error_deg[ECCENTRICITY_MAX_SAMPLES];
    /* Compact original paired coordinates: 0..65535 means 0..360 deg. */
    uint16_t paired_angle_phase[ECCENTRICITY_MAX_SAMPLES];
    float offset_b_deg;
    /* Zero until done. Then replay the centered cyclic FIR result. */
    float paired_error_deg_display;
    /* Unfiltered paired error interpolated onto the same display angle. */
    float paired_error_deg_before_fir_display;
    float paired_angle_deg_display;
    float paired_electrical_angle_deg_display;
    float fir_window_deg;
    uint16_t forward_sample_count;
    uint16_t paired_sample_count;
    uint16_t fir_sample_count;   /* Uniform samples per mechanical revolution N. */
    uint16_t fir_window_samples; /* Taps M=N/p, all coefficients are 1/M. */
    /* 0: waiting, 1: forward, 2: reverse, 3: removing b,
       4: uniform resampling, 5: FIR sum initialization,
       6: cyclic FIR, 7: complete. */
    uint8_t stage;
    uint8_t done;
} eccentricity_measure_t;

typedef struct
{
    float correction_deg_lut[ECCENTRICITY_LUT_SIZE];
    float mechanical_angle_rad_before;
    float mechanical_angle_rad_after;
    float mechanical_angle_deg_before;
    float mechanical_angle_deg_after;
    float correction_deg;
    uint16_t mechanical_zero_raw;
    int8_t electrical_direction;
    uint8_t ready;
    uint8_t flash_loaded;
    uint8_t save_pending;
    uint8_t flash_saved;
    HAL_StatusTypeDef flash_status;
} eccentricity_compensation_t;

extern eccentricity_compensation_t eccentricity_compensation;
extern eccentricity_measure_t eccentricity_measure;
extern float theta_e;

void eccentricity_VF(pmsm_t *pm);
void eccentricity_identification(pmsm_t *pm);
float eccentricity_compensation_lookup_deg(float mechanical_angle_deg);
void eccentricity_compensation_observe(pmsm_t *pm);
HAL_StatusTypeDef eccentricity_compensation_flash_load(void);
void eccentricity_compensation_save_process(void);
uint8_t eccentricity_flash_record_valid(const volatile uint64_t *record);

#endif /* USER_ECCENTRICITY_IDENTIFICATION_H */

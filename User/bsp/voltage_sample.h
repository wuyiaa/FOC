#ifndef USER_BSP_VOLTAGE_SAMPLE_H
#define USER_BSP_VOLTAGE_SAMPLE_H

#include "adc.h"

/* SCH_axdr_v1.2: every voltage divider is (15k + 15k) / 2k. */
#define VOLTAGE_ADC_REFERENCE_V       3.3f
#define VOLTAGE_ADC_FULL_SCALE        4096.0f
#define VOLTAGE_DIVIDER_TOP_OHM        30000.0f
#define VOLTAGE_DIVIDER_BOTTOM_OHM     2000.0f
#define VOLTAGE_ADC_V_PER_COUNT \
    (VOLTAGE_ADC_REFERENCE_V / VOLTAGE_ADC_FULL_SCALE * \
     (VOLTAGE_DIVIDER_TOP_OHM + VOLTAGE_DIVIDER_BOTTOM_OHM) / \
     VOLTAGE_DIVIDER_BOTTOM_OHM)

typedef struct
{
    uint16_t va_raw;
    uint16_t vb_raw;
    uint16_t vc_raw;
    uint16_t vbus_raw;

    /* Physical board terminals MA/MB/MC to GND; no phase-order remapping. */
    float va_v;
    float vb_v;
    float vc_v;
    float vbus_v;

    /* Publication counts, not the total number of hardware conversions. */
    uint32_t phase_update_count;
    uint32_t bus_update_count;
    uint32_t bus_not_ready_count;
    uint32_t adc_error_code;
    uint8_t phase_valid;
    uint8_t bus_valid;
    uint8_t bus_fresh;
} voltage_sample_t;

/* Keil Watch: voltage_sample. All voltage fields are unfiltered volts. */
extern volatile voltage_sample_t voltage_sample;

/* Call once after MX_ADC2_Init(), before starting TIM1 CH4. */
HAL_StatusTypeDef voltage_sample_start(void);
/* Call only from the ADC1 injected-sequence-complete callback. */
void voltage_sample_update(void);

#endif /* USER_BSP_VOLTAGE_SAMPLE_H */

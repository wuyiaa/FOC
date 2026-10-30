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

#define VOLTAGE_AVERAGE_SAMPLE_COUNT 32U//32

typedef struct
{
    uint16_t raw; /* ADC counts before conversion. */
    float value;  /* Terminal-to-GND voltage, divider restored, V. */
    float average; /* Moving mean of published samples, not PWM-cycle mean. */
} terminal_voltage_sample_t;

typedef struct
{
    /* Physical terminals, independent of calibrated phase-order mapping. */
    terminal_voltage_sample_t Uag;
    terminal_voltage_sample_t Ubg;
    terminal_voltage_sample_t Ucg;
    uint16_t vbus_raw;
    float vbus_v;

    /* Estimated neutral and winding voltages for physical MA/MB/MC (V). */
    float U_ng;
    float Uan;
    float Ubn;
    float Ucn;

    /* Clarke/Park results in the calibrated logical A/B/C frame (V). */
    float Ualpha_fdb;
    float Ubeta_fdb;
    float Ud_fdb;
    float Uq_fdb;

    /* Publication counts, not the total number of hardware conversions. */
    uint32_t phase_update_count;
    uint32_t bus_update_count;
    uint32_t transform_update_count;
    uint32_t bus_not_ready_count;
    uint32_t adc_error_code;
    uint8_t phase_valid;
    uint8_t bus_valid;
    uint8_t bus_fresh;
    uint8_t transform_valid;
} voltage_sample_t;

/* Keil Watch: voltage_sample. Voltage values are volts; raw fields are ADC counts. */
extern volatile voltage_sample_t voltage_sample;

/* Call once after MX_ADC2_Init(), before starting TIM1 CH4. */
HAL_StatusTypeDef voltage_sample_start(void);
/* Call only from the ADC1 injected-sequence-complete callback. */
void voltage_sample_bus_update(void);
/* Main loop: read a completed ADC2 regular DMA scan and update phase voltages. */
uint8_t voltage_sample_regular_update(void);
/* Main loop: estimate winding voltages and transform to alpha/beta/dq. */
void voltage_sample_transform_update(float electrical_angle_rad,
                                     uint8_t phase_reversed);

#endif /* USER_BSP_VOLTAGE_SAMPLE_H */

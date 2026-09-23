#ifndef USER_MOTOR_AS5047P_H
#define USER_MOTOR_AS5047P_H

#include "main.h"

#define AS5047P_DATA_MASK              0x3FFFU
#define AS5047P_RAD_PER_COUNT          0.000383495197f
#define AS5047P_CPU_CYCLE_TO_US        0.00625f
#define AS5047P_FAULT_LIMIT            5U

typedef enum
{
    AS5047P_DMA_IDLE = 0,
    AS5047P_DMA_ACTIVE
} as5047p_dma_state_t;

typedef struct
{
    uint16_t raw;
    uint8_t valid;
    uint32_t sequence;
} as5047p_sample_t;

typedef struct
{
    volatile uint16_t encoder_raw_latest;
    volatile uint8_t encoder_valid;
    volatile uint32_t encoder_sample_sequence;
    volatile float mechanical_angle_rad_latest;

    volatile uint32_t transfer_start_cycle;
    volatile uint32_t transfer_cycles;
    volatile float transfer_time_us;
    volatile uint32_t angle_request_cycle;
    volatile uint32_t angle_sample_cycles;
    volatile float angle_sample_time_us;

    volatile as5047p_dma_state_t dma_state;
    volatile uint8_t pipeline_primed;
    uint16_t dma_tx_word;
    uint16_t dma_rx_word;

    volatile uint32_t busy_skip_count;
    volatile uint32_t dma_error_count;
    volatile uint32_t parity_error_count;
    volatile uint32_t sensor_error_count;

    uint32_t sequence_last;
    volatile uint8_t consecutive_error_count;
    volatile uint8_t fault;
} as5047p_t;

void as5047p_init(as5047p_t *encoder);
HAL_StatusTypeDef as5047p_read_start_dma(as5047p_t *encoder);
uint8_t as5047p_latest_get(const as5047p_t *encoder,
                           as5047p_sample_t *sample);
uint8_t as5047p_protection_update(as5047p_t *encoder,
                                  const as5047p_sample_t *sample);

#endif /* USER_MOTOR_AS5047P_H */

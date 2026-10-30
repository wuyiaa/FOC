#ifndef USER_APP_AS5047P_H
#define USER_APP_AS5047P_H

#include "main.h"

#define AS5047P_DATA_MASK              0x3FFFU
#define AS5047P_RAD_PER_COUNT          0.000383495197f
#define AS5047P_COUNT_PER_RAD          2607.59459f
#define AS5047P_FAULT_LIMIT            5U

/* 0: angle requests only; 1: insert ERRFL reads on parity error or EF. */
#ifndef AS5047P_ERRFL_AUTO_READ_ENABLE
#define AS5047P_ERRFL_AUTO_READ_ENABLE 0U
#endif

typedef enum
{
    AS5047P_IDLE = 0,
    AS5047P_ACTIVE
} as5047p_state_t;

typedef struct
{
    uint16_t raw;
    uint8_t valid;
    uint32_t sequence;
    uint32_t cycle;
} as5047p_sample_t;

typedef struct
{
    volatile uint16_t encoder_raw_latest;
    volatile uint8_t encoder_valid;
    volatile uint32_t encoder_sample_sequence;
    volatile uint32_t encoder_sample_cycle;
    volatile float mechanical_angle_rad_latest;

    volatile uint32_t transfer_start_cycle; /* DWT angle timestamp for speed. */
    volatile uint32_t transfer_start_phase; /* TIM1 phase for DMA timing. */
    volatile uint32_t transfer_timer_ticks;
    volatile float transfer_time_us;
    volatile uint32_t read_timer_ticks; /* Whole blocking read, TIM1 ticks. */
    volatile float read_time_us;
    volatile uint32_t angle_request_cycle;

    volatile as5047p_state_t state;
    volatile uint8_t pipeline_primed;
    uint16_t tx_word;
    uint16_t rx_word;

    volatile uint32_t busy_skip_count;
    volatile uint32_t dma_error_count;
    volatile uint32_t spi_error_count;
    /* Latest received word and diagnostics; never reject an angle. */
    volatile uint16_t rx_response_word;
    volatile uint8_t rx_parity_bit;
    volatile uint8_t rx_ef_flag;
    volatile uint8_t rx_parity_error;
    volatile uint32_t parity_error_count;
    volatile uint32_t sensor_error_count;
    volatile uint16_t errfl_response_word;
    volatile uint16_t errfl_status;
    volatile uint32_t errfl_read_count;
    volatile uint8_t errfl_valid;
    volatile uint8_t errfl_parity_error;
    volatile uint8_t errfl_invalid_command;
    volatile uint8_t errfl_frame_error;
    volatile uint8_t errfl_read_pending;
    volatile uint8_t errfl_read_in_progress;
    volatile uint8_t tx_is_errfl;
    volatile uint8_t response_is_errfl_pending;

    uint32_t sequence_last;
    volatile uint8_t consecutive_error_count;
    volatile uint8_t fault;
} as5047p_t;

void as5047p_init(as5047p_t *encoder);
HAL_StatusTypeDef as5047p_read_start_dma(as5047p_t *encoder);
/* One blocking 16-bit frame, without timeout; RX is the preceding response.
   The FOC sample receives the latest published valid angle. */
HAL_StatusTypeDef as5047p_read_blocking(as5047p_t *encoder,
                                        as5047p_sample_t *sample);
uint8_t as5047p_latest_get(const as5047p_t *encoder,
                           as5047p_sample_t *sample);
uint8_t as5047p_protection_update(as5047p_t *encoder,
                                  const as5047p_sample_t *sample);

#endif /* USER_APP_AS5047P_H */

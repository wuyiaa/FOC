#include "as5047p.h"

#include <stddef.h>

#include "spi.h"

#define AS5047P_READ_BIT               0x4000U
#define AS5047P_PARITY_BIT             0x8000U
#define AS5047P_ERROR_BIT              0x4000U
#define AS5047P_ERRFL_REG              0x0001U
#define AS5047P_ANGLECOM_REG           0x3FFFU
#define AS5047P_ERRFL_PARERR_BIT       0x0004U
#define AS5047P_ERRFL_INVCOMM_BIT      0x0002U
#define AS5047P_ERRFL_FRERR_BIT        0x0001U

static as5047p_t *as5047p_active_encoder;

static uint8_t as5047p_parity_is_odd(uint16_t value)
{
    value ^= value >> 8U;
    value ^= value >> 4U;
    value ^= value >> 2U;
    value ^= value >> 1U;

    return (uint8_t)(value & 1U);
}

static uint16_t as5047p_make_read_command(uint16_t address)
{
    uint16_t command = AS5047P_READ_BIT | (address & AS5047P_DATA_MASK);

    if (as5047p_parity_is_odd(command) != 0U)
    {
        command |= AS5047P_PARITY_BIT;
    }

    return command;
}

static void as5047p_transaction_end(as5047p_t *encoder)
{
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
    /* Clear the owner before publishing IDLE so a higher-priority FOC IRQ
       cannot start a new transaction and then have its owner cleared here. */
    as5047p_active_encoder = NULL;
    __DMB();
    encoder->dma_state = AS5047P_DMA_IDLE;
}

static void as5047p_publish(as5047p_t *encoder, uint16_t raw_angle)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    encoder->encoder_raw_latest = raw_angle;
    encoder->mechanical_angle_rad_latest =
        (float)raw_angle * AS5047P_RAD_PER_COUNT;
    encoder->encoder_sample_sequence++;
    __DMB();
    encoder->encoder_valid = 1U;
    __DMB();

    if (primask == 0U)
    {
        __enable_irq();
    }
}

void as5047p_init(as5047p_t *encoder)
{
    if (encoder == NULL)
    {
        return;
    }

    encoder->dma_state = AS5047P_DMA_IDLE;
    encoder->pipeline_primed = 0U;
    encoder->encoder_valid = 0U;
    encoder->transfer_start_cycle = 0U;
    encoder->transfer_cycles = 0U;
    encoder->transfer_time_us = 0.0f;
    encoder->angle_request_cycle = 0U;
    encoder->angle_sample_cycles = 0U;
    encoder->angle_sample_time_us = 0.0f;
    encoder->sequence_last = 0U;
    encoder->consecutive_error_count = 0U;
    encoder->errfl_response_word = 0U;
    encoder->errfl_status = 0U;
    encoder->errfl_read_count = 0U;
    encoder->errfl_valid = 0U;
    encoder->errfl_parity_error = 0U;
    encoder->errfl_invalid_command = 0U;
    encoder->errfl_frame_error = 0U;
    encoder->errfl_read_pending = 0U;
    encoder->errfl_read_in_progress = 0U;
    encoder->dma_tx_is_errfl = 0U;
    encoder->response_is_errfl_pending = 0U;
    encoder->fault = 0U;
    as5047p_active_encoder = NULL;
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef as5047p_read_start_dma(as5047p_t *encoder)
{
    HAL_StatusTypeDef status;
    uint8_t tx_is_errfl;

    if (encoder == NULL)
    {
        return HAL_ERROR;
    }

    if ((encoder->dma_state != AS5047P_DMA_IDLE) ||
        (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY))
    {
        encoder->busy_skip_count++;
        return HAL_BUSY;
    }

    tx_is_errfl = encoder->errfl_read_pending;
    encoder->dma_tx_word = as5047p_make_read_command(
        (tx_is_errfl != 0U) ? AS5047P_ERRFL_REG : AS5047P_ANGLECOM_REG);
    encoder->dma_rx_word = 0U;
    encoder->dma_state = AS5047P_DMA_ACTIVE;
    as5047p_active_encoder = encoder;

    /* Measure from CS assertion/DMA launch to the DMA-complete callback. */
    encoder->transfer_start_cycle = DWT->CYCCNT;
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive_DMA(&hspi1,
                                        (uint8_t *)&encoder->dma_tx_word,
                                        (uint8_t *)&encoder->dma_rx_word,
                                        1U);
    if (status != HAL_OK)
    {
        HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
        encoder->dma_state = AS5047P_DMA_IDLE;
        as5047p_active_encoder = NULL;

        if (status == HAL_BUSY)
        {
            encoder->busy_skip_count++;
        }
        else
        {
            encoder->dma_error_count++;
            encoder->pipeline_primed = 0U;
            encoder->angle_request_cycle = 0U;
            encoder->response_is_errfl_pending = 0U;
        }
    }
    else
    {
        encoder->dma_tx_is_errfl = tx_is_errfl;
        if (tx_is_errfl != 0U)
        {
            encoder->errfl_read_pending = 0U;
            encoder->errfl_read_in_progress = 1U;
        }
    }

    return status;
}

uint8_t as5047p_latest_get(const as5047p_t *encoder,
                           as5047p_sample_t *sample)
{
    uint32_t primask;

    if ((encoder == NULL) || (sample == NULL))
    {
        return 0U;
    }

    /* A short critical section gives the FOC ISR one coherent publication. */
    primask = __get_PRIMASK();
    __disable_irq();
    sample->raw = encoder->encoder_raw_latest;
    sample->sequence = encoder->encoder_sample_sequence;
    sample->valid = encoder->encoder_valid;
    __DMB();

    if (primask == 0U)
    {
        __enable_irq();
    }

    return sample->valid;
}

uint8_t as5047p_protection_update(as5047p_t *encoder,
                                  const as5047p_sample_t *sample)
{
    if ((encoder == NULL) || (sample == NULL))
    {
        return 1U;
    }

    if (encoder->fault != 0U)
    {
        return 1U;
    }

    if ((sample->valid != 0U) &&
        (sample->sequence != encoder->sequence_last))
    {
        encoder->sequence_last = sample->sequence;
        encoder->consecutive_error_count = 0U;
        return 0U;
    }
//encoder->consecutive_error_count++;
    if (encoder->consecutive_error_count < AS5047P_FAULT_LIMIT)
    {
        encoder->consecutive_error_count++;
    }

    if (encoder->consecutive_error_count >= AS5047P_FAULT_LIMIT)
    {
        encoder->fault = 1U;
    }

    return encoder->fault;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    as5047p_t *encoder = as5047p_active_encoder;
    uint32_t transfer_end_cycle = DWT->CYCCNT;
    uint32_t angle_sample_cycles;
    uint16_t response;
    uint8_t response_is_errfl;

    if ((hspi->Instance != SPI1) || (encoder == NULL))
    {
        return;
    }

    encoder->transfer_cycles =
        (uint32_t)(transfer_end_cycle - encoder->transfer_start_cycle);
    encoder->transfer_time_us =
        (float)encoder->transfer_cycles * AS5047P_CPU_CYCLE_TO_US;

    if (encoder->dma_state != AS5047P_DMA_ACTIVE)
    {
        encoder->dma_error_count++;
        encoder->pipeline_primed = 0U;
        as5047p_transaction_end(encoder);
        return;
    }

    response = encoder->dma_rx_word;
    as5047p_transaction_end(encoder);

    /*
     * One-frame continuous pipeline: normally each FOC period sends ANGLECOM.
     * After EF is observed, one ERRFL read is inserted. RX always belongs to
     * the preceding transaction, so only the first completed frame is dropped.
     * CS remains high between periods; no delay or polling is used.
     */
    if (encoder->pipeline_primed == 0U)
    {
        encoder->pipeline_primed = 1U;
        encoder->response_is_errfl_pending = encoder->dma_tx_is_errfl;
        encoder->angle_request_cycle = encoder->transfer_start_cycle;
        return;
    }

    /* SPI is pipelined: RX is the response to the preceding transaction. */
    response_is_errfl = encoder->response_is_errfl_pending;
    encoder->response_is_errfl_pending = encoder->dma_tx_is_errfl;

    if (response_is_errfl != 0U)
    {
        /* Capture and decode ERRFL before normal EF handling discards the frame. */
        encoder->errfl_response_word = response;
        encoder->errfl_read_count++;

        if (as5047p_parity_is_odd(response) != 0U)
        {
            encoder->parity_error_count++;
            encoder->errfl_valid = 0U;
            encoder->errfl_read_in_progress = 0U;
            encoder->angle_request_cycle = encoder->transfer_start_cycle;
            return;
        }

        encoder->errfl_status = response & AS5047P_DATA_MASK;
        encoder->errfl_parity_error =
            (uint8_t)((encoder->errfl_status & AS5047P_ERRFL_PARERR_BIT) != 0U);
        encoder->errfl_invalid_command =
            (uint8_t)((encoder->errfl_status & AS5047P_ERRFL_INVCOMM_BIT) != 0U);
        encoder->errfl_frame_error =
            (uint8_t)((encoder->errfl_status & AS5047P_ERRFL_FRERR_BIT) != 0U);
        encoder->errfl_valid = 1U;
        encoder->errfl_read_in_progress = 0U;

        /* The current transaction sent ANGLECOM; restart its timing window. */
        encoder->angle_request_cycle = encoder->transfer_start_cycle;
        return;
    }

    /* This response belongs to the command sent in the preceding FOC period. */
    angle_sample_cycles =
        (uint32_t)(transfer_end_cycle - encoder->angle_request_cycle);
    encoder->angle_request_cycle = encoder->transfer_start_cycle;

    if (as5047p_parity_is_odd(response) != 0U)
    {
        encoder->parity_error_count++;
        return;
    }

    if ((response & AS5047P_ERROR_BIT) != 0U)
    {
        encoder->sensor_error_count++;
        if ((encoder->errfl_read_pending == 0U) &&
            (encoder->errfl_read_in_progress == 0U))
        {
            encoder->errfl_valid = 0U;
            encoder->errfl_read_pending = 1U;
        }
        return;
    }

    encoder->angle_sample_cycles = angle_sample_cycles;
    encoder->angle_sample_time_us =
        (float)angle_sample_cycles * AS5047P_CPU_CYCLE_TO_US;
    as5047p_publish(encoder, response & AS5047P_DATA_MASK);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    as5047p_t *encoder = as5047p_active_encoder;

    if ((hspi->Instance != SPI1) || (encoder == NULL))
    {
        return;
    }

    encoder->dma_error_count++;
    if (encoder->errfl_read_in_progress != 0U)
    {
        encoder->errfl_read_pending = 1U;
        encoder->errfl_read_in_progress = 0U;
    }
    encoder->pipeline_primed = 0U;
    encoder->response_is_errfl_pending = 0U;
    encoder->angle_request_cycle = 0U;
    as5047p_transaction_end(encoder);
}

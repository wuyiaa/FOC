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
static float as5047p_timer_tick_us;
static uint32_t as5047p_timeout_ticks;
static uint32_t as5047p_timer_phase_get(void);
static uint32_t as5047p_timer_elapsed(uint32_t start_phase);
static void as5047p_response_process(as5047p_t *encoder, uint16_t response,
                                     as5047p_sample_t *sample);

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
    encoder->state = AS5047P_IDLE;
}

static void as5047p_publish(as5047p_t *encoder,
                            uint16_t raw_angle,
                            uint32_t sample_cycle,
                            as5047p_sample_t *sample)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    encoder->encoder_raw_latest = raw_angle;
    encoder->encoder_sample_cycle = sample_cycle;
    encoder->mechanical_angle_rad_latest =
        (float)raw_angle * AS5047P_RAD_PER_COUNT;
    encoder->encoder_sample_sequence++;
    __DMB();
    encoder->encoder_valid = 1U;
    if (sample != NULL)
    {
        sample->raw = raw_angle;
        sample->sequence = encoder->encoder_sample_sequence;
        sample->cycle = sample_cycle;
        sample->valid = 1U;
    }
    __DMB();

    if (primask == 0U)
    {
        __enable_irq();
    }
}

void as5047p_init(as5047p_t *encoder)
{
    uint32_t timer_clock_hz;

    if (encoder == NULL)
    {
        return;
    }

    /* TIM1 is configured and running before encoder initialization. */
    timer_clock_hz = HAL_RCC_GetPCLK2Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0U)
    {
        timer_clock_hz *= 2U;
    }
    as5047p_timer_tick_us = 1000000.0f * (float)(TIM1->PSC + 1U) /
                            (float)timer_clock_hz;
    as5047p_timeout_ticks = (uint32_t)
        ((float)AS5047P_SPI_TIMEOUT_US / as5047p_timer_tick_us + 0.5f);

    encoder->state = AS5047P_IDLE;
    encoder->pipeline_primed = 0U;
    encoder->encoder_valid = 0U;
    encoder->encoder_sample_cycle = 0U;
    encoder->transfer_start_cycle = 0U;
    encoder->transfer_start_phase = 0U;
    encoder->transfer_timer_ticks = 0U;
    encoder->transfer_time_us = 0.0f;
    encoder->read_timer_ticks = 0U;
    encoder->read_time_us = 0.0f;
    encoder->angle_request_cycle = 0U;
    encoder->sequence_last = 0U;
    encoder->consecutive_error_count = 0U;
    encoder->rx_response_word = 0U;
    encoder->rx_parity_bit = 0U;
    encoder->rx_ef_flag = 0U;
    encoder->rx_parity_error = 0U;
    encoder->parity_error_count = 0U;
    encoder->sensor_error_count = 0U;
    encoder->errfl_response_word = 0U;
    encoder->errfl_status = 0U;
    encoder->errfl_read_count = 0U;
    encoder->errfl_valid = 0U;
    encoder->errfl_parity_error = 0U;
    encoder->errfl_invalid_command = 0U;
    encoder->errfl_frame_error = 0U;
    encoder->errfl_read_pending = 0U;
    encoder->errfl_read_in_progress = 0U;
    encoder->tx_is_errfl = 0U;
    encoder->response_is_errfl_pending = 0U;
    encoder->fault = 0U;
    encoder->spi_error_count = 0U;
    encoder->spi_timeout_count = 0U;
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

    if ((encoder->state != AS5047P_IDLE) ||
        (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY))
    {
        encoder->busy_skip_count++;
        return HAL_BUSY;
    }

#if (AS5047P_ERRFL_AUTO_READ_ENABLE != 0U)
    tx_is_errfl = encoder->errfl_read_pending;
#else
    tx_is_errfl = 0U;
#endif
    encoder->tx_word = as5047p_make_read_command(
        (tx_is_errfl != 0U) ? AS5047P_ERRFL_REG : AS5047P_ANGLECOM_REG);
    encoder->rx_word = 0U;
    encoder->state = AS5047P_ACTIVE;
    as5047p_active_encoder = encoder;

    /* DWT is only the angle timestamp; TIM1 measures transfer duration. */
    encoder->transfer_start_cycle = DWT->CYCCNT;
    encoder->transfer_start_phase = as5047p_timer_phase_get();
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive_DMA(&hspi1,
                                        (uint8_t *)&encoder->tx_word,
                                        (uint8_t *)&encoder->rx_word,
                                        1U);
    if (status != HAL_OK)
    {
        HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
        encoder->state = AS5047P_IDLE;
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
        encoder->tx_is_errfl = tx_is_errfl;
        if (tx_is_errfl != 0U)
        {
            encoder->errfl_read_pending = 0U;
            encoder->errfl_read_in_progress = 1U;
        }
    }

    return status;
}

/* Unfold center-aligned TIM1 into a phase from 0 to 2 * ARR. */
static uint32_t as5047p_timer_phase_get(void)
{
    uint32_t direction_before;
    uint32_t direction_after;
    uint32_t count;

    do
    {
        direction_before = TIM1->CR1 & TIM_CR1_DIR;
        count = TIM1->CNT;
        direction_after = TIM1->CR1 & TIM_CR1_DIR;
    } while (direction_before != direction_after);

    return (direction_after != 0U) ? (2U * TIM1->ARR - count) : count;
}

/* Valid for elapsed times below one PWM period (currently 50 us). */
static uint32_t as5047p_timer_elapsed(uint32_t start_phase)
{
    uint32_t end_phase = as5047p_timer_phase_get();
    uint32_t period = 2U * TIM1->ARR;

    return (end_phase >= start_phase) ?
        (end_phase - start_phase) : (period - start_phase + end_phase);
}

/* All three SPI waits share the same TIM1 start point and timeout budget. */
static HAL_StatusTypeDef as5047p_wait_spi(uint32_t flag, uint32_t expected,
                                        uint32_t start_phase)
{
    uint32_t sr;

    do
    {
        sr = hspi1.Instance->SR;
        if ((sr & (SPI_SR_OVR | SPI_SR_MODF | SPI_SR_FRE)) != 0U)
        {
            return HAL_ERROR;
        }
        if (as5047p_timer_elapsed(start_phase) >= as5047p_timeout_ticks)
        {
            return HAL_TIMEOUT;
        }
        if ((sr & flag) == expected)
        {
            return HAL_OK;
        }
    } while (1);
}

HAL_StatusTypeDef as5047p_read_blocking(as5047p_t *encoder,
                                        as5047p_sample_t *sample)
{
    HAL_StatusTypeDef status;
    uint32_t start_phase;
    uint32_t read_start_phase;
    uint32_t read_elapsed_ticks;
    uint32_t i;
    uint8_t tx_is_errfl;
    uint16_t response;

    if ((encoder == NULL) || (sample == NULL))
    {
        return HAL_ERROR;
    }
    if ((encoder->state != AS5047P_IDLE) ||
        (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) ||
        (hspi1.Lock == HAL_LOCKED))
    {
        encoder->busy_skip_count++;
        return HAL_BUSY;
    }

    /* Match the full read measurement to the PB11 low pulse. */
    ENCODER_TIME_MARK_GPIO_Port->BSRR = (uint32_t)ENCODER_TIME_MARK_Pin << 16U;
    read_start_phase = as5047p_timer_phase_get();

    hspi1.Lock = HAL_LOCKED;
    hspi1.State = HAL_SPI_STATE_BUSY_TX_RX;
    encoder->state = AS5047P_ACTIVE; /* Shared transaction guard only. */
#if (AS5047P_ERRFL_AUTO_READ_ENABLE != 0U)
    tx_is_errfl = encoder->errfl_read_pending;
#else
    tx_is_errfl = 0U;
#endif
    encoder->tx_word = as5047p_make_read_command(
        (tx_is_errfl != 0U) ? AS5047P_ERRFL_REG : AS5047P_ANGLECOM_REG);
    encoder->rx_word = 0U;

    /* Poll SPI hardware directly: no DMA requests or completion interrupt. */
    CLEAR_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN |
                                  SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE);
    __HAL_SPI_ENABLE(&hspi1);
    encoder->transfer_start_cycle = DWT->CYCCNT;
    start_phase = as5047p_timer_phase_get();
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
    status = as5047p_wait_spi(SPI_SR_TXE, SPI_SR_TXE,
                              start_phase);
    if (status == HAL_OK)
    {
        *(__IO uint16_t *)&hspi1.Instance->DR = encoder->tx_word;
        status = as5047p_wait_spi(SPI_SR_RXNE, SPI_SR_RXNE,
                                  start_phase);
    }
    if (status == HAL_OK)
    {
        encoder->rx_word = *(__IO uint16_t *)&hspi1.Instance->DR;
        status = as5047p_wait_spi(SPI_SR_BSY, 0U,
                                  start_phase);
    }

    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
    encoder->transfer_timer_ticks = as5047p_timer_elapsed(start_phase);
    encoder->transfer_time_us =
        (float)encoder->transfer_timer_ticks * as5047p_timer_tick_us;

    if (status == HAL_OK)
    {
        response = encoder->rx_word;
        encoder->tx_is_errfl = tx_is_errfl;
        if (tx_is_errfl != 0U)
        {
            encoder->errfl_read_pending = 0U;
            encoder->errfl_read_in_progress = 1U;
        }
        /* Finish parsing before releasing the transaction guard. */
        as5047p_response_process(encoder, response, sample);
    }
    else
    {
        encoder->spi_error_count++;
        if (status == HAL_TIMEOUT)
        {
            encoder->spi_timeout_count++;
        }
        __HAL_SPI_DISABLE(&hspi1);
        /* Flush at most the four-byte FIFO; never wait indefinitely on error. */
        for (i = 0U; i < 4U; i++)
        {
            if ((hspi1.Instance->SR & SPI_SR_FRLVL) == 0U)
            {
                break;
            }
            (void)*(__IO uint8_t *)&hspi1.Instance->DR;
        }
        __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
        if (encoder->errfl_read_in_progress != 0U)
        {
            encoder->errfl_read_pending = 1U;
        }
        encoder->errfl_read_in_progress = 0U;
        encoder->pipeline_primed = 0U;
        encoder->response_is_errfl_pending = 0U;
        encoder->angle_request_cycle = 0U;
    }

    hspi1.State = HAL_SPI_STATE_READY;
    hspi1.Lock = HAL_UNLOCKED;
    __DMB();
    encoder->state = AS5047P_IDLE;
    read_elapsed_ticks = as5047p_timer_elapsed(read_start_phase);
    ENCODER_TIME_MARK_GPIO_Port->BSRR = ENCODER_TIME_MARK_Pin;
    encoder->read_timer_ticks = read_elapsed_ticks;
    encoder->read_time_us = (float)read_elapsed_ticks * as5047p_timer_tick_us;
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
    sample->cycle = encoder->encoder_sample_cycle;
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
    uint16_t response;

    if ((hspi->Instance != SPI1) || (encoder == NULL))
    {
        return;
    }

    encoder->transfer_timer_ticks =
        as5047p_timer_elapsed(encoder->transfer_start_phase);
    encoder->transfer_time_us =
        (float)encoder->transfer_timer_ticks * as5047p_timer_tick_us;

    if (encoder->state != AS5047P_ACTIVE)
    {
        encoder->dma_error_count++;
        encoder->pipeline_primed = 0U;
        as5047p_transaction_end(encoder);
        return;
    }

    response = encoder->rx_word;
    as5047p_transaction_end(encoder);
    as5047p_response_process(encoder, response, NULL);
}

static void as5047p_response_process(as5047p_t *encoder, uint16_t response,
                                     as5047p_sample_t *sample)
{
    uint32_t sample_cycle;
#if (AS5047P_ERRFL_AUTO_READ_ENABLE != 0U)
    uint8_t response_is_errfl;
#endif

    /* Diagnostics only: never reject an angle based on these flags. */
    encoder->rx_response_word = response;
    encoder->rx_parity_bit = (uint8_t)((response >> 15U) & 1U);
    encoder->rx_ef_flag = (uint8_t)((response & AS5047P_ERROR_BIT) != 0U);
    encoder->rx_parity_error = as5047p_parity_is_odd(response);

    /*
     * One-frame continuous pipeline: normally each FOC period sends ANGLECOM.
     * After EF is observed, one ERRFL read is inserted. RX always belongs to
     * the preceding transaction, so only the first completed frame is dropped.
     * CS remains high between periods.
     */
    if (encoder->pipeline_primed == 0U)
    {
        encoder->pipeline_primed = 1U;
        encoder->response_is_errfl_pending = encoder->tx_is_errfl;
        encoder->angle_request_cycle = encoder->transfer_start_cycle;
        return;
    }
    encoder->parity_error_count += encoder->rx_parity_error;
    encoder->sensor_error_count += encoder->rx_ef_flag;

#if (AS5047P_ERRFL_AUTO_READ_ENABLE != 0U)
    response_is_errfl = encoder->response_is_errfl_pending;
    encoder->response_is_errfl_pending = encoder->tx_is_errfl;
    if (response_is_errfl != 0U)
    {
        /* Register data must not be published as an angle. */
        encoder->errfl_response_word = response;
        encoder->errfl_status = response & AS5047P_DATA_MASK;
        encoder->errfl_read_count++;
        encoder->errfl_parity_error =
            (uint8_t)((encoder->errfl_status & AS5047P_ERRFL_PARERR_BIT) != 0U);
        encoder->errfl_invalid_command =
            (uint8_t)((encoder->errfl_status & AS5047P_ERRFL_INVCOMM_BIT) != 0U);
        encoder->errfl_frame_error =
            (uint8_t)((encoder->errfl_status & AS5047P_ERRFL_FRERR_BIT) != 0U);
        /* Means a detail response arrived, not that parity passed. */
        encoder->errfl_valid = 1U;
        encoder->errfl_read_in_progress = 0U;
        encoder->angle_request_cycle = encoder->transfer_start_cycle;
        return;
    }

    if (((encoder->rx_parity_error != 0U) || (encoder->rx_ef_flag != 0U)) &&
        (encoder->errfl_read_pending == 0U) &&
        (encoder->errfl_read_in_progress == 0U))
    {
        encoder->errfl_valid = 0U;
        encoder->errfl_read_pending = 1U;
    }
#endif

    /* This response belongs to the command sent in the preceding FOC period. */
    sample_cycle = encoder->angle_request_cycle;
    encoder->angle_request_cycle = encoder->transfer_start_cycle;

    /* RX contains the angle requested in the preceding SPI transaction. */
    as5047p_publish(encoder, response & AS5047P_DATA_MASK,
                    sample_cycle,
                    sample);
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

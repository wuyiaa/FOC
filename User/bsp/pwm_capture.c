#include "pwm_capture.h"

#define PWM_CAPTURE_TIMEOUT_MS  10U

#if ((PWM_CAPTURE_SAMPLE_COUNT == 0U) || (PWM_CAPTURE_SAMPLE_COUNT > 65535U))
#error "PWM capture must fit one normal-mode DMA transfer"
#endif

volatile pwm_capture_t pwm_capture;

/* Ordinary SRAM (.bss), not CCM. DMA writes; CPU reads only after completion. */
static volatile uint16_t pwm_capture_samples[PWM_CAPTURE_SAMPLE_COUNT];
static DMA_HandleTypeDef pwm_capture_dma;
static uint32_t pwm_capture_start_tick;
static uint8_t pwm_capture_initialized;

static void pwm_capture_stop(void)
{
    if (pwm_capture_initialized != 0U)
    {
        CLEAR_BIT(TIM6->DIER, TIM_DIER_UDE);
        CLEAR_BIT(TIM6->CR1, TIM_CR1_CEN);
        if (pwm_capture_dma.State == HAL_DMA_STATE_BUSY)
        {
            /* Also releases the HAL lock after a completed polled transfer. */
            (void)HAL_DMA_Abort(&pwm_capture_dma);
        }
        __DMB();
    }
}

static HAL_StatusTypeDef pwm_capture_fail(pwm_capture_error_t error)
{
    pwm_capture_stop();
    pwm_capture.error = error;
    pwm_capture.state = PWM_CAPTURE_ERROR;
    return HAL_ERROR;
}

static HAL_StatusTypeDef pwm_capture_init(void)
{
    uint32_t timer_clock;
    uint32_t timer_divisor;

    timer_clock = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_HCLK_DIV1)
    {
        timer_clock *= 2U;
    }
    timer_divisor = timer_clock / PWM_CAPTURE_SAMPLE_RATE_HZ;
    if ((timer_divisor == 0U) || (timer_divisor > 65536U) ||
        ((timer_clock % PWM_CAPTURE_SAMPLE_RATE_HZ) != 0U))
    {
        return pwm_capture_fail(PWM_CAPTURE_ERROR_CLOCK);
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_TIM6_CLK_ENABLE();

    /* Do not take over a running peripheral if future code reuses it. */
    if (((TIM6->CR1 & TIM_CR1_CEN) != 0U) ||
        ((DMA2_Channel1->CCR & DMA_CCR_EN) != 0U))
    {
        return pwm_capture_fail(PWM_CAPTURE_ERROR_RESOURCE_BUSY);
    }

    pwm_capture_dma.Instance = DMA2_Channel1;
    pwm_capture_dma.Init.Request = DMA_REQUEST_TIM6_UP;
    pwm_capture_dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    pwm_capture_dma.Init.PeriphInc = DMA_PINC_DISABLE;
    pwm_capture_dma.Init.MemInc = DMA_MINC_ENABLE;
    pwm_capture_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    pwm_capture_dma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    pwm_capture_dma.Init.Mode = DMA_NORMAL;
    pwm_capture_dma.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&pwm_capture_dma) != HAL_OK)
    {
        return pwm_capture_fail(PWM_CAPTURE_ERROR_DMA_INIT);
    }
    __HAL_DMA_DISABLE_IT(&pwm_capture_dma, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);

    /* No timer IRQ, no DMA IRQ, no GPIO reconfiguration, no TIM1 changes. */
    TIM6->CR1 = TIM_CR1_URS;
    TIM6->CR2 = 0U;
    TIM6->DIER = 0U;
    TIM6->PSC = 0U;
    TIM6->ARR = timer_divisor - 1U;
    TIM6->CNT = 0U;
    TIM6->EGR = TIM_EGR_UG;
    TIM6->SR = 0U;

    pwm_capture.sample_period_us =
        1000000.0f / (float)PWM_CAPTURE_SAMPLE_RATE_HZ;
    pwm_capture_initialized = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef pwm_capture_start(void)
{
    if (pwm_capture.state == PWM_CAPTURE_RUNNING)
    {
        return HAL_BUSY;
    }
    if ((pwm_capture_initialized == 0U) &&
        (pwm_capture_init() != HAL_OK))
    {
        return HAL_ERROR;
    }

    pwm_capture_stop();
    TIM6->CNT = 0U;
    TIM6->EGR = TIM_EGR_UG;
    TIM6->SR = 0U;
    pwm_capture.error = PWM_CAPTURE_ERROR_NONE;

    /* The request comes from TIM6; the source address is GPIOB, not TIM6.
       One halfword read captures PB13/PB14/PB15 together, including AF pins. */
    if (HAL_DMA_Start(&pwm_capture_dma, (uint32_t)&GPIOB->IDR,
                      (uint32_t)pwm_capture_samples,
                      PWM_CAPTURE_SAMPLE_COUNT) != HAL_OK)
    {
        return pwm_capture_fail(PWM_CAPTURE_ERROR_DMA_START);
    }

    pwm_capture_start_tick = HAL_GetTick();
    pwm_capture.state = PWM_CAPTURE_RUNNING;
    __DMB();
    SET_BIT(TIM6->DIER, TIM_DIER_UDE);
    SET_BIT(TIM6->CR1, TIM_CR1_CEN);
    return HAL_OK;
}

void pwm_capture_process(void)
{
    if (pwm_capture.state != PWM_CAPTURE_RUNNING)
    {
        return;
    }

    if (__HAL_DMA_GET_FLAG(&pwm_capture_dma,
                          __HAL_DMA_GET_TE_FLAG_INDEX(&pwm_capture_dma)) != 0U)
    {
        (void)pwm_capture_fail(PWM_CAPTURE_ERROR_DMA_TRANSFER);
    }
    else if (__HAL_DMA_GET_FLAG(&pwm_capture_dma,
                               __HAL_DMA_GET_TC_FLAG_INDEX(&pwm_capture_dma)) != 0U)
    {
        /* DMA_NORMAL has already stopped writing. Freeze the record for USB. */
        pwm_capture_stop();
        pwm_capture.completed_count++;
        pwm_capture.state = PWM_CAPTURE_READY;
    }
    else if ((uint32_t)(HAL_GetTick() - pwm_capture_start_tick) >=
             PWM_CAPTURE_TIMEOUT_MS)
    {
        (void)pwm_capture_fail(PWM_CAPTURE_ERROR_TIMEOUT);
    }
}

void pwm_capture_cancel(void)
{
    pwm_capture_stop();
    pwm_capture.state = PWM_CAPTURE_IDLE;
    pwm_capture.error = PWM_CAPTURE_ERROR_NONE;
}

uint16_t pwm_capture_sample_get(uint32_t index)
{
    if ((pwm_capture.state != PWM_CAPTURE_READY) ||
        (index >= PWM_CAPTURE_SAMPLE_COUNT))
    {
        return 0U;
    }
    return pwm_capture_samples[index];
}

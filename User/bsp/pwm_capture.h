#ifndef USER_BSP_PWM_CAPTURE_H
#define USER_BSP_PWM_CAPTURE_H

#include "stm32g4xx_hal.h"

/* Dedicated debug resources: TIM6 update -> DMA2_Channel1 -> GPIOB IDR.
   Keep TIM6 and DMA2_Channel1 reserved when editing the CubeMX project.
   Initialization belongs to this module, not the generated Core files. */
#define PWM_CAPTURE_SAMPLE_RATE_HZ  4000000UL
#define PWM_CAPTURE_SAMPLE_COUNT    4000U

typedef enum
{
    PWM_CAPTURE_IDLE = 0,
    PWM_CAPTURE_RUNNING = 1,
    PWM_CAPTURE_READY = 2,
    PWM_CAPTURE_ERROR = 3
} pwm_capture_state_t;

typedef enum
{
    PWM_CAPTURE_ERROR_NONE = 0,
    PWM_CAPTURE_ERROR_RESOURCE_BUSY = 1,
    PWM_CAPTURE_ERROR_CLOCK = 2,
    PWM_CAPTURE_ERROR_DMA_INIT = 3,
    PWM_CAPTURE_ERROR_DMA_START = 4,
    PWM_CAPTURE_ERROR_DMA_TRANSFER = 5,
    PWM_CAPTURE_ERROR_TIMEOUT = 6
} pwm_capture_error_t;

typedef struct
{
    pwm_capture_state_t state;
    pwm_capture_error_t error;
    float sample_period_us;
    uint32_t completed_count;
} pwm_capture_t;

/* Keil Watch diagnostics. Only vofa_pwm_capture_request is a capture command. */
extern volatile pwm_capture_t pwm_capture;

/* All functions are foreground-only. Start returns immediately. */
HAL_StatusTypeDef pwm_capture_start(void);
void pwm_capture_process(void);
void pwm_capture_cancel(void);
/* Raw GPIOB bits; valid only while state == PWM_CAPTURE_READY. */
uint16_t pwm_capture_sample_get(uint32_t index);

#endif /* USER_BSP_PWM_CAPTURE_H */

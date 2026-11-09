#include "key_control.h"

#include "common.h"
#include "B_J_Tf_Identification.h"

#define KEY1_DEBOUNCE_TIME_MS  20U

void key_control_process(void)
{
  key1_process();
}

static void key1_process(void)
{
  static GPIO_PinState raw_state;
  static GPIO_PinState stable_state;
  static uint32_t last_change_tick;
  static uint8_t initialized;
  GPIO_PinState current_state = HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);
  uint32_t now = HAL_GetTick();

  /* Use the power-up level as the initial state; a held key won't toggle. */
  if (initialized == 0U)
  {
    raw_state = current_state;
    stable_state = current_state;
    last_change_tick = now;
    initialized = 1U;
    return;
  }

  if (current_state != raw_state)
  {
    raw_state = current_state;
    last_change_tick = now;
  }
  else if ((raw_state != stable_state) &&
           ((uint32_t)(now - last_change_tick) >= KEY1_DEBOUNCE_TIME_MS))
  {
    stable_state = raw_state;
    if (stable_state == GPIO_PIN_RESET)
    {
      if (foc_pwm_output_is_enabled() != 0U)
      {
        foc_pwm_output_disable();
      }
      else
      {
        const uint32_t primask = __get_PRIMASK();

        /* Publish PWM enable and the new identification request together. */
        __disable_irq();
        /* A latched encoder/current fault keeps the PWM disabled. */
        if (foc_pwm_output_enable(&pm) == HAL_OK)
        {
#if (FOC_J_B_Tf_IDENTIFICATION_ENABLE != 0U)
          /* Start a fresh run even when parameters were restored from Flash.
             The first TIM7 step clears the previous observations/save latch. */
          motor_identification.stage = MOTOR_IDENT_IDLE;
          motor_identification_requested = 1U;
#endif
        }
        if (primask == 0U)
        {
          __enable_irq();
        }
      }
    }
  }
}



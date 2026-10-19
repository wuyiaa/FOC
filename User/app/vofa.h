#ifndef USER_APP_VOFA_H
#define USER_APP_VOFA_H

#include <stdint.h>

#define VOFA_MODE_TELEMETRY       0U
#define VOFA_MODE_PWM_CAPTURE     1U
#define VOFA_DEFAULT_MODE         VOFA_MODE_TELEMETRY

/* Keil Debug: 0 = original live channels, 1 = LIN1/LIN2/LIN3 replay.
   After entering capture mode/opening the port, first capture waits for PWM.
   auto_repeat = 1 repeats capture after each complete replay (default).
   auto_repeat = 0 finishes the current record, then waits for a request.
   Set request = 1 to queue another capture (also allowed with PWM off).
   CH6 = capture ID, CH7 = sample index; separate records have time gaps.
   VOFA point interval within one record: 0.00025 ms, NOT USB send interval. */
extern volatile uint8_t vofa_mode;
extern volatile uint8_t vofa_pwm_auto_repeat;
extern volatile uint8_t vofa_pwm_capture_request;
extern volatile uint32_t vofa_pwm_sent_samples;

void vofa_init(void);
void vofa_process(void);
void vofa_usb_host_request(void);
void vofa_usb_disconnected(void);

#endif /* USER_APP_VOFA_H */

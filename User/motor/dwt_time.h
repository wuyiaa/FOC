#ifndef USER_MOTOR_DWT_TIME_H
#define USER_MOTOR_DWT_TIME_H

#include <stdint.h>

void DWT_Init(void);
float DWT_GetDeltaT(uint32_t *cnt_last);

#endif /* USER_MOTOR_DWT_TIME_H */

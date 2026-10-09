#ifndef USER_BSP_DWT_TIME_H
#define USER_BSP_DWT_TIME_H

#include <stdint.h>

#define DWT_CPU_FREQ_HZ  160000000UL

void DWT_Init(void);
uint32_t DWT_GetElapsedCycles(uint32_t start_cycle);
float DWT_GetDeltaT(uint32_t *cnt_last);

#endif /* USER_BSP_DWT_TIME_H */

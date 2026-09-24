#include "main.h"
#include "dwt_time.h"

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t DWT_GetElapsedCycles(uint32_t start_cycle)
{
    return (uint32_t)(DWT->CYCCNT - start_cycle);
}

float DWT_GetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt = (float)((uint32_t)(cnt_now - *cnt_last)) /
               (float)DWT_CPU_FREQ_HZ;

    *cnt_last = cnt_now;

    return dt;
}

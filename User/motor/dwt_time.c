#include "main.h"
#include "dwt_time.h"

#define CPU_FREQ_HZ  160000000.0f

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

float DWT_GetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt = (float)((uint32_t)(cnt_now - *cnt_last)) / CPU_FREQ_HZ;

    *cnt_last = cnt_now;

    return dt;
}

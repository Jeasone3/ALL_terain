#include "Delay_us.h"

void Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; /* 使能 DWT 跟踪 */
    DWT->CYCCNT = 0;                                 /* 计数器清零 */
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;           /* 使能 CYCCNT 计数 */
}

void Delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U); /* 168MHz -> us*168 */
    while ((DWT->CYCCNT - start) < ticks)
    {
    }
}

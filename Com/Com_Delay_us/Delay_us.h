#ifndef __DELAY_US_H__
#define __DELAY_US_H__

#include "main.h"

void Delay_Init(void);      /* 使能 DWT CYCCNT，在时钟配置后调用一次 */
void Delay_us(uint32_t us); /* 阻塞微秒级延时 */

#endif /* __DELAY_US_H__ */

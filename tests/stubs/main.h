#ifndef TEST_MAIN_H
#define TEST_MAIN_H

#include <stdint.h>
#include <stddef.h>

/* 测试只替换硬件接口，状态机头文件和状态机实现使用工程原文件。 */
typedef struct {
    void *Instance;
} TIM_HandleTypeDef;

extern uintptr_t test_tim4_instance;
#define TIM4 ((void *)&test_tim4_instance)

uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);
void __set_PRIMASK(uint32_t value);

#endif

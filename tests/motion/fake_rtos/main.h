/** @brief 在原最小 HAL 上补充控制任务使用的 DWT 和定时器接口。 */
#ifndef TEST_RTOS_MAIN_H
#define TEST_RTOS_MAIN_H
#include "../fake_hal/main.h"
typedef struct { volatile uint32_t CYCCNT; } TestDwt;
extern TestDwt g_test_dwt;
extern uint32_t SystemCoreClock;
#define DWT (&g_test_dwt)
#define HAL_OK 0
#define TIM_FLAG_UPDATE 1u
#define EC_Pin 1u
#define MPU_INT_Pin 2u
int HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *timer);
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *timer);
void HAL_GPIO_EXTI_IRQHandler(uint16_t pin);
void HAL_IncTick(void);
void TestTimCounter(TIM_HandleTypeDef *timer, uint32_t value);
void TestTimClear(TIM_HandleTypeDef *timer, uint32_t flag);
void TestTimCompare(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t value);
#define __HAL_TIM_SET_COUNTER(timer, value) TestTimCounter(timer, value)
#define __HAL_TIM_CLEAR_FLAG(timer, flag) TestTimClear(timer, flag)
#define __HAL_TIM_SET_COMPARE(timer, channel, value) TestTimCompare(timer, channel, value)
#endif

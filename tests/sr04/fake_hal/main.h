#ifndef SR04_TEST_MAIN_H
#define SR04_TEST_MAIN_H

#include <stdint.h>

/* 仅提供 SR04 需要的 GPIO、DWT 和临界区接口，不模拟电机或定时器。 */
typedef struct
{
    uint32_t unused;
} GPIO_TypeDef;

typedef enum
{
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET
} GPIO_PinState;

typedef struct
{
    volatile uint32_t CYCCNT;
} DWT_Type;

extern GPIO_TypeDef g_test_gpioa;
extern DWT_Type g_test_dwt;
extern uint32_t SystemCoreClock;

#define EC_Pin        ((uint16_t)0x0010u)
#define TR_Pin        ((uint16_t)0x0020u)
#define MPU_INT_Pin   ((uint16_t)0x0080u)
#define EC_GPIO_Port  (&g_test_gpioa)
#define TR_GPIO_Port  (&g_test_gpioa)
#define DWT           (&g_test_dwt)

uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
uint32_t HAL_GetTick(void);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
void HAL_GPIO_EXTI_Callback(uint16_t pin);

#endif /* SR04_TEST_MAIN_H */

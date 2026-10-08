#ifndef OLED_TEST_MAIN_H
#define OLED_TEST_MAIN_H
#include <stdint.h>
/* 只替换 UI 所需 GPIO 和时钟声明，显示及日志逻辑使用真实源码。 */
typedef struct { unsigned unused; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
typedef struct { void *Instance; } TIM_HandleTypeDef;
#define LEFT_Pin (1u << 12)
#define RIGHT_Pin (1u << 10)
#define UP_Pin (1u << 8)
#define DOWN_Pin (1u << 9)
#define OK_Pin (1u << 11)
#define LEFT_GPIO_Port ((GPIO_TypeDef *)0)
#define RIGHT_GPIO_Port ((GPIO_TypeDef *)0)
#define UP_GPIO_Port ((GPIO_TypeDef *)0)
#define DOWN_GPIO_Port ((GPIO_TypeDef *)0)
#define OK_GPIO_Port ((GPIO_TypeDef *)0)
extern uint32_t SystemCoreClock;
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
#endif

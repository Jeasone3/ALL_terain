/**
 * @file main.h
 * @brief 宿主测试使用的最小 HAL 类型；不包含 STM32 外设实现。
 */
#ifndef MOTION_TEST_MAIN_H
#define MOTION_TEST_MAIN_H

#include <stdint.h>
#include <stddef.h>

typedef struct {
    void *Instance;
} TIM_HandleTypeDef;

typedef struct {
    uint32_t unused;
} GPIO_TypeDef;

#define TIM4 ((void *)(uintptr_t)4u)

/**
 * @brief 读取测试维护的中断屏蔽位。
 * @param 无。
 * @return 0 表示允许中断，1 表示屏蔽中断。
 * @note 用于检查生产代码进入临界区后是否恢复原来的屏蔽状态。
 */
uint32_t __get_PRIMASK(void);

/**
 * @brief 将测试维护的中断屏蔽位设置为 1。
 * @param 无。
 * @return 无。
 * @note 仅替代 CMSIS 内联操作，不在宿主进程中真正屏蔽中断。
 */
void __disable_irq(void);

/**
 * @brief 恢复测试维护的中断屏蔽位。
 * @param mask 进入临界区前的中断屏蔽状态，取 0 或 1。
 * @return 无。
 * @note 测试会确认原先已屏蔽的调用者仍保持屏蔽。
 */
void __set_PRIMASK(uint32_t mask);

/**
 * @brief 读取测试指定的开机时间，替代 HAL 毫秒计数器。
 * @param 无。
 * @return 当前模拟开机毫秒数，允许覆盖 UINT32_MAX 边界。
 * @note 测试可独立推进时间与控制帧，验证 10 ms 事件在 100 ms 刷新后仍然保留。
 */
uint32_t HAL_GetTick(void);

#endif

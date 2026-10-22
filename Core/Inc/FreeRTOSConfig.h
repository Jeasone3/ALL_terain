/**
 * @file FreeRTOSConfig.h
 * @brief STM32F103RC 三任务配置：1 ms 系统节拍、TIM4 通知和动态任务栈。
 * @note 配合 FreeRTOS V10.5.1 的 RVDS/ARM_CM3 移植层与 heap_4 使用。
 */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>
#include <stddef.h>

/* 致命错误只关闭 PWM 并保留文件、行号，不访问 OLED 或执行阻塞通信。 */
void AppRTOS_Fatal(const char *file, uint32_t line);

#define configUSE_PREEMPTION                     1
#define configCPU_CLOCK_HZ                       (72000000UL)
#define configTICK_RATE_HZ                       ((TickType_t)1000)
#define configMAX_PRIORITIES                     5
#define configMINIMAL_STACK_SIZE                 ((uint16_t)128)
#define configTOTAL_HEAP_SIZE                    ((size_t)(12 * 1024))
#define configMAX_TASK_NAME_LEN                  16
#define configUSE_16_BIT_TICKS                   0
#define configIDLE_SHOULD_YIELD                  1

/* 第一版只有三个应用任务和内核空闲任务，不创建软件定时器任务。 */
#define configSUPPORT_DYNAMIC_ALLOCATION         1
#define configSUPPORT_STATIC_ALLOCATION          0
#define configUSE_TASK_NOTIFICATIONS             1
#define configUSE_TIMERS                         0
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configUSE_TRACE_FACILITY                 0
#define configCHECK_FOR_STACK_OVERFLOW           2
#define configUSE_MALLOC_FAILED_HOOK             1

#define INCLUDE_vTaskDelay                       1
#define INCLUDE_xTaskDelayUntil                  1
#define INCLUDE_vTaskSuspend                     1
#define INCLUDE_xTaskGetSchedulerState           1
#define INCLUDE_uxTaskGetStackHighWaterMark       1

/* Cortex-M3 实现四位优先级；TIM4=5 可以调用 FromISR，内核异常使用最低优先级。 */
#define configPRIO_BITS                          4
#define configLIBRARY_KERNEL_INTERRUPT_PRIORITY  15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY \
    (configLIBRARY_KERNEL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* SysTick 保留独立组合入口，同时更新 HAL 时间和内核时间，不在这里映射。 */
#define vPortSVCHandler                          SVC_Handler
#define xPortPendSVHandler                       PendSV_Handler

#define configASSERT(condition) do { \
    if (!(condition)) { AppRTOS_Fatal(__FILE__, (uint32_t)__LINE__); } \
} while (0)

#endif

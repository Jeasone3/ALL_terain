/**
 * @file App_RTOS.h
 * @brief 三任务启动、TIM4 中断通知和调试器可观察的运行统计。
 * @note 运动算法不依赖本头文件；控制任务通过原 ModeFSM 接口执行整帧控制。
 */
#ifndef APP_RTOS_H
#define APP_RTOS_H

#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

/* 栈深度单位为 StackType_t，当前 Cortex-M3 上每个元素为 4 字节。 */
#define CONTROL_TASK_STACK_WORDS 512u
#define DISPLAY_TASK_STACK_WORDS 768u
#define RECOVERY_TASK_STACK_WORDS 384u
#define CONTROL_TASK_PRIORITY 3u
#define DISPLAY_TASK_PRIORITY 2u
#define RECOVERY_TASK_PRIORITY 1u
#define CONTROL_PERIOD_MS 10u

/** @brief 调试统计；耗时包含期间发生的中断，栈水位单位为剩余字数。 */
typedef struct {
    uint32_t control_frames;             /* 已处理的控制唤醒次数，包含故障唤醒 */
    uint32_t control_last_us;            /* 最近一帧 ModeFSM 执行耗时 */
    uint32_t control_max_us;             /* 开机以来最大单帧耗时 */
    uint32_t wake_interval_last_us;      /* 最近两次任务唤醒之间的时间 */
    uint32_t wake_interval_max_us;       /* 开机以来最大唤醒间隔 */
    uint32_t control_period_faults;      /* 通知积压或执行超期的次数 */
    UBaseType_t control_stack_free_words;/* 控制任务历史最小剩余栈 */
    UBaseType_t display_stack_free_words;/* 显示任务历史最小剩余栈 */
    UBaseType_t recovery_stack_free_words;/* 恢复任务历史最小剩余栈 */
    size_t free_heap_bytes;              /* 最近一次采样的 FreeRTOS 剩余堆 */
} AppRTOS_Stats;

extern volatile AppRTOS_Stats g_rtos_stats;
extern const char * volatile g_rtos_fatal_file;
extern volatile uint32_t g_rtos_fatal_line;

/**
 * @brief 创建三个任务并启动抢占调度器。
 * @param 无。
 * @return 正常不返回；任务创建或调度器启动失败进入应急停车。
 * @note 在电机、FSM、IMU 初始化和启动请求选择完成后调用一次，TIM4 此时尚未启动。
 */
void AppRTOS_Start(void);

/**
 * @brief 从 TIM4 中断通知控制任务，需要时立即请求切换。
 * @param 无。
 * @return 无。
 * @note 仅由 TIM4 周期回调调用；任务句柄无效或调度器未启动时不通知。
 */
void AppRTOS_ControlTickFromISR(void);

/**
 * @brief 低频采样任务栈水位和剩余堆，供调试器检查初始资源分配。
 * @param 无。
 * @return 无。
 * @note 仅由显示任务每秒调用，不在 10 ms 控制帧中扫描任务栈。
 */
void AppRTOS_SampleDiagnostics(void);

/**
 * @brief 内核致命错误时立即关闭两轮 PWM 并停止执行。
 * @param file 触发错误的文件或静态原因字符串。
 * @param line 文件行号；钩子没有源码位置时传 0。
 * @return 不返回。
 * @note 不依赖任务调度，不格式化字符串，不调用日志或 HAL 延时。
 */
void AppRTOS_Fatal(const char *file, uint32_t line);

/* 任务入口只交给 xTaskCreate，禁止从主循环或中断直接调用。 */
void ControlTask(void *argument);
void DisplayTask(void *argument);
void RecoveryTask(void *argument);

#endif

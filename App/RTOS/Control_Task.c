/**
 * @file Control_Task.c
 * @brief TIM4 驱动的唯一运动任务：等待一帧、执行一帧、检查控制周期。
 */
#include "App_RTOS.h"
#include "Mode_FSM.h"
#include "tim.h"

/**
 * @brief 启动 TIM4 并按通知执行完整的 10 ms 运动帧。
 * @param argument 未使用，任务创建时传 NULL。
 * @return 不返回。
 * @note DWT 已在 main 中初始化；差值使用无符号减法，可跨越 CYCCNT 回绕。
 *       同一任务内保持灰度、Route、FSM、IMU 和 PID 的原有先后关系。
 */
void ControlTask(void *argument)
{
    uint32_t notifications;
    uint32_t frame_start;
    uint32_t elapsed_cycles;
    uint32_t previous_wake = 0u;
    uint32_t wake_us;
    uint32_t cycles_per_us = SystemCoreClock / 1000000u;
    uint8_t have_previous_wake = 0u;
    (void)argument;

    /* 清掉初始化产生的更新标志，从完整的第一个 10 ms 周期开始控制。 */
    __HAL_TIM_SET_COUNTER(&htim4, 0u);
    __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
    if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK)
    {
        AppRTOS_Fatal("TIM4_START", 0u);
    }

    for (;;)
    {
        notifications = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        frame_start = DWT->CYCCNT;
        if (have_previous_wake != 0u)
        {
            wake_us = (frame_start - previous_wake) / cycles_per_us;
            g_rtos_stats.wake_interval_last_us = wake_us;
            if (wake_us > g_rtos_stats.wake_interval_max_us)
                g_rtos_stats.wake_interval_max_us = wake_us;
        }
        previous_wake = frame_start;
        have_previous_wake = 1u;
        ++g_rtos_stats.control_frames;

        /* 已错过节拍时立即故障停车，不能把积压通知逐个转换成快速连续 PID。 */
        if (notifications != 1u)
        {
            ++g_rtos_stats.control_period_faults;
            ModeFSM_ControlPeriodFault();
            continue;
        }

        ModeFSM_Tick();
        elapsed_cycles = DWT->CYCCNT - frame_start;
        g_rtos_stats.control_last_us = elapsed_cycles / cycles_per_us;
        if (g_rtos_stats.control_last_us > g_rtos_stats.control_max_us)
            g_rtos_stats.control_max_us = g_rtos_stats.control_last_us;

        /* 测量包括帧内被中断占用的时间；10 ms 截止后只停车，不补跑姿态积分。 */
        if (elapsed_cycles >= (SystemCoreClock / 1000u) * CONTROL_PERIOD_MS)
        {
            ++g_rtos_stats.control_period_faults;
            ModeFSM_ControlPeriodFault();
        }
    }
}

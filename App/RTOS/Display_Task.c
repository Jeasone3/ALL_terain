/**
 * @file Display_Task.c
 * @brief 独占 OLED 的低优先级显示任务，保留原日志快照和重连行为。
 */
#include "App_RTOS.h"
#include "Debug_Log.h"
#include "Int_OLED.h"
#include "main.h"

/* 通信状态仅由本任务更新，调试器可直接查看。 */
static volatile OLED_Status s_oled_status = OLED_NOT_INITIALIZED;

/**
 * @brief 初始化 OLED，随后每 100 ms 显示，失败时按 1 s 间隔重连。
 * @param argument 未使用，任务创建时传 NULL。
 * @return 不返回。
 * @note 格式化和 I2C 在临界区外执行；重连不清日志，不改变运动或路线状态。
 */
void DisplayTask(void *argument)
{
    TickType_t last_wake;
    uint32_t last_retry_ms;
    uint32_t last_diagnostics_ms;
    uint32_t now_ms;
    uint8_t oled_ready;
    (void)argument;

    s_oled_status = OLED_Init();
    oled_ready = (uint8_t)(s_oled_status == OLED_OK);
    last_retry_ms = HAL_GetTick();
    last_diagnostics_ms = last_retry_ms;
    last_wake = xTaskGetTickCount();

    for (;;)
    {
        now_ms = HAL_GetTick();
        if (oled_ready == 0u && (uint32_t)(now_ms - last_retry_ms) >= 1000u)
        {
            last_retry_ms = now_ms;
            s_oled_status = OLED_Init();
            oled_ready = (uint8_t)(s_oled_status == OLED_OK);
        }
        if (oled_ready != 0u)
        {
            s_oled_status = DebugLog_Display();
            if (s_oled_status != OLED_OK)
            {
                oled_ready = 0u;
                last_retry_ms = HAL_GetTick();
            }
        }
        if ((uint32_t)(now_ms - last_diagnostics_ms) >= 1000u)
        {
            last_diagnostics_ms = now_ms;
            AppRTOS_SampleDiagnostics();
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100u));
    }
}

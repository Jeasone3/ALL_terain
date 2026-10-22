/**
 * @file App_RTOS.c
 * @brief 创建三任务、转发控制通知并提供内核异常时的最小停车出口。
 */
#include "App_RTOS.h"
#include "main.h"
#include "tim.h"
#include "motor.h"

extern Motor_Struct motorLeft;
extern Motor_Struct motorRight;

static TaskHandle_t s_control_task;
static TaskHandle_t s_display_task;
static TaskHandle_t s_recovery_task;

volatile AppRTOS_Stats g_rtos_stats;
const char * volatile g_rtos_fatal_file;
volatile uint32_t g_rtos_fatal_line;

/**
 * @brief 先创建所有任务再启动调度，避免 TIM4 在句柄准备完成前发出通知。
 * @param 无。
 * @return 正常不返回，创建失败或调度器意外返回时应急停车。
 * @note 本函数调用前电机已经初始化为零输出，调度前不会执行任何任务入口。
 */
void AppRTOS_Start(void)
{
    if (xTaskCreate(ControlTask, "Control", CONTROL_TASK_STACK_WORDS, NULL,
                    CONTROL_TASK_PRIORITY, &s_control_task) != pdPASS ||
        xTaskCreate(DisplayTask, "Display", DISPLAY_TASK_STACK_WORDS, NULL,
                    DISPLAY_TASK_PRIORITY, &s_display_task) != pdPASS ||
        xTaskCreate(RecoveryTask, "Recovery", RECOVERY_TASK_STACK_WORDS, NULL,
                    RECOVERY_TASK_PRIORITY, &s_recovery_task) != pdPASS)
    {
        AppRTOS_Fatal("TASK_CREATE", 0u);
    }

    vTaskStartScheduler();
    AppRTOS_Fatal("SCHED_START", 0u);
}

/**
 * @brief 保存每一次 TIM4 到期，并在控制任务就绪时请求及时切换。
 * @param 无。
 * @return 无。
 * @note 只从 TIM4 中断调用；积压判断交给控制任务，不在中断中处理运动状态。
 */
void AppRTOS_ControlTickFromISR(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (s_control_task != NULL &&
        xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        vTaskNotifyGiveFromISR(s_control_task, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

/**
 * @brief 在低优先级任务中检查历史最小剩余栈和剩余堆。
 * @param 无。
 * @return 无，结果保存在 g_rtos_stats，栈单位为字，堆单位为字节。
 * @note 任务创建完成后由显示任务每秒调用；这些调试值不参与运动判断。
 */
void AppRTOS_SampleDiagnostics(void)
{
    g_rtos_stats.control_stack_free_words = uxTaskGetStackHighWaterMark(s_control_task);
    g_rtos_stats.display_stack_free_words = uxTaskGetStackHighWaterMark(s_display_task);
    g_rtos_stats.recovery_stack_free_words = uxTaskGetStackHighWaterMark(s_recovery_task);
    g_rtos_stats.free_heap_bytes = xPortGetFreeHeapSize();
}

/**
 * @brief 直接写比较寄存器关闭 PWM，不继续使用可能损坏的内核状态。
 * @param file 静态错误原因或触发断言的源码路径。
 * @param line 对应行号，没有行号时为 0。
 * @return 不返回，停在循环中以便调试器读取原因。
 * @note 不调用 PID、动态内存、日志、HAL_Delay 或任何阻塞通信。
 */
void AppRTOS_Fatal(const char *file, uint32_t line)
{
    __disable_irq();
    __HAL_TIM_SET_COMPARE(motorLeft.htim, motorLeft.channel, 0u);
    __HAL_TIM_SET_COMPARE(motorRight.htim, motorRight.channel, 0u);
    motorLeft.speed = 0;
    motorRight.speed = 0;
    g_rtos_fatal_file = file;
    g_rtos_fatal_line = line;
    for (;;) { }
}

/**
 * @brief 处理 FreeRTOS 堆分配失败。
 * @note 钩子可能在调度器启动前执行，只做硬件停车并保留静态原因。
 */
void vApplicationMallocFailedHook(void)
{
    AppRTOS_Fatal("MALLOC", 0u);
}

/**
 * @brief 处理任务栈溢出，禁止继续使用该任务执行 OLED 或日志格式化。
 * @param task 内核报告的任务句柄，不在错误出口进一步访问。
 * @param task_name 内核报告的任务名称，不依赖其存储仍然有效。
 */
void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void)task;
    (void)task_name;
    AppRTOS_Fatal("STACK", 0u);
}

/**
 * @brief HAL 定时器统一回调，只把 TIM4 到期转发为任务通知。
 * @param timer 本次回调的定时器；空指针与其他定时器均忽略。
 * @note 中断中不采样传感器、不执行 ModeFSM_Tick、不输出电机 PWM。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer)
{
    if (timer != NULL && timer->Instance == TIM4)
    {
        AppRTOS_ControlTickFromISR();
    }
}

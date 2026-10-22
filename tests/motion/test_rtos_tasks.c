/**
 * @file test_rtos_tasks.c
 * @brief 编译真实三个任务、启动模块和异常入口，验证通知、超期与重连行为。
 * @note setjmp 仅用于结束生产代码的永久任务循环；不替代板上的调度与时序验证。
 */
#include "App_RTOS.h"
#include "Mode_FSM.h"
#include "Int_MPU6050.h"
#include "Debug_Log.h"
#include "motor.h"
#include "tim.h"
#include "stm32f1xx_it.h"
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL RTOS line %d: %s\n", __LINE__, #condition); exit(1); \
} } while (0)

TestDwt g_test_dwt;
uint32_t SystemCoreClock = 72000000u;
TIM_HandleTypeDef htim4 = {TIM4};
Motor_Struct motorLeft;
Motor_Struct motorRight;
volatile uint8_t g_imu_ready;
Gyro_Accel_Struct g_imu_data;
static jmp_buf s_loop_exit;
static unsigned s_create_count, s_notify_count, s_yield_count;
static BaseType_t s_scheduler, s_last_yield;
static TaskHandle_t s_notified_task;
static unsigned s_mode_frames, s_faults, s_timer_starts, s_timer_reset, s_timer_clear;
static unsigned s_tick_calls, s_kernel_ticks;
static uint32_t s_now_ms, s_irq_mask;
static unsigned s_wait_count, s_event_count, s_delay_count, s_delay_limit;
static unsigned s_oled_inits, s_displays, s_recovery_inits;
static unsigned s_diagnostic_reads;
static uint8_t s_oled_init_fail, s_display_fail, s_fail_after_recovery;
typedef struct { uint32_t count, wake_cycles, work_cycles; } ControlEvent;
static ControlEvent s_events[4];

/* 内核替身记录创建参数，任务仍使用项目中的真实入口函数。 */
BaseType_t xTaskCreate(TaskFunction_t function, const char *name, uint16_t stack,
                       void *argument, UBaseType_t priority, TaskHandle_t *handle)
{
    static TaskFunction_t const expected[] = {ControlTask, DisplayTask, RecoveryTask};
    static const uint16_t stacks[] = {512u, 768u, 384u};
    static const UBaseType_t priorities[] = {3u, 2u, 1u};
    CHECK(s_create_count < 3u && function == expected[s_create_count]);
    CHECK(name != NULL && argument == NULL);
    CHECK(stack == stacks[s_create_count] && priority == priorities[s_create_count]);
    *handle = (TaskHandle_t)(uintptr_t)(++s_create_count);
    return pdPASS;
}
void vTaskStartScheduler(void) { s_scheduler = taskSCHEDULER_RUNNING; longjmp(s_loop_exit, 1); }
BaseType_t xTaskGetSchedulerState(void) { return s_scheduler; }
void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *woken)
{
    ++s_notify_count;
    s_notified_task = task;
    *woken = pdTRUE;
}
void TestYieldFromISR(BaseType_t woken) { ++s_yield_count; s_last_yield = woken; }
uint32_t ulTaskNotifyTake(BaseType_t clear_on_exit, TickType_t timeout)
{
    CHECK(clear_on_exit == pdTRUE && timeout == portMAX_DELAY);
    if (s_wait_count >= s_event_count) longjmp(s_loop_exit, 1);
    DWT->CYCCNT = s_events[s_wait_count].wake_cycles;
    return s_events[s_wait_count++].count;
}
TickType_t xTaskGetTickCount(void) { return s_now_ms; }
void vTaskDelayUntil(TickType_t *last_wake, TickType_t period)
{
    *last_wake += period;
    s_now_ms = *last_wake;
    if (++s_delay_count >= s_delay_limit) longjmp(s_loop_exit, 1);
}
UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t task)
{
    CHECK(task != NULL);
    ++s_diagnostic_reads;
    return 100u + (UBaseType_t)(uintptr_t)task;
}
size_t xPortGetFreeHeapSize(void) { return 4096u; }

/* HAL 与业务替身只记录副作用；实际回调、任务循环和截止判断均来自生产代码。 */
uint32_t __get_PRIMASK(void) { return s_irq_mask; }
void __disable_irq(void) { s_irq_mask = 1u; }
void __set_PRIMASK(uint32_t mask) { s_irq_mask = mask; }
uint32_t HAL_GetTick(void) { return s_now_ms; }
void HAL_IncTick(void) { ++s_tick_calls; ++s_now_ms; }
void xPortSysTickHandler(void) { ++s_kernel_ticks; }
void HAL_GPIO_EXTI_IRQHandler(uint16_t pin) { (void)pin; }
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer);
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *timer) { HAL_TIM_PeriodElapsedCallback(timer); }
void TestTimCounter(TIM_HandleTypeDef *timer, uint32_t value)
{ CHECK(timer == &htim4 && value == 0u); ++s_timer_reset; }
void TestTimClear(TIM_HandleTypeDef *timer, uint32_t flag)
{ CHECK(timer == &htim4 && flag == TIM_FLAG_UPDATE); ++s_timer_clear; }
void TestTimCompare(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t value)
{ (void)timer; (void)channel; CHECK(value == 0u); }
int HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *timer)
{ CHECK(timer == &htim4 && s_timer_reset && s_timer_clear); ++s_timer_starts; return HAL_OK; }
void ModeFSM_Tick(void)
{ ++s_mode_frames; DWT->CYCCNT += s_events[s_wait_count - 1u].work_cycles; }
void ModeFSM_ControlPeriodFault(void) { ++s_faults; }
OLED_Status OLED_Init(void)
{ ++s_oled_inits; return (s_oled_init_fail && s_oled_inits == 1u) ? OLED_IO_ERROR : OLED_OK; }
OLED_Status DebugLog_Display(void)
{ ++s_displays; return (s_display_fail && s_displays == 1u) ? OLED_IO_ERROR : OLED_OK; }
uint8_t Int_MPU6050_Init(void)
{
    CHECK(g_imu_ready == 0u);
    ++s_recovery_inits;
    g_imu_ready = 1u;
    /* 模拟初始化完成后被控制任务抢占，新的读取已经失败，旧成功值不能写回来。 */
    if (s_fail_after_recovery) g_imu_ready = 0u;
    return 1u;
}

/** @brief 清空场景计数，不清除真实启动模块已经创建的任务句柄。 */
static void reset_case(void)
{
    memset((void *)&g_rtos_stats, 0, sizeof(g_rtos_stats));
    memset(s_events, 0, sizeof(s_events));
    s_notify_count = s_yield_count = s_mode_frames = s_faults = 0u;
    s_timer_starts = s_timer_reset = s_timer_clear = 0u;
    s_wait_count = s_event_count = s_delay_count = s_delay_limit = 0u;
    s_oled_inits = s_displays = s_recovery_inits = s_diagnostic_reads = 0u;
    s_oled_init_fail = s_display_fail = s_fail_after_recovery = 0u;
    s_now_ms = 0u;
}

/** @brief 检查任务创建参数、调度前保护、非 TIM4 过滤和中断只通知不执行控制。 */
static void test_start_and_interrupts(void)
{
    TIM_HandleTypeDef other = {(void *)(uintptr_t)3u};
    reset_case();
    AppRTOS_ControlTickFromISR();
    CHECK(s_notify_count == 0u);
    if (setjmp(s_loop_exit) == 0) AppRTOS_Start();
    CHECK(s_create_count == 3u);
    s_scheduler = taskSCHEDULER_NOT_STARTED;
    HAL_TIM_PeriodElapsedCallback(&htim4);
    CHECK(s_notify_count == 0u);
    s_scheduler = taskSCHEDULER_RUNNING;
    HAL_TIM_PeriodElapsedCallback(NULL);
    HAL_TIM_PeriodElapsedCallback(&other);
    CHECK(s_notify_count == 0u);
    TIM4_IRQHandler();
    CHECK(s_notify_count == 1u && s_notified_task == (void *)(uintptr_t)1u);
    CHECK(s_yield_count == 1u && s_last_yield == pdTRUE);
    CHECK(s_mode_frames == 0u);
    s_tick_calls = s_kernel_ticks = 0u;
    s_scheduler = taskSCHEDULER_NOT_STARTED;
    SysTick_Handler();
    CHECK(s_tick_calls == 1u && s_kernel_ticks == 0u);
    s_scheduler = taskSCHEDULER_RUNNING;
    SysTick_Handler();
    CHECK(s_tick_calls == 2u && s_kernel_ticks == 1u);
    puts("PASS task creation, TIM4 notification-only entry and HAL/RTOS tick gating");
}

/** @brief 检查正常控制、积压不补跑、恰好 10 ms 超期及 DWT 回绕。 */
static void test_control_deadline(void)
{
    reset_case();
    s_event_count = 3u;
    s_events[0] = (ControlEvent){1u, 720000u, 14400u};
    s_events[1] = (ControlEvent){3u, 2880000u, 0u};
    s_events[2] = (ControlEvent){1u, 3600000u, 720000u};
    if (setjmp(s_loop_exit) == 0) ControlTask(NULL);
    CHECK(s_timer_starts == 1u && s_mode_frames == 2u && s_faults == 2u);
    CHECK(g_rtos_stats.control_frames == 3u && g_rtos_stats.control_period_faults == 2u);
    CHECK(g_rtos_stats.control_last_us == 10000u && g_rtos_stats.control_max_us == 10000u);
    reset_case();
    s_event_count = 1u;
    s_events[0] = (ControlEvent){1u, UINT32_MAX - 100u, 14400u};
    if (setjmp(s_loop_exit) == 0) ControlTask(NULL);
    CHECK(s_faults == 0u && g_rtos_stats.control_last_us == 200u);
    puts("PASS one frame per notification, backlog stop, exact deadline and DWT wrap");
}

/** @brief 检查显示失败重连和采样资源水位时不会执行运动算法。 */
static void test_display_reconnect(void)
{
    reset_case();
    s_oled_init_fail = 1u;
    s_delay_limit = 12u;
    if (setjmp(s_loop_exit) == 0) DisplayTask(NULL);
    CHECK(s_oled_inits == 2u && s_displays == 2u && s_now_ms == 1200u);
    CHECK(s_diagnostic_reads == 3u && g_rtos_stats.free_heap_bytes == 4096u);
    CHECK(s_mode_frames == 0u && s_recovery_inits == 0u);
    reset_case();
    s_display_fail = 1u;
    s_delay_limit = 12u;
    if (setjmp(s_loop_exit) == 0) DisplayTask(NULL);
    CHECK(s_oled_inits == 2u && s_displays == 3u);
    puts("PASS OLED init/refresh failure retries after one second and low-priority diagnostics");
}

/** @brief 检查只重试失联 IMU，且不会把返回的过期成功状态覆盖新的失败。 */
static void test_recovery_publication(void)
{
    reset_case();
    g_imu_ready = 1u;
    s_delay_limit = 3u;
    if (setjmp(s_loop_exit) == 0) RecoveryTask(NULL);
    CHECK(s_recovery_inits == 0u);
    reset_case();
    g_imu_ready = 0u;
    s_delay_limit = 2u;
    s_fail_after_recovery = 1u;
    if (setjmp(s_loop_exit) == 0) RecoveryTask(NULL);
    CHECK(s_recovery_inits == 1u && g_imu_ready == 0u);
    CHECK(s_mode_frames == 0u && s_displays == 0u);
    puts("PASS ready-only retry and no stale recovery success writeback");
}

int main(void)
{
    test_start_and_interrupts();
    test_control_deadline();
    test_display_reconnect();
    test_recovery_publication();
    puts("All RTOS task boundary scenarios passed.");
    return 0;
}

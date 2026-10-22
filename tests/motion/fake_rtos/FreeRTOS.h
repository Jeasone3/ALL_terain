/** @brief 任务宿主测试的最小内核接口；不模拟 Cortex-M 上下文切换。 */
#ifndef TEST_FREERTOS_H
#define TEST_FREERTOS_H
#include <stdint.h>
#include <stddef.h>
typedef int32_t BaseType_t;
typedef uint32_t UBaseType_t;
typedef uint32_t TickType_t;
typedef uint32_t StackType_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
void TestYieldFromISR(BaseType_t higher_priority_task_woken);
#define portYIELD_FROM_ISR(woken) TestYieldFromISR(woken)
size_t xPortGetFreeHeapSize(void);
#endif

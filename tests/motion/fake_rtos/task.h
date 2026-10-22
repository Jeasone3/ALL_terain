/** @brief 通过替身驱动真实任务循环，检查顺序和边界，不宣称验证内核调度。 */
#ifndef TEST_TASK_H
#define TEST_TASK_H
#include "FreeRTOS.h"
typedef void *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);
#define taskSCHEDULER_NOT_STARTED 0
#define taskSCHEDULER_RUNNING 1
#define taskSCHEDULER_SUSPENDED 2
BaseType_t xTaskCreate(TaskFunction_t function, const char *name, uint16_t stack,
                       void *argument, UBaseType_t priority, TaskHandle_t *handle);
void vTaskStartScheduler(void);
BaseType_t xTaskGetSchedulerState(void);
void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *woken);
uint32_t ulTaskNotifyTake(BaseType_t clear_on_exit, TickType_t timeout);
TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *last_wake, TickType_t period);
UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t task);
#endif

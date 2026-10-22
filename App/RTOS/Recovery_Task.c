/**
 * @file Recovery_Task.c
 * @brief 每秒重试失联的 MPU6050，只恢复通信，不恢复运动。
 */
#include "App_RTOS.h"
#include "Int_MPU6050.h"

/**
 * @brief 在最低应用优先级下重试 IMU 初始化。
 * @param argument 未使用，任务创建时传 NULL。
 * @return 不返回。
 * @note 初始化期间 ready 始终为 0，控制任务不会同时访问软件 I2C；所有事务
 *       结束后驱动才发布 ready=1。不能再次把函数返回值写回共享标志，否则可能
 *       覆盖控制任务抢占后报告的新读取失败。复位等待仍允许控制任务抢占。
 */
void RecoveryTask(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();
    (void)argument;

    for (;;)
    {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000u));
        if (g_imu_ready == 0u)
        {
            (void)Int_MPU6050_Init();
        }
    }
}

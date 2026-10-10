/**
 * @file test_attitude.c
 * @brief 直接编译真实 Attitude.c，验证复位输出与后续姿态基准同步。
 * @note 六轴原始输入由本文件提供，Mahony 解算使用生产代码。
 */
#include "Attitude.h"
#include "Int_MPU6050.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL attitude line %d: %s\n", __LINE__, #condition); \
        exit(EXIT_FAILURE); \
    } \
} while (0)

Gyro_Accel_Struct g_imu_data;
uint8_t g_imu_ready;

/**
 * @brief 检查真实姿态复位与输出偏置的即时和下一帧行为。
 * @param 无。
 * @return EXIT_SUCCESS 表示场景全部通过；失败由 CHECK 返回非零退出码。
 * @note 输入为水平静止重力和 100 度/秒的 Z 轴角速度，每 10 ms 约转 1 度。
 */
int main(void)
{
    g_imu_ready = 1u;
    g_imu_data.accel.accel_z = 16384;
    g_imu_data.gyro.gyro_z = 1640;
    Attitude_Init();
    CHECK(g_euler.yaw == 0.0f && g_euler.pitch == 0.0f && g_euler.roll == 0.0f);
    Attitude_Tick();
    CHECK(g_euler.yaw > 0.99f && g_euler.yaw < 1.01f);
    Attitude_Tick();
    CHECK(g_euler.yaw > 1.99f && g_euler.yaw < 2.01f);

    /* 复位必须立即清除旧反馈；ready=0 不允许把旧姿态重新写回来。 */
    g_euler.pitch = 14.0f;
    g_euler.roll = -15.0f;
    Attitude_Reset();
    CHECK(g_euler.yaw == 0.0f && g_euler.pitch == 0.0f && g_euler.roll == 0.0f);
    g_imu_ready = 0u;
    Attitude_Tick();
    CHECK(g_euler.yaw == 0.0f && g_euler.pitch == 0.0f && g_euler.roll == 0.0f);
    g_imu_ready = 1u;
    Attitude_Tick();
    CHECK(g_euler.yaw > 0.99f && g_euler.yaw < 1.01f);

    /* 设置偏置会重建四元数，因此新一帧必须从 70 度附近继续，而非沿用旧 raw yaw。 */
    Attitude_SetYawOffset(70.0f);
    CHECK(g_euler.yaw == 70.0f && g_euler.pitch == 0.0f && g_euler.roll == 0.0f);
    Attitude_Tick();
    CHECK(g_euler.yaw > 70.99f && g_euler.yaw < 71.01f);
    Attitude_SetYawOffset(-45.0f);
    CHECK(g_euler.yaw == -45.0f && g_euler.pitch == 0.0f && g_euler.roll == 0.0f);
    Attitude_Tick();
    CHECK(g_euler.yaw > -44.01f && g_euler.yaw < -43.99f);
    Attitude_Init();
    CHECK(g_euler.yaw == 0.0f && g_euler.pitch == 0.0f && g_euler.roll == 0.0f);
    puts("PASS real Attitude reset, ready gating and yaw-offset continuation");
    return EXIT_SUCCESS;
}

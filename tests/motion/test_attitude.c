/**
 * @file test_attitude.c
 * @brief 直接编译真实 Attitude.c，验证姿态基准、俯仰和侧倾组合运动。
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
 * @brief 按已知车体姿态和角速度，生成当前安装方向下的六轴原始输入。
 * @param pitch_deg 绕车体左轴的俯仰角，单位为度。
 * @param roll_deg 绕车体前轴的侧倾角，单位为度。
 * @param pitch_rate 俯仰角变化率，单位为度/秒。
 * @param roll_rate 侧倾角变化率，单位为度/秒。
 * @note 水平航向始终为零；同时侧倾和俯仰时，机体 Z 轴角速度可以非零，
 *       不能直接把 gyro_z 当成航向变化率。传感器 Y 前、X 右、Z 上。
 */
static void set_tilt_sample(float pitch_deg, float roll_deg,
                            float pitch_rate, float roll_rate)
{
    const float deg_to_rad = 0.01745329252f;
    float pitch = pitch_deg * deg_to_rad;
    float roll = roll_deg * deg_to_rad;

    /* 从已知姿态投影重力，再转换到传感器坐标；加速度不含平移或冲击。 */
    g_imu_data.accel.accel_x = (int16_t)lroundf(-sinf(roll) * cosf(pitch) * IMU_ACCEL_LSB_PER_G);
    g_imu_data.accel.accel_y = (int16_t)lroundf(-sinf(pitch) * IMU_ACCEL_LSB_PER_G);
    g_imu_data.accel.accel_z = (int16_t)lroundf(cosf(roll) * cosf(pitch) * IMU_ACCEL_LSB_PER_G);
    g_imu_data.gyro.gyro_x = (int16_t)lroundf(-pitch_rate * cosf(roll) * IMU_GYRO_LSB_PER_DPS);
    g_imu_data.gyro.gyro_y = (int16_t)lroundf(roll_rate * IMU_GYRO_LSB_PER_DPS);
    g_imu_data.gyro.gyro_z = (int16_t)lroundf(-pitch_rate * sinf(roll) * IMU_GYRO_LSB_PER_DPS);
}

/**
 * @brief 检查解算角度与预设物理姿态一致，且倾斜没有引入明显的虚假航向。
 * @param pitch_deg 本帧预期俯仰角，单位为度。
 * @param roll_deg 本帧预期侧倾角，单位为度。
 * @note 0.3 度容差覆盖原始 ADC 量化和 10 ms 离散积分误差，不代表实车精度。
 */
static void check_tilt(float pitch_deg, float roll_deg)
{
    CHECK(isfinite(g_euler.yaw) && isfinite(g_euler.pitch) && isfinite(g_euler.roll));
    CHECK(fabsf(g_euler.yaw) < 0.3f);
    CHECK(fabsf(g_euler.pitch - pitch_deg) < 0.3f);
    CHECK(fabsf(g_euler.roll - roll_deg) < 0.3f);
}

/**
 * @brief 用 100 个真实解算周期模拟一秒的匀速倾斜，并逐帧检查航向和倾角。
 * @param start_pitch 起始俯仰角，单位为度。
 * @param start_roll 起始侧倾角，单位为度。
 * @param end_pitch 结束俯仰角，单位为度。
 * @param end_roll 结束侧倾角，单位为度。
 */
static void move_tilt(float start_pitch, float start_roll, float end_pitch, float end_roll)
{
    unsigned int frame;
    for (frame = 1u; frame <= 100u; frame++) {
        float pitch = start_pitch + (end_pitch - start_pitch) * (float)frame / 100.0f;
        float roll = start_roll + (end_roll - start_roll) * (float)frame / 100.0f;
        set_tilt_sample(pitch, roll, end_pitch - start_pitch, end_roll - start_roll);
        Attitude_Tick();
        check_tilt(pitch, roll);
    }
}

/**
 * @brief 模拟先侧倾、再俯仰、保持四秒、最后依次回正的完整过程。
 * @param pitch_deg 目标俯仰角，正负值分别覆盖下俯和抬头。
 * @param roll_deg 目标侧倾角；零值用于单独验证纯俯仰。
 * @note 保持阶段角速度为零，用于发现重力估计公式在倾斜时产生的错误校正。
 */
static void test_tilt(float pitch_deg, float roll_deg)
{
    unsigned int frame;
    printf("TEST physical tilt: pitch=%.0f roll=%.0f\n", (double)pitch_deg, (double)roll_deg);
    g_imu_data = (Gyro_Accel_Struct){0};
    g_imu_ready = 1u;
    Attitude_Init();
    move_tilt(0.0f, 0.0f, 0.0f, roll_deg);
    move_tilt(0.0f, roll_deg, pitch_deg, roll_deg);
    set_tilt_sample(pitch_deg, roll_deg, 0.0f, 0.0f);
    for (frame = 0u; frame < 400u; frame++) {
        Attitude_Tick();
        check_tilt(pitch_deg, roll_deg);
    }
    move_tilt(pitch_deg, roll_deg, 0.0f, roll_deg);
    move_tilt(0.0f, roll_deg, 0.0f, 0.0f);
}

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

    /* 单独检查右转符号，避免修正俯仰映射时意外反转原有 Z 轴航向约定。 */
    g_imu_data.gyro.gyro_z = -1640;
    Attitude_Tick();
    CHECK(g_euler.yaw > -1.01f && g_euler.yaw < -0.99f);
    puts("PASS clockwise yaw remains negative");

    test_tilt(30.0f, 0.0f);
    test_tilt(-30.0f, 0.0f);
    test_tilt(30.0f, 20.0f);
    test_tilt(-30.0f, -20.0f);
    puts("PASS pitch, combined tilt, stationary hold and level return");
    return EXIT_SUCCESS;
}

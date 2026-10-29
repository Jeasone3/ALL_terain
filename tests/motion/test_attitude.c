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
 * @brief 在已有侧倾和俯仰时叠加短时冲击，检查真实运动仍被跟踪。
 * @param impact_kind 0=混合冲击，1=仅方向大偏差，2=仅模长异常，3=方向中等偏差。
 * @note 航向始终为零，冲击期间真实俯仰由 30°快速变化到 40°再返回；
 *       加速度干扰和真实陀螺角速度分开生成，避免测试把冲击误当作真实转向。
 */
static void test_pitch_impact(int impact_kind)
{
    unsigned int frame;
    float peak_yaw = 0.0f;
    float peak_pitch_error = 0.0f;
    g_imu_data = (Gyro_Accel_Struct){0};
    Attitude_Init();
    move_tilt(0.0f, 0.0f, 0.0f, 20.0f);
    move_tilt(0.0f, 20.0f, 30.0f, 20.0f);
    for (frame = 0u; frame < 520u; frame++) {
        float pitch = 30.0f;
        float rate = 0.0f;
        if (frame < 10u) {
            pitch += (float)(frame + 1u);
            rate = 100.0f;
        } else if (frame < 20u) {
            pitch += 19.0f - (float)frame;
            rate = -100.0f;
        }
        set_tilt_sample(pitch, 20.0f, rate, 0.0f);
        if (frame < 20u) {
            if (impact_kind == 1 || impact_kind == 3) {
                /* 分别覆盖 25°完全拒绝和 8°部分减权，模长始终为 1g。 */
                float angle = (impact_kind == 1 ? 25.0f : 8.0f) * 0.01745329252f;
                float ax = (float)g_imu_data.accel.accel_y;
                float az = (float)g_imu_data.accel.accel_z;
                g_imu_data.accel.accel_y = (int16_t)lroundf(cosf(angle)*ax + sinf(angle)*az);
                g_imu_data.accel.accel_z = (int16_t)lroundf(-sinf(angle)*ax + cosf(angle)*az);
            } else {
                /* 放大重力投影模拟垂向冲击，同时叠加前向运动加速度。 */
                g_imu_data.accel.accel_x = (int16_t)lroundf(1.4f*g_imu_data.accel.accel_x);
                g_imu_data.accel.accel_y = (int16_t)lroundf(1.4f*g_imu_data.accel.accel_y + (impact_kind == 2 ? 0.0f : 0.5f*IMU_ACCEL_LSB_PER_G));
                g_imu_data.accel.accel_z = (int16_t)lroundf(1.4f*g_imu_data.accel.accel_z);
            }
        }
        Attitude_Tick();
        CHECK(isfinite(g_euler.yaw) && isfinite(g_euler.pitch));
        CHECK(g_attitude_diagnostic.direction_ready != 0u);
        CHECK(fabsf(g_attitude_diagnostic.gyro_yaw_sum +
                    g_attitude_diagnostic.accel_yaw_sum - g_euler.yaw) < 0.01f);
        if (fabsf(g_euler.yaw) > peak_yaw) peak_yaw = fabsf(g_euler.yaw);
        if (fabsf(g_euler.pitch - pitch) > peak_pitch_error) peak_pitch_error = fabsf(g_euler.pitch - pitch);
    }
    printf("impact kind=%d: peak yaw %.4f, pitch error %.4f, final yaw %.4f deg\n",
           impact_kind, (double)peak_yaw, (double)peak_pitch_error, (double)g_euler.yaw);
    CHECK(peak_yaw < 0.3f);
    CHECK(peak_pitch_error < 0.5f);
}

/**
 * @brief 检查冲击期间真实转向未冻结，以及失重后仍能恢复加速度校正。
 * @note 先以水平重力完成对准，再连续 200 ms 转向；随后用无陀螺的
 *       小倾角误差检查校正恢复。另覆盖倾斜初始姿态不会被方向判据永久拒绝。
 */
static void test_impact_recovery(void)
{
    unsigned int frame;
    g_imu_data = (Gyro_Accel_Struct){0};
    Attitude_Init();
    set_tilt_sample(0.0f, 0.0f, 0.0f, 0.0f);
    for (frame = 0u; frame < 100u; frame++) Attitude_Tick();
    for (frame = 0u; frame < 20u; frame++) {
        set_tilt_sample(0.0f, 0.0f, 0.0f, 0.0f);
        g_imu_data.accel.accel_y = (int16_t)(0.6f*IMU_ACCEL_LSB_PER_G);
        g_imu_data.gyro.gyro_z = 30.0f*IMU_GYRO_LSB_PER_DPS;
        Attitude_Tick();
    }
    CHECK(fabsf(g_euler.yaw - 6.0f) < 0.1f);
    /* 零加速度既不能产生 NaN，也不能阻止真实角速度积分。 */
    g_imu_data.accel.accel_x = g_imu_data.accel.accel_y = g_imu_data.accel.accel_z = 0;
    Attitude_Tick();
    CHECK(fabsf(g_euler.yaw - 6.3f) < 0.1f);
    set_tilt_sample(3.0f, 0.0f, 0.0f, 0.0f);
    for (frame = 0u; frame < 1000u; frame++) Attitude_Tick();
    CHECK(fabsf(g_euler.pitch - 3.0f) < 0.1f);
    /* 复位保持原有零基准语义；静止倾斜输入应逐渐对准，而非永远被拒绝。 */
    Attitude_Reset();
    set_tilt_sample(30.0f, 0.0f, 0.0f, 0.0f);
    for (frame = 0u; frame < 1600u; frame++) Attitude_Tick();
    CHECK(fabsf(g_euler.pitch - 30.0f) < 0.1f);
    puts("PASS impact yaw tracking, zero acceleration, correction recovery and tilted startup");
}

/**
 * @brief 验证诊断累计跨角度边界、复位和纯加速度反馈时的来源分解。
 * @note 诊断只记录原算法的变化，不应改变姿态；纯陀螺测试跨过 -180°。
 */
static void test_yaw_diagnostic(void)
{
    unsigned int frame;
    uint32_t previous_resets = g_attitude_diagnostic.reset_count;
    Attitude_Reset();
    CHECK(g_attitude_diagnostic.reset_count == previous_resets + 1u);
    g_imu_data = (Gyro_Accel_Struct){0};
    g_imu_data.gyro.gyro_z = -100.0f*IMU_GYRO_LSB_PER_DPS;
    for (frame = 0u; frame < 450u; frame++) Attitude_Tick();
    CHECK(fabsf(g_attitude_diagnostic.gyro_yaw_sum + 450.0f) < 0.05f);
    CHECK(fabsf(g_attitude_diagnostic.accel_yaw_sum) < 0.01f);
    CHECK(fabsf(g_euler.yaw + 90.0f) < 0.05f);
    Attitude_Reset();
    CHECK(g_attitude_diagnostic.gyro_yaw_sum == 0.0f);
    CHECK(g_attitude_diagnostic.accel_yaw_sum == 0.0f);
    CHECK(g_attitude_diagnostic.direction_ready == 0u);
    set_tilt_sample(25.0f, 20.0f, 0.0f, 0.0f);
    for (frame = 0u; frame < 1500u; frame++) Attitude_Tick();
    CHECK(fabsf(g_attitude_diagnostic.gyro_yaw_sum) < 0.01f);
    CHECK(fabsf(g_attitude_diagnostic.accel_yaw_sum - g_euler.yaw) < 0.01f);
    puts("PASS yaw diagnostic wrap, reset and gyro/acceleration decomposition");
}

/** @brief 检查静止平均重力能立即建立真实倾角，并在第一帧冲击前启用方向拒绝。 */
static void test_gravity_alignment(void)
{
    Accel_struct invalid = {0};
    unsigned int frame;
    uint32_t resets = g_attitude_diagnostic.reset_count;
    set_tilt_sample(30.0f, 20.0f, 0.0f, 0.0f);
    CHECK(Attitude_AlignFromAccel(&g_imu_data.accel));
    CHECK(g_attitude_diagnostic.reset_count == resets + 1u);
    CHECK(g_attitude_diagnostic.direction_ready == 1u);
    CHECK(g_attitude_diagnostic.accel_weight == 1.0f);
    CHECK(fabsf(g_euler.pitch - 30.0f) < 0.01f);
    CHECK(fabsf(g_euler.roll - 20.0f) < 0.01f);
    CHECK(g_euler.yaw == 0.0f);
    for (frame = 0u; frame < 100u; frame++) {
        Attitude_Tick();
        check_tilt(30.0f, 20.0f);
    }
    CHECK(!Attitude_AlignFromAccel(&invalid));
    CHECK(!Attitude_AlignFromAccel(NULL));
    CHECK(g_attitude_diagnostic.reset_count == resets + 1u);
    check_tilt(30.0f, 20.0f);
    set_tilt_sample(31.0f, 20.0f, 100.0f, 0.0f);
    g_imu_data.accel.accel_y += (int16_t)(0.8f*IMU_ACCEL_LSB_PER_G);
    Attitude_Tick();
    CHECK(g_attitude_diagnostic.direction_ready == 1u);
    CHECK(g_attitude_diagnostic.accel_weight == 0.0f);
    check_tilt(31.0f, 20.0f);
    puts("PASS gravity alignment, invalid mean preservation and first-frame impact rejection");
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
    test_pitch_impact(0);
    test_pitch_impact(1);
    test_pitch_impact(2);
    test_pitch_impact(3);
    test_impact_recovery();
    test_yaw_diagnostic();
    test_gravity_alignment();
    return EXIT_SUCCESS;
}

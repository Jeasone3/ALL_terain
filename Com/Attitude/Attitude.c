/**
 * @file Attitude.c
 * @author Jeason
 * @brief 姿态解算
 * @version 0.1
 * @date 2026-09-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "Attitude.h"
#include "Int_MPU6050.h"
#include <math.h>

/* Mahony 参数起点(需上电机实测整定) */
#define ATT_KP        1.0f          /* 比例反馈，小载体 0.5~2 起步 */
#define ATT_KI        0.002f        /* 积分反馈，抑制陀螺零偏残差 */
#define ATT_DT        0.01f         /* TIM4 10ms 固定周期 */
#define ATT_DEG2RAD   0.01745329f
#define ATT_RAD2DEG   57.2957795f

/* 输出欧拉角全局 */
Euler_struct g_euler = {0};

/* 内部四元数与 PI 积分 */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
static float s_yaw_offset = 0.0f;   /* 输出 yaw = raw_yaw - s_yaw_offset */

void Attitude_Init(void)
{
    q0 = 1.0f; q1 = q2 = q3 = 0.0f;
    exInt = eyInt = ezInt = 0.0f;
    s_yaw_offset = 0.0f;
    g_euler.yaw = g_euler.pitch = g_euler.roll = 0.0f;
}

void Attitude_Reset(void)
{
    q0 = 1.0f; q1 = q2 = q3 = 0.0f;
    exInt = eyInt = ezInt = 0.0f;
    s_yaw_offset = 0.0f;             /* 重置后输出 yaw = 0 */
}

void Attitude_SetYawOffset(float deg)
{
    q0 = 1.0f; q1 = q2 = q3 = 0.0f;
    exInt = eyInt = ezInt = 0.0f;
    s_yaw_offset = -deg;             /* raw=0 → 输出 = 0 - (-deg) = deg */
}

/* 6 轴 Mahony 更新：gx/gy/gz 陀螺 rad/s，ax/ay/az 加速度(任意单位，内部归一化) */
static void mahony_update(float gx, float gy, float gz,
                          float ax, float ay, float az)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float ex, ey, ez;

    /* 加速度有效才做重力校正(自由落体/失重时跳过，仅靠陀螺积分) */
    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
        recipNorm = 1.0f / sqrtf(ax*ax + ay*ay + az*az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        /* 估计重力方向(机体系下)，对应 q 旋转矩阵第三列 */
        halfvx = q1*q3 - q0*q2;
        halfvy = q0*q1 + q2*q3;
        halfvz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        /* 误差 = 测量(加速度) × 估计(重力) */
        ex = (ay*halfvz - az*halfvy);
        ey = (az*halfvx - ax*halfvz);
        ez = (ax*halfvy - ay*halfvx);

        /* PI 反馈 */
        exInt += ex * ATT_KI * ATT_DT;
        eyInt += ey * ATT_KI * ATT_DT;
        ezInt += ez * ATT_KI * ATT_DT;
        gx += ATT_KP*ex + exInt;
        gy += ATT_KP*ey + eyInt;
        gz += ATT_KP*ez + ezInt;
    }

    /* 四元数积分：q' = 0.5 * q ⊗ omega */
    gx *= 0.5f; gy *= 0.5f; gz *= 0.5f;
    float dq0 = (-q1*gx - q2*gy - q3*gz) * ATT_DT;
    float dq1 = ( q0*gx + q2*gz - q3*gy) * ATT_DT;
    float dq2 = ( q0*gy - q1*gz + q3*gx) * ATT_DT;
    float dq3 = ( q0*gz + q1*gy - q2*gx) * ATT_DT;
    q0 += dq0; q1 += dq1; q2 += dq2; q3 += dq3;

    /* 归一化 */
    recipNorm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

void Attitude_Tick(void)
{
    if (!g_imu_ready) return;

    /* 取本周期六轴(中断内刚由 Int_MPU6050_Tick 刷新，无竞争)。
       约定 X前 Y右 Z上(模块水平贴装)。若安装方向不同，在此调整轴序/符号：
       例如 pitch 反了就翻转 gx 和 ax；roll 反了翻转 gy/ay；yaw 反向翻转 gz */
    float gx = (float)g_imu_data.gyro.gyro_x / IMU_GYRO_LSB_PER_DPS * ATT_DEG2RAD;
    float gy = (float)g_imu_data.gyro.gyro_y / IMU_GYRO_LSB_PER_DPS * ATT_DEG2RAD;
    float gz = (float)g_imu_data.gyro.gyro_z / IMU_GYRO_LSB_PER_DPS * ATT_DEG2RAD;
    float ax = (float)g_imu_data.accel.accel_x;
    float ay = (float)g_imu_data.accel.accel_y;
    float az = (float)g_imu_data.accel.accel_z;

    mahony_update(gx, gy, gz, ax, ay, az);

    /* 欧拉角(度) */
    float pitch = asinf(2.0f*q0*q2 - 2.0f*q1*q3);
    float roll  = atan2f(2.0f*q0*q1 + 2.0f*q2*q3,
                         1.0f - 2.0f*q1*q1 - 2.0f*q2*q2);
    float yaw   = atan2f(2.0f*q0*q3 + 2.0f*q1*q2,
                         1.0f - 2.0f*q2*q2 - 2.0f*q3*q3);

    g_euler.pitch = pitch * ATT_RAD2DEG;
    g_euler.roll  = roll  * ATT_RAD2DEG;
    g_euler.yaw   = yaw   * ATT_RAD2DEG - s_yaw_offset;
}

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
// ------------------------------Mahony 参数 ------------------------------//
/* Mahony 参数起点(需上电机实测整定) */
#define ATT_KP        1.0f          /* 比例反馈，小载体 0.5~2 起步 */
#define ATT_KI        0.002f        /* 积分反馈，抑制陀螺零偏残差 */
#define ATT_DT        0.01f         /* TIM4 10ms 固定周期 */
#define ATT_DEG2RAD   0.01745329f       //不要改 度转弧度
#define ATT_RAD2DEG   57.2957795f       ///弧度转度

/* 输出欧拉角全局 */
Euler_struct g_euler = {0};

/* 内部四元数与 PI 积分 */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
static float s_yaw_offset = 0.0f;   /* 输出 yaw = raw_yaw - s_yaw_offset */
static float s_last_yaw = 0.0f;          /* 上一帧最终输出 yaw，突变剔除基准 */
static uint8_t s_yaw_initialized = 0;    /* 首帧/复位后首帧标志：跳过判断直接采纳 */
static uint8_t s_spike_check = 1u;       /* 突变剔除开关：受控快转(对齐/转弯)时由 ModeFSM 置 0，循迹/直行时保留防御 */

/**
 * @brief 初始化姿态解算器，使内部状态和对外欧拉角从同一零基准开始。
 * @param 无。
 * @return 无。
 * @note 在 IMU 初始化、静止校准后调用一次；不读取传感器，不驱动电机。
 *       与 Attitude_Reset 使用同一复位操作，避免内部四元数和 g_euler 不一致。
 */
void Attitude_Init(void)
{
    Attitude_Reset();
}

/**
 * @brief 重置四元数、PI 积分和航向偏置，并同步清零输出欧拉角。
 * @param 无。
 * @return 无。
 * @note 在建立新动作的相对角度基准时调用；复位后 yaw、pitch、roll 立即为 0°。
 *       复位不能代替采样，状态机须先获得新一帧有效 IMU 数据再运行角度 PID。
 *       与 Attitude_Tick 在同一控制上下文调用，避免输出被异步更新。
 */
void Attitude_Reset(void)
{
    q0 = 1.0f; q1 = q2 = q3 = 0.0f;
    exInt = eyInt = ezInt = 0.0f;
    s_yaw_offset = 0.0f;
    /* 立即清除外部旧角度，不能等下一次解算才更新，否则新动作可能使用上一动作反馈。 */
    g_euler.yaw = g_euler.pitch = g_euler.roll = 0.0f;
    s_last_yaw = 0.0f;
    s_yaw_initialized = 0;   /* 复位后首帧重新初始化基准，避免用上一动作旧值误判 */
}

/**
 * @brief 重新建立姿态基准，把当前时刻的对外 yaw 设为指定角度。
 * @param deg 当前方向对应的输出 yaw，单位为度；传 0 等同于零航向基准。
 * @return 无。
 * @note 保留原接口语义：重置四元数和积分，再设置输出偏置，而非只平移旧四元数。
 *       yaw 立即为 deg、pitch/roll 立即为 0°；之后输出为原始 yaw 加 deg。
 *       不采样传感器，应在控制上下文调用并等待新一帧有效数据后执行 PID。
 */
void Attitude_SetYawOffset(float deg)
{
    Attitude_Reset();
    s_yaw_offset = -deg;             /* raw=0 → 输出 = 0 - (-deg) = deg */
    g_euler.yaw = deg;
}

/**
 * @brief 设置 yaw 突变剔除开关：受控快转(对齐/转弯)时关，循迹/直行时开。
 * @param enable 1 启用剔除(防御传感器尖峰)；0 禁用(允许正常快转的大角速度)。
 * @return 无。
 * @note 不重置姿态基准；ModeFSM 每帧在 Attitude_Tick 前调用。默认 1(启用)。
 */
void Attitude_SetSpikeCheck(uint8_t enable)
{
    s_spike_check = enable;
}

/**
 * @brief 使用六轴 Mahony 算法更新内部四元数，融合角速度与重力方向。
 * @param gx 车体系前向轴角速度，单位 rad/s。
 * @param gy 车体系左向轴角速度，单位 rad/s。
 * @param gz 车体系向上轴角速度，单位 rad/s；沿用现有 yaw 方向约定。
 * @param ax 车体系前向加速度，任意同量纲单位，内部归一化。
 * @param ay 车体系左向加速度，与 ax 使用同一量纲。
 * @param az 车体系向上加速度，与 ax 使用同一量纲。
 * @return 无。
 * @note 固定以 10 ms 积分，更新四元数和重力误差积分，不直接改 g_euler。
 *       加速度三轴全零时跳过重力校正；六轴无磁力计，yaw 对重力不可观，
 *       故 ez 不反馈到 gz，yaw 纯靠陀螺 z 积分；ex/ey 仍校正 roll/pitch。
 */
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

        /* 估计机体系中的半重力向量；三轴必须使用相同的 1/2 比例。
         * 若 X/Y 为半向量而 Z 为完整分量，倾斜时会产生错误的重力校正。 */
        halfvx = q1*q3 - q0*q2;
        halfvy = q0*q1 + q2*q3;
        halfvz = 0.5f * (q0*q0 - q1*q1 - q2*q2 + q3*q3);

        /* 误差 = 测量(加速度) × 估计(重力) */
        ex = (ay*halfvz - az*halfvy);
        ey = (az*halfvx - ax*halfvz);
        ez = (ax*halfvy - ay*halfvx);

        /* PI 反馈。六轴无磁力计，yaw 对重力不可观：ez 项没有绝对参考，
         * 动态(上坡/下坡、颠簸)时把加速度噪声/线加速度耦合进 gz 积分，
         * 且 ezInt 持续累积造成 yaw 单向偏航。故禁用 ez 反馈，yaw 纯靠
         * 陀螺 z 积分(已做静态零偏校准，平地慢漂移可控)；ex/ey 仍校正
         * roll/pitch。ez 计算保留，便于日后九轴扩展接磁力计。 */
        exInt += ex * ATT_KI * ATT_DT;
        eyInt += ey * ATT_KI * ATT_DT;
        /* ezInt += ez * ATT_KI * ATT_DT; */   /* 六轴禁用 yaw 重力反馈 */
        gx += ATT_KP*ex + exInt;
        gy += ATT_KP*ey + eyInt;
        /* gz += ATT_KP*ez + ezInt; */          /* 同上 */
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

/**
 * @brief 用本周期 IMU 数据解算一帧姿态，并同步写出 yaw、pitch、roll。
 * @param 无。
 * @return 无。
 * @note 在同一 10 ms 控制入口中紧跟成功的 Int_MPU6050_Tick 调用；ready=0 时不更新。
 *       各运动模式均持续更新；不能用 ready 值代替上层对本周期采样成功的检查。
 *       传感器 Y 为前、X 为右、Z 为上；输入统一为前、左、上的右手车体系。
 */
void Attitude_Tick(void)
{
    if (!g_imu_ready) return;

    /* 取本周期六轴(中断内刚由 Int_MPU6050_Tick 刷新，无竞争)。
       当前安装为 Y 前、X 右、Z 上，转换为车体 X 前、Y 左、Z 上：
       车体 X=传感器 Y、车体 Y=-传感器 X、车体 Z=传感器 Z。
       加速度与角速度使用同一右手映射；Z 轴不变，保留左转 yaw 增大的约定。 */
    float gx = (float)g_imu_data.gyro.gyro_y / IMU_GYRO_LSB_PER_DPS * ATT_DEG2RAD;
    float gy = -(float)g_imu_data.gyro.gyro_x / IMU_GYRO_LSB_PER_DPS * ATT_DEG2RAD;
    float gz = (float)g_imu_data.gyro.gyro_z / IMU_GYRO_LSB_PER_DPS * ATT_DEG2RAD;
    float ax = (float)g_imu_data.accel.accel_y;
    float ay = -(float)g_imu_data.accel.accel_x;
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

    /* 帧间 yaw 突变剔除(限幅式)：当前帧相对上一帧最终输出的最短角差 ≥ 阈值时，
       不丢弃整帧，而把角差 clamp 到 ±阈值内再叠加到上帧输出。
       单帧尖峰被压到阈值(暂态振荡 1~2 帧自恢复)；真实大角速度按阈值速率保留，
       台阶持续冲击下不累积冻结误差。
       s_spike_check 由 ModeFSM 控制：受控快转(对齐/转弯)时关，避免正常快转被误判限幅。 */
    if (s_spike_check && s_yaw_initialized) {
        float dyaw = fmodf(g_euler.yaw - s_last_yaw, 360.0f);
        if (dyaw > 180.0f)  dyaw -= 360.0f;
        else if (dyaw < -180.0f) dyaw += 360.0f;
        if (fabsf(dyaw) >= ATT_YAW_SPIKE_THRESH) {
            dyaw = copysignf(ATT_YAW_SPIKE_THRESH, dyaw);   /* 限幅：保留方向，幅值压到阈值 */
            g_euler.yaw = fmodf(s_last_yaw + dyaw + 180.0f, 360.0f) - 180.0f;   /* 归一化到 [-180,180) */
        }
    }
    s_last_yaw = g_euler.yaw;           /* 基准更新为本次最终输出（含替换后的值） */
    s_yaw_initialized = 1;
}

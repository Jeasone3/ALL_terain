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
#define ATT_KI        0.0f          /* 首轮关闭积分学习，避免把坡口运动加速度记成零偏 */
#define ATT_DT        0.01f         /* TIM4 10ms 固定周期 */
#define ATT_DEG2RAD   0.01745329f       //不要改 度转弧度
#define ATT_RAD2DEG   57.2957795f       ///弧度转度

/* 冲击抑制的初始参数，须用实车数据整定；模长误差以 g 为单位。
 * 方向判断使用 1-cos(夹角)，在 3° 到 15°之间降低校正，避免每帧调用 acosf。 */
#define ATT_ACC_MAG_FULL       0.05f
#define ATT_ACC_MAG_REJECT     0.20f
#define ATT_ACC_DIR_FULL       (1.0f - 0.998629535f)
#define ATT_ACC_DIR_REJECT     (1.0f - 0.965925826f)
#define ATT_ACC_ALIGN_FRAMES   20u     /* 连续 200 ms 接近预测方向后启用方向拒绝 */
#define ATT_ACC_RECOVER_STEP   (ATT_DT / 0.20f) /* 权重最快用 200 ms 从 0 恢复到 1 */

/* 输出欧拉角全局 */
Euler_struct g_euler = {0};
AttitudeDiagnostic g_attitude_diagnostic = {0};

/* 内部四元数与 PI 积分 */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
static float s_yaw_offset = 0.0f;   /* 输出 yaw = raw_yaw - s_yaw_offset */
static float s_acc_weight = 0.0f;   /* 加速度比例校正权重；冲击立即降低，平稳后缓慢恢复 */
static uint8_t s_acc_align_frames = 0u; /* 饱和计数；对准完成后保持，直到显式复位 */
static float s_previous_raw_yaw = 0.0f; /* 上一帧原始航向，单位为弧度 */
static float s_gyro_predicted_yaw = 0.0f; /* 当前帧仅按陀螺传播的原始航向 */

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
    uint32_t reset_count = g_attitude_diagnostic.reset_count + 1u;
    q0 = 1.0f; q1 = q2 = q3 = 0.0f;
    exInt = eyInt = ezInt = 0.0f;
    s_yaw_offset = 0.0f;
    s_acc_weight = 0.0f;
    s_acc_align_frames = 0u;
    s_previous_raw_yaw = s_gyro_predicted_yaw = 0.0f;
    g_attitude_diagnostic = (AttitudeDiagnostic){0};
    g_attitude_diagnostic.reset_count = reset_count;
    /* 立即清除外部旧角度，不能等下一次解算才更新，否则新动作可能使用上一动作反馈。 */
    g_euler.yaw = g_euler.pitch = g_euler.roll = 0.0f;
}

uint8_t Attitude_AlignFromAccel(const Accel_struct *gravity_mean)
{
    float ax, ay, az, norm, pitch, roll;
    float sp, cp, sr, cr;
    if (gravity_mean == 0) return 0u;
    ax = (float)gravity_mean->accel_y;
    ay = -(float)gravity_mean->accel_x;
    az = (float)gravity_mean->accel_z;
    norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < IMU_CAL_ACCEL_MIN_G*IMU_ACCEL_LSB_PER_G ||
        norm > IMU_CAL_ACCEL_MAX_G*IMU_ACCEL_LSB_PER_G)
        return 0u;

    pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
    roll = atan2f(ay, az);
    sp = sinf(0.5f*pitch); cp = cosf(0.5f*pitch);
    sr = sinf(0.5f*roll);  cr = cosf(0.5f*roll);
    /* 当前朝向作为 yaw=0，实际倾角写入四元数，不能把倾斜车身当成水平。
     * 全复位先清除历史与诊断，随后直接标记已对准；起步时不得再调用 Reset。 */
    Attitude_Reset();
    q0 = cr*cp; q1 = sr*cp; q2 = cr*sp; q3 = -sr*sp;
    g_euler.pitch = pitch*ATT_RAD2DEG;
    g_euler.roll = roll*ATT_RAD2DEG;
    s_acc_weight = 1.0f;
    s_acc_align_frames = ATT_ACC_ALIGN_FRAMES;
    g_attitude_diagnostic.accel_weight = 1.0f;
    g_attitude_diagnostic.direction_ready = 1u;
    return 1u;
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
 * @brief 将非负误差转换为校正权重，两个阈值之间线性下降。
 * @param error 当前模长误差或方向误差。
 * @param full 低于此值时完全采用加速度比例校正。
 * @param reject 高于此值时不采用该帧加速度比例校正。
 * @return 范围为 0 到 1 的权重。
 */
static float attitude_acc_weight(float error, float full, float reject)
{
    if (error <= full) return 1.0f;
    if (error >= reject) return 0.0f;
    return (reject - error) / (reject - full);
}

/**
 * @brief 使用六轴 Mahony 算法更新内部四元数，融合角速度与重力方向。
 * @param gx 车体系前向轴角速度，单位 rad/s。
 * @param gy 车体系左向轴角速度，单位 rad/s。
 * @param gz 车体系向上轴角速度，单位 rad/s；沿用现有 yaw 方向约定。
 * @param ax 车体系前向加速度，单位为原始计数，灵敏度由 IMU_ACCEL_LSB_PER_G 指定。
 * @param ay 车体系左向加速度，与 ax 使用同一量纲。
 * @param az 车体系向上加速度，与 ax 使用同一量纲。
 * @return 无。
 * @note 固定以 10 ms 积分，更新四元数和重力误差积分，不直接改 g_euler。
 *       冲击期间只减弱重力校正，不停止三轴陀螺积分；六轴仍没有绝对航向参考。
 *       复位后先用模长筛选完成倾角收敛，再启用方向拒绝，故启动时仍须保持静止。
 */
static void mahony_update(float gx, float gy, float gz,
                          float ax, float ay, float az)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float ex = 0.0f, ey = 0.0f, ez = 0.0f;
    float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
    float target_weight = 0.0f;

    /* 从同一帧起点额外计算一次纯陀螺传播，只用于诊断，不替换真实状态。
     * 航向公式采用齐次分母，无需归一化临时四元数，避免多一次开方。
     * 用它和真实更新结果作差，能区分原始角速度与加速度反馈的影响。 */
    {
        float p0 = q0 + 0.5f*(-q1*gx - q2*gy - q3*gz)*ATT_DT;
        float p1 = q1 + 0.5f*( q0*gx + q2*gz - q3*gy)*ATT_DT;
        float p2 = q2 + 0.5f*( q0*gy - q1*gz + q3*gx)*ATT_DT;
        float p3 = q3 + 0.5f*( q0*gz + q1*gy - q2*gx)*ATT_DT;
        s_gyro_predicted_yaw = atan2f(2.0f*(p0*p3 + p1*p2),
                                      p0*p0 + p1*p1 - p2*p2 - p3*p3);
    }

    /* 接近失重时方向不可靠，不归一化；模长必须在归一化之前保留。 */
    if (acc_norm >= 0.1f * IMU_ACCEL_LSB_PER_G) {
        float mag_error = fabsf(acc_norm / IMU_ACCEL_LSB_PER_G - 1.0f);
        float dir_error;
        float dir_weight;
        recipNorm = 1.0f / acc_norm;
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        /* 估计机体系中的半重力向量；三轴必须使用相同的 1/2 比例。
         * 若 X/Y 为半向量而 Z 为完整分量，倾斜时会产生错误的重力校正。 */
        halfvx = q1*q3 - q0*q2;
        halfvy = q0*q1 + q2*q3;
        halfvz = 0.5f * (q0*q0 - q1*q1 - q2*q2 + q3*q3);

        /* 半重力向量乘 2 才是单位方向；正常俯仰应随四元数改变预测方向。
         * 此处使用上一帧估计，方向阈值需容纳 10 ms 的正常运动差和传感器延迟。 */
        dir_error = 1.0f - 2.0f * (ax*halfvx + ay*halfvy + az*halfvz);
        target_weight = attitude_acc_weight(mag_error, ATT_ACC_MAG_FULL, ATT_ACC_MAG_REJECT);
        if (s_acc_align_frames < ATT_ACC_ALIGN_FRAMES) {
            /* 对准前不按绝对方向拒绝，否则坡上复位后可能一直不能收敛。
             * 连续满足模长和方向条件才锁定；单帧碰巧重合不算完成对准。 */
            if (mag_error <= ATT_ACC_MAG_FULL && dir_error <= ATT_ACC_DIR_FULL)
                s_acc_align_frames++;
            else
                s_acc_align_frames = 0u;
        } else {
            dir_weight = attitude_acc_weight(dir_error, ATT_ACC_DIR_FULL, ATT_ACC_DIR_REJECT);
            if (dir_weight < target_weight) target_weight = dir_weight;
        }

        /* 误差 = 测量(加速度) × 估计(重力) */
        ex = (ay*halfvz - az*halfvy);
        ey = (az*halfvx - ax*halfvz);
        ez = (ax*halfvy - ay*halfvx);

    } else if (s_acc_align_frames < ATT_ACC_ALIGN_FRAMES) {
        s_acc_align_frames = 0u;
    }

    /* 干扰立即生效，恢复则限制每帧增量，避免冲击结束时骤然拉动姿态。 */
    if (target_weight < s_acc_weight) {
        s_acc_weight = target_weight;
    } else {
        s_acc_weight += ATT_ACC_RECOVER_STEP;
        if (s_acc_weight > target_weight) s_acc_weight = target_weight;
    }

    /* 首轮 Ki=0，保留原有 PI 结构；若后续重新整定，受干扰和对准期间禁止
     * 累积新误差，已有补偿仍保留。完全可信也不等于已确认静止。 */
    if (s_acc_align_frames >= ATT_ACC_ALIGN_FRAMES && s_acc_weight >= 1.0f) {
        exInt += ex * ATT_KI * ATT_DT;
        eyInt += ey * ATT_KI * ATT_DT;
        ezInt += ez * ATT_KI * ATT_DT;
    }
    /* 三轴使用同一个权重，保留完整校正方向；原始陀螺角速度始终参与积分。 */
    gx += ATT_KP*s_acc_weight*ex + exInt;
    gy += ATT_KP*s_acc_weight*ey + eyInt;
    gz += ATT_KP*s_acc_weight*ez + ezInt;

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
 *       循迹模式可暂停本函数；不能用旧的 ready 值代替上层对本周期采样成功的检查。
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

    /* 逐帧累计而非只保存瞬时峰值，100 ms 打印也不会丢掉已采到的冲击增量。
     * 每一项都取最短角差，避免跨越 ±180°时把正常过界误算为 360°跳变。
     * 分解依赖当前估计的倾角；G 非零仍须区分真实转向、陀螺误差和倾角估计误差。 */
    {
        float gyro_delta = (s_gyro_predicted_yaw - s_previous_raw_yaw)*ATT_RAD2DEG;
        float accel_delta = (yaw - s_gyro_predicted_yaw)*ATT_RAD2DEG;
        if (gyro_delta > 180.0f) gyro_delta -= 360.0f;
        if (gyro_delta < -180.0f) gyro_delta += 360.0f;
        if (accel_delta > 180.0f) accel_delta -= 360.0f;
        if (accel_delta < -180.0f) accel_delta += 360.0f;
        g_attitude_diagnostic.gyro_yaw_sum += gyro_delta;
        g_attitude_diagnostic.accel_yaw_sum += accel_delta;
        g_attitude_diagnostic.accel_weight = s_acc_weight;
        g_attitude_diagnostic.direction_ready = (uint8_t)(s_acc_align_frames >= ATT_ACC_ALIGN_FRAMES);
        s_previous_raw_yaw = yaw;
    }
}

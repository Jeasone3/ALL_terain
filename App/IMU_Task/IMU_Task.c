#include "IMU_Task.h"
#include "Com_pid.h"
#include "Com_Limit.h"
#include "motor.h"
#include "Attitude.h"      /* 姿态解算输出 g_euler */
#include <math.h>          /* fabsf、fmodf */

/* 电机对象(Int/motor.c 定义) */
extern Motor_Struct motorLeft;
extern Motor_Struct motorRight;

/* 转弯 PID (量纲: yaw 误差度 -> PWM ±1000) 转弯PID对象 */
static pid_type_def s_turn_pid;
/* 直行 PID (量纲: yaw 度 -> PWM 差速) 角度环执行PID对象 */
static pid_type_def s_fwd_pid;

/* 当前目标 yaw(度), 由 IMUTask_SetTarget 设定 */
static float s_target_yaw = 0.0f;

/* 电机 PWM 量程(浮点, 与 Com_Limit 的 float 输入类型一致) */
#define MOTOR_MAX_SPEED   1000.0f
/* 转弯 PID 限幅 */
#define TURN_MAX_OUT      800.0f
#define TURN_MAX_IOUT     50.0f
/* 直行参数 */
#define FWD_BASE_SPEED    400.0f    /* 直行基础占空比 */
#define FWD_MAX_OUT       400.0f    /* 纠偏差速限幅 */
#define FWD_DEADBAND      10.0f     /* 最短角差绝对值小于此值时，送入 PID 的误差归零 */

/**
 * @brief 计算从当前航向转到目标航向的最短角差，避免跨越 ±180° 时绕远路。
 * @param target 目标 yaw，单位为度；与 yaw 使用相同的姿态角基准。
 * @param yaw 当前 yaw，单位为度；允许输出角度超出 ±180°。
 * @return [-180°, 180°] 内的 target-yaw；正值需要左转，负值需要右转。
 * @note 仅做角度换算，不改目标、姿态或 PID；恰好相差 180° 时保留差值的符号。
 */
static float imu_angle_error(float target, float yaw)
{
    float error = fmodf(target - yaw, 360.0f);

    /* 例如目标 +170°、当前 -170°，应右转 20°，而不是左转 340°。 */
    if (error > 180.0f) {
        error -= 360.0f;
    } else if (error < -180.0f) {
        error += 360.0f;
    }
    return error;
}

/**
 * @brief 初始化转弯和直行 PID，使两种 IMU 动作拥有独立的历史状态。
 * @param 无。
 * @return 无。
 * @note 启动时调用一次；保留现有增益、积分分离、微分滤波及 PWM 限幅，目标归零。
 *       本函数不采样 IMU，也不驱动电机；动作切换时使用 IMUTask_SetTarget 清历史。
 */

void IMUTask_Init(void)
{
    /* 转弯 PID 起点: Kp=8(90°误差->720), Ki=0.1, Kd=0.5; 需实测整定 */
    pid_real_t turn_params[3] = {8.0f, 0.1f, 0.5f};
    PID_init(&s_turn_pid, PID_POSITION, turn_params, TURN_MAX_OUT, TURN_MAX_IOUT);
    PID_set_integral_separation(&s_turn_pid, 30.0f);   /* 大误差暂停积分防超调 */
    PID_set_deriv_filter(&s_turn_pid, 0.7f);

    /* 直行 PID 起点: Kp=30, Ki=0, Kd=5; 需实测整定 */
    pid_real_t fwd_params[3] = {30.0f, 0.0f, 5.0f};
    PID_init(&s_fwd_pid, PID_POSITION, fwd_params, FWD_MAX_OUT, 0.0f);
    PID_set_deriv_filter(&s_fwd_pid, 0.7f);

    s_target_yaw = 0.0f;
}

/**
 * @brief 设置 IMU 动作的目标航向，并清除两个 PID 上一动作的误差历史。
 * @param target 目标 yaw，单位为度；必须与当前 g_euler.yaw 使用相同基准。
 * @return 无。
 * @note 由运动状态机在动作开始或控制器切换时调用，避免积分和微分历史带入新动作。
 *       左转目标为起始航向 +90°，右转为起始航向 -90°；不会修改姿态或电机输出。
 */
void IMUTask_SetTarget(float target)
{
    s_target_yaw = target;
    PID_clear(&s_turn_pid);
    PID_clear(&s_fwd_pid);
}

/**
 * @brief 读取当前目标与姿态之间的最短角差，供控制和转弯完成判断使用。
 * @param 无。
 * @return [-180°, 180°] 内的目标减当前航向，单位为度；正值左转、负值右转。
 * @note 不采样、不修改 PID；调用前由运动状态机保证本周期 IMU 数据有效。
 */
float IMUTask_GetError(void)
{
    return imu_angle_error(s_target_yaw, g_euler.yaw);
}

/**
 * @brief 执行一帧原地转弯角度 PID，使用两轮反向差速接近目标航向。
 * @param 无。
 * @return 无。
 * @note 仅由当前控制入口每 10 ms 调用一次，不能与循迹或直行控制同时运行。
 *       调用前须取得本周期有效 IMU 数据；目标设置、完成判断和超时停车由状态机负责。
 */
void IMUTask_TurnTick(void)
{
    float error = IMUTask_GetError();

    /* PID 内部仍用 set-ref。把最短角差换成等效反馈，使它实际使用 error；
     * 不跨角度边界时与原来的 target-yaw 数学等价，PID 参数和输出符号不变。 */
    float out = PID_calc(&s_turn_pid, s_target_yaw - error, s_target_yaw);
    /* error>0 时 L<0、R>0，左转增大 yaw；error<0 时反向，右转减小 yaw。 */
    int16_t L = (int16_t)Com_Limit(-out, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    int16_t R = (int16_t)Com_Limit( out, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    Motor_SetSpeed(&motorLeft,  L);
    Motor_SetSpeed(&motorRight, R);
}


/**
 * @brief 执行给定航向的一帧直行控制，统一普通直行和旧的临时前移逻辑。
 * @param target 本帧保持的目标 yaw，单位为度；不改全局动作目标。
 * @return 无。
 * @note 使用直行 PID 与 400 的基础 PWM；保留 10° 死区和 ±400 纠偏限幅。
 *       死区仅令 PID 输入误差归零，已有微分历史仍按原滤波系数衰减。
 *       会写两轮电机，调用前必须保证 IMU 有效及控制器互斥。
 */
static void imu_forward_tick(float target)
{
    float error = imu_angle_error(target, g_euler.yaw);

    /* 先按最短角差判断死区，跨 ±180° 时也使用相同的纠偏方向。 */
    if (fabsf(error) < FWD_DEADBAND) {
        error = 0.0f;
    }
    float out  = PID_calc(&s_fwd_pid, target - error, target);
    float base = FWD_BASE_SPEED;
    /* 正误差需要左转：左轮减速、右轮加速；负误差需要右转，差速方向相反。 */
    int16_t L = (int16_t)Com_Limit(base - out, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    int16_t R = (int16_t)Com_Limit(base + out, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    Motor_SetSpeed(&motorLeft,  L);
    Motor_SetSpeed(&motorRight, R);
}

/**
 * @brief 保留旧的临时前移接口，以 yaw=0° 为目标走一帧 IMU 直行。
 * @param 无。
 * @return 无。
 * @note 仅兼容旧调用；需先把所需前移方向设为姿态的 0° 基准，并保证 IMU 有效。
 *       不修改 s_target_yaw，不清 PID 历史；新路线应明确选择 IMU 直行动作。
 */
void Temp_Turn_Forward(void)
{
    imu_forward_tick(0.0f);
}

/**
 * @brief 执行一帧 IMU 直行，靠角度 PID 保持最近一次设置的目标航向。
 * @param 无。
 * @return 无。
 * @note 由当前控制入口每 10 ms 调用一次，调用前 IMU 必须有效。
 *       转后接 IMU 直行时继续保持转弯目标；本函数不改变目标，也不判断动作结束条件。
 */
void IMUTask_ForwardTick(void)
{
    imu_forward_tick(s_target_yaw);
}

/**
 * @brief 把两轮 PWM 设为零，供正常停车及故障停车统一调用。
 * @param 无。
 * @return 无。
 * @note 不改目标和 PID 历史，也不关闭传感器；恢复动作时由状态机重设目标并清历史。
 *       仅在当前控制入口调用，避免与另一控制器同时写电机。
 */
void IMUTask_Stop(void)
{
    Motor_SetSpeed(&motorLeft,  0);
    Motor_SetSpeed(&motorRight, 0);
}

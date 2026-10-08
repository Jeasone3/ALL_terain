#include "IMU_Task.h"
#include "Com_pid.h"
#include "Com_Limit.h"
#include "motor.h"
#include "Attitude.h"      /* g_euler */
#include <math.h>          /* fabsf */

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
#define FWD_MAX_OUT       400.0f   /* 纠偏差速限幅 */
/* 航向保护默认 15°，不能等偏差达到 10° 才开始纠偏。
 * 使用较小死区，在候选和驶离阶段提前修正；实际增益仍需实车整定。 */
#define FWD_DEADBAND      2.0f      /* 仅抑制小于 2° 的微小角度波动 */

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

void IMUTask_SetTarget(float target)
{
    s_target_yaw = target;
    PID_clear(&s_turn_pid);
    PID_clear(&s_fwd_pid);
}


/**
 * @brief 将目标航向与当前航向的误差限制在 [-180, 180] 范围内
 *          航向跨过 ±180° 时，控制器和状态机使用相同的最短角度误差
 * @param target 
 * @param yaw 
 * @return float 
 */
static float heading_error(float target, float yaw)
{
    float error = target - yaw;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

/**
 * @brief 角度控制转弯，原地差速转弯
 * 
 */
void IMUTask_TurnTick(void)
{
    /* PID_calc 内部 err = set - ref = target - yaw
     * 左转 target>0 -> err>0 -> out>0 -> L=-out<0 R=+out>0 左反转右正转, yaw 增大 ✓
     * 右转 target<0 -> err<0 -> out<0 -> L>0 R<0 左正转右反转, yaw 减小 ✓ */
    float error = heading_error(s_target_yaw, g_euler.yaw);
    float out = PID_calc(&s_turn_pid, s_target_yaw - error, s_target_yaw);
    int16_t L = (int16_t)Com_Limit(-out, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    int16_t R = (int16_t)Com_Limit( out, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    Motor_SetSpeed(&motorLeft,  L);
    Motor_SetSpeed(&motorRight, R);
}



/**
 * @brief 直行控制函数
 * 直行共用计算：进入路口保持原航向，转后保持新的目标航向。
 * @param target  目标速度
 * @param base_speed    基础速度(占空比)
 */
static void forward_tick(float target, float base_speed)
{
    float error = heading_error(target, g_euler.yaw);
    /* 目标减实测得到误差；小偏差归零，大于死区就开始纠偏。 */
    if (fabsf(error) < FWD_DEADBAND) {
        error = 0.0f;
    }
    float out = PID_calc(&s_fwd_pid, target - error, target);
    float base = Com_Limit(base_speed, 0.0f, MOTOR_MAX_SPEED);
    /* 低速驶离时只前进纠偏，差速不能大到让某一轮反转。 */
    out = Com_Limit(out, -base, base);
    /* 按本工程的约定：正误差要求 yaw 增大，左轮减速、右轮加速。
     * 实测 yaw 已偏正且目标为 0 时，误差为负，输出相反差速让 yaw 减小。
     * 该符号是否匹配真实 MPU 安装和电机接线，必须在车上观察确认。 */
    int16_t L = (int16_t)Com_Limit(base - out, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    int16_t R = (int16_t)Com_Limit(base + out, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    Motor_SetSpeed(&motorLeft,  L);
    Motor_SetSpeed(&motorRight, R);
}

/* 保留旧接口：转前直行固定保持进入路口时的零度基准。 */
void Temp_Turn_Forward(void)
{
    forward_tick(0.0f, FWD_BASE_SPEED);
}

void IMUTask_ForwardTick(void)
{
    forward_tick(s_target_yaw, FWD_BASE_SPEED);
}

void IMUTask_ForwardTickWithSpeed(float base_speed)
{
    forward_tick(s_target_yaw, base_speed);
}

void IMUTask_Stop(void)
{
    Motor_SetSpeed(&motorLeft,  0);
    Motor_SetSpeed(&motorRight, 0);
}

#include "IMU_Task.h"
#include "Com_pid.h"
#include "Com_Limit.h"
#include "motor.h"
#include "Attitude.h"      /* g_euler */
#include <math.h>          /* fabsf */

/* 电机对象(Int/motor.c 定义) */
extern Motor_Struct motorLeft;
extern Motor_Struct motorRight;

/* 转弯 PID (量纲: yaw 误差度 -> PWM ±1000) */
static pid_type_def s_turn_pid;
/* 直行 PID (量纲: yaw 度 -> PWM 差速) */
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
#define FWD_DEADBAND      10.0f     /* |yaw-target|<此值不纠偏, 防抖 */

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

void IMUTask_TurnTick(void)
{
    /* PID_calc 内部 err = set - ref = target - yaw
     * 左转 target>0 -> err>0 -> out>0 -> L=-out<0 R=+out>0 左反转右正转, yaw 增大 ✓
     * 右转 target<0 -> err<0 -> out<0 -> L>0 R<0 左正转右反转, yaw 减小 ✓ */
    float out = PID_calc(&s_turn_pid, g_euler.yaw, s_target_yaw);
    int16_t L = (int16_t)Com_Limit(-out, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    int16_t R = (int16_t)Com_Limit( out, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    Motor_SetSpeed(&motorLeft,  L);
    Motor_SetSpeed(&motorRight, R);
}

void IMUTask_ForwardTick(void)
{
    /* 直行纠偏(原版): 保持 s_target_yaw 航向直行. FORWARD 态用(target=0).
     * 车偏右 yaw<0 -> err=target-yaw>0 -> out>0 -> L=base-out 减速, R=base+out 加速 -> 左转回正 ✓
     * 死区: |yaw-target|<10° 置 yaw=target 使 err=0, 不纠偏防抖 */
    float yaw = g_euler.yaw;
    if (fabsf(s_target_yaw - yaw) < FWD_DEADBAND) {
        yaw = s_target_yaw;
    }
    float out  = PID_calc(&s_fwd_pid, yaw, s_target_yaw);
    float base = FWD_BASE_SPEED;
    int16_t L = (int16_t)Com_Limit(base - out, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    int16_t R = (int16_t)Com_Limit(base + out, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    Motor_SetSpeed(&motorLeft,  L);
    Motor_SetSpeed(&motorRight, R);
}

/* 临时转弯前直行: 固定 target=0 保持当前航向直行, 用于 TURN 态 settling 期间
 * 向前走到路口中心. 不读 s_target_yaw(那是转弯目标 ±90, 读它会变弧形转弯而非直行). */
void Temp_Turn_Forward(void)
{
    float yaw = g_euler.yaw;
    if (fabsf(yaw) < FWD_DEADBAND) {
        yaw = 0.0f;
    }
    float out  = PID_calc(&s_fwd_pid, yaw, 0.0f);
    float base = FWD_BASE_SPEED;
    int16_t L = (int16_t)Com_Limit(base - out, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    int16_t R = (int16_t)Com_Limit(base + out, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    Motor_SetSpeed(&motorLeft,  L);
    Motor_SetSpeed(&motorRight, R);
}

void IMUTask_Stop(void)
{
    Motor_SetSpeed(&motorLeft,  0);
    Motor_SetSpeed(&motorRight, 0);
}

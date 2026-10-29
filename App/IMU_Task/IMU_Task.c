/**
 * @file IMU_Task.c
 * @author your name (you@domain.com)
 * @brief 角度PID控制任务：转弯和直行，独立的 PID 历史状态。
 * @version 0.1
 * @date 2026-10-11
 * 
 * @copyright Copyright (c) 2026
 * 
 */
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
#define TURN_MAX_OUT      600.0f
#define TURN_MAX_IOUT     50.0f
#define TURN_MIN_OUT      210.0f    /* 容差外的最低转弯 PWM，首轮试用 200，须按实车转动门槛标定 */
/* 对齐用更柔的限幅：功能切换时慢转到 ideal_yaw，避免快速过冲；首轮标定值，可调 */
#define ALIGN_MAX_OUT     500.0f
#define ALIGN_MIN_OUT     250.0f
/* 直行参数 */
#define FWD_BASE_SPEED    400.0f    /* 直行基础占空比 */
#define FWD_MAX_OUT       1000.0f    /* 首轮越障纠偏上限，防止一侧大幅减速或反转 */
#define FWD_DEADBAND      1.5f      /* 死区 */

/**
 * @brief 计算从当前航向转到目标航向的最短角差，避免跨越 ±180° 时绕远路。
 * @param target 目标 yaw，单位为度；与 yaw 使用相同的姿态角基准。
 * @param yaw 当前 yaw，单位为度；允许输出角度超出 ±180°。
 * @return [-180°, 180°] 内的 target-yaw；正值需要左转，负值需要右转。
 * @note 仅做角度换算，不改目标、姿态或 PID；恰好相差 180° 时保留差值的符号。
 */
static float imu_angle_error(float target, float yaw)
{
    float error = fmodf(target - yaw, 360.0f);//取余数 ，差角到360有多少

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
    pid_real_t turn_params[3] = {9.0f, 0.1f, 0.5f};
    PID_init(&s_turn_pid, PID_POSITION, turn_params, TURN_MAX_OUT, TURN_MAX_IOUT);
    PID_set_integral_separation(&s_turn_pid, 30.0f);   /* 大误差暂停积分防超调 */
    PID_set_deriv_filter(&s_turn_pid, 0.7f);

    /* 保留当前直行增益，仅调整输出限幅；仍需实车整定。 */
    pid_real_t fwd_params[3] = {75.0f, 0.3f, 35.0f};
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
 *       尚未到位且 PID 输出不足时，按剩余角差方向补足最低 PWM，帮助克服末段静摩擦。
 *       进入共用的到位容差后输出零，等待状态机用后续新采样完成连续帧确认。
 */
void IMUTask_TurnTick(void)
{
    //保证最短角度差，避免跨越 ±180° 时绕远路。
    float error = IMUTask_GetError();

    /* PID 内部仍用 set-ref。把最短角差换成等效反馈，使它实际使用 error；
     * 不跨角度边界时与原来的 target-yaw 数学等价，PID 参数和输出符号不变。 */
    float out = PID_calc(&s_turn_pid, s_target_yaw - error, s_target_yaw);

    /* 例如还差 6.8°时，原比例项只有约 54 PWM，积分上限也只有 50 PWM。
     * 此时继续等待不一定能克服静摩擦；仅补足小输出，不改变原 PID 增益与最大限幅。
     * 进入容差后先停转，避免最低 PWM 在完成确认期间继续把车身推过目标。 */
    if (fabsf(error) < IMU_TURN_DONE_TOLERANCE) {
        out = 0.0f;
    }else if(fabsf(out) < TURN_MIN_OUT) { //如果输出小于最低PWM，则补足最低PWM
        if (error > 0.0f) 
            out = TURN_MIN_OUT;
        else 
            out = -TURN_MIN_OUT;
    }

    /* error>0 时 L<0、R>0，左转增大 yaw；error<0 时反向，右转减小 yaw。 */
    int16_t L = (int16_t)Com_Limit(-out, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    int16_t R = (int16_t)Com_Limit( out, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    Motor_SetSpeed(&motorLeft,  L);
    Motor_SetSpeed(&motorRight, R);
}


/**
 * @brief 运行一帧原地对齐 PID，复用转弯 PID 增益，但限幅更低、转速更柔。
 * @param 无。
 * @return 无。
 * @note 功能切换时把车慢转到 g_route.ideal_yaw（90° 倍数）；逻辑同 IMUTask_TurnTick，
 *       仅换容差为 IMU_ALIGN_DONE_TOLERANCE、限幅为 ALIGN_MAX_OUT/ALIGN_MIN_OUT。
 *       PID_calc 内部仍按 TURN_MAX_OUT 限幅，故输出后再用 ALIGN_MAX_OUT 二次限幅。
 *       进入对齐容差后输出零，等待状态机连续帧确认完成。
 */
void IMUTask_AlignTick(void)
{
    float error = IMUTask_GetError();

    /* 与 TurnTick 相同的 PID 计算，使用最短角差作为等效反馈。 */
    float out = PID_calc(&s_turn_pid, s_target_yaw - error, s_target_yaw);

    /* PID 内部按 TURN_MAX_OUT(600) 限幅；对齐需要更柔，再用 ALIGN_MAX_OUT(350) 二次限幅。 */
    out = Com_Limit(out, -ALIGN_MAX_OUT, ALIGN_MAX_OUT);

    if (fabsf(error) < IMU_ALIGN_DONE_TOLERANCE) {
        out = 0.0f;
    } else if (fabsf(out) < ALIGN_MIN_OUT) {
        /* 容差外的小输出补足最低对齐 PWM，帮助克服末段静摩擦。 */
        if (error > 0.0f)
            out = ALIGN_MIN_OUT;
        else
            out = -ALIGN_MIN_OUT;
    }

    /* error>0 时 L<0、R>0，左转增大 yaw；error<0 时反向，右转减小 yaw。 */
    int16_t L = (int16_t)Com_Limit(-out, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    int16_t R = (int16_t)Com_Limit( out, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    Motor_SetSpeed(&motorLeft,  L);
    Motor_SetSpeed(&motorRight, R);
}


/**
 * @brief 执行给定航向的一帧前进或倒退控制，共用航向 PID 与差速方向。
 * @param target 本帧保持的目标 yaw，单位为度；不改全局动作目标。
 * @param base_pwm 带方向的基础 PWM，正值前进、负值倒退；不在此处重置姿态或 PID。
 * @return 无。
 * @note 保留当前死区和 PID 限幅；两轮输出限制在基础 PWM 对应的方向，防止纠偏时反转。
 *       死区仅令 PID 输入误差归零，已有微分历史仍按原滤波系数衰减。
 *       会写两轮电机，调用前必须保证 IMU 有效及控制器互斥。
 */
static void imu_straight_tick(float target, int16_t base_pwm)
{
    float error = imu_angle_error(target, g_euler.yaw);

    /* 先按最短角差判断死区，跨 ±180° 时也使用相同的纠偏方向。 */
    if (fabsf(error) < FWD_DEADBAND) {
        error = 0.0f;
    }
    float out  = PID_calc(&s_fwd_pid, target - error, target);
    float base = (float)base_pwm;
    float min_pwm = base_pwm < 0 ? -MOTOR_MAX_SPEED : 0.0f; /* 倒退允许负 PWM，前进下限为零 */
    float max_pwm = base_pwm < 0 ? 0.0f : MOTOR_MAX_SPEED;  /* 倒退纠偏不能令某一轮向前 */
    /* 正误差需要左转：左轮减速、右轮加速；负误差需要右转，差速方向相反。 */
    int16_t L = (int16_t)Com_Limit(base - out, min_pwm, max_pwm);
    int16_t R = (int16_t)Com_Limit(base + out, min_pwm, max_pwm);
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
    imu_straight_tick(0.0f, (int16_t)FWD_BASE_SPEED);
}

/**
 * @brief 执行一帧 IMU 直行，靠角度 PID 保持最近一次设置的目标航向。
 * @param base_pwm 本次动作的基础 PWM，不是编码器反馈车速。
 * @return 无。
 * @note 由当前控制入口每 10 ms 调用一次，调用前 IMU 必须有效。
 *       转后接 IMU 直行时继续保持转弯目标；本函数不改变目标，也不判断动作结束条件。
 */
void IMUTask_ForwardTick(uint16_t base_pwm)
{
    imu_straight_tick(s_target_yaw, (int16_t)base_pwm);
}

/**
 * @brief 执行一帧保持当前目标航向的倒退，沿用直行 PID 的角差与差速符号。
 * @param base_pwm 已由运动层校验的正值 PWM 幅度，范围 1 至 1000。
 * @return 无。
 * @note 只将基础 PWM 取负，不能把两轮前进输出整体取负，否则航向纠偏方向也会反转。
 *       两轮输出均限制在 [-1000, 0]；调用前须取得本帧有效 IMU 数据。
 */
void IMUTask_ReverseTick(uint16_t base_pwm)
{
    imu_straight_tick(s_target_yaw, -(int16_t)base_pwm);
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

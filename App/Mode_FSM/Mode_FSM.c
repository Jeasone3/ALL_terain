#include "Mode_FSM.h"
#include "Track_Task.h"    /* follow_line, g_line_controller, LINE_RAW_VALUE */
#include "Int_Track.h"     /* Read_All_Track, GRAYSCALE_SENSOR_CHANNELS */
#include "Int_MPU6050.h"   /* Int_MPU6050_Tick, g_imu_ready */
#include "Attitude.h"      /* Attitude_Reset, Attitude_Tick, g_euler */
#include "IMU_Task.h"      /* IMUTask_SetTarget/TurnTick/ForwardTick/Stop */
#include "motor.h"         /* Motor_SetSpeed */
#include <math.h>          /* fabsf */

/* 电机对象(Int/motor.c 定义) */
extern Motor_Struct motorLeft;
extern Motor_Struct motorRight;

/* 灰度传感器数组(Int/Int_Track.c 定义) */
extern uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];

/* 全局状态机对象 */
ModeFSM_t g_mode_fsm;

/* 转弯完成容差(度): |yaw - target| < 此值判完成 */
#define TURN_DONE_TOLERANCE   5.0f
/* FORWARD 固定持续帧数(10ms/帧, 50 = 500ms) */
#define FORWARD_HOLD_FRAMES   10u
/* 切换状态后稳定帧数(10ms*10=100ms): 期间停车, 防立刻动作时序错乱
 * A.进转弯/直行态: 姿态收敛+机械稳定; B.切回循迹: 传感器稳定防误触发 */
#define SETTLE_FRAMES         10u

/* ==================== 触发判据(简单版, 后期加持续帧确认防误触发) ====================
 * g_sensor_data[0..7] 对应 IN1..IN8, 权重 -5,-4,-2,-1,1,2,4,5; LINE_RAW_VALUE=1 在线上.
 * 左4个(IN1..IN4)全亮 -> 左转; 右4个(IN5..IN8)全亮 -> 右转; 8全亮 -> 十字.
 * 注: 普通弯道也会触发, 后期路线规划时再加确认机制.
 */
static uint8_t left4_all_on(void)
{
    return g_sensor_data[0] && g_sensor_data[1] &&
           g_sensor_data[2] && g_sensor_data[3];
}
static uint8_t right4_all_on(void)
{
    return g_sensor_data[4] && g_sensor_data[5] &&
           g_sensor_data[6] && g_sensor_data[7];
}
static uint8_t all8_on(void)
{
    return left4_all_on() && right4_all_on();
}


/* ==================== 状态机对象初始化 ==================== */

/**
 * @brief 状态开关
 *        传入不同的状态，用来开启循迹或者关闭循迹，并设置目标角度
 * 
 * @param new_state 如果不是 STATE_NORMAL_TRACK, 则进入非循迹态
 * @param target_yaw 要转的期望角度
 */
static void enter_state(Run_State new_state, float target_yaw)
{
    /* 离开 NORMAL_TRACK 进入非循迹态: 归0 yaw 作角度基准, 设目标, 清 PID */
    if (g_mode_fsm.state == STATE_NORMAL_TRACK &&
        new_state != STATE_NORMAL_TRACK)
    {
        Attitude_Reset();            /* 四元数复位, yaw 归0, 消除漂移 */
        IMUTask_SetTarget(target_yaw);
    }
    /* 回到 NORMAL_TRACK: 两轮停一帧, 防残留差速抖动 */
    if (new_state == STATE_NORMAL_TRACK &&
        g_mode_fsm.state != STATE_NORMAL_TRACK)
    {
        IMUTask_Stop();
    }
    g_mode_fsm.state        = new_state;
    g_mode_fsm.target_yaw   = target_yaw;
    g_mode_fsm.state_frames = 0u;
}


/**
 * @brief 判断是否完成转弯，如果小于死区角度就是转弯完成
 * 
 * @return 1: 转弯完成; 0: 转弯未完成
 */
static uint8_t is_turn_done(void)
{
    float err = g_mode_fsm.target_yaw - g_euler.yaw;
    return fabsf(err) < TURN_DONE_TOLERANCE;
}

/* FORWARD 完成: 固定帧数到(FORWARD 本身直行穿过路口, 无 settle) */
static uint8_t is_forward_done(void)
{
    return g_mode_fsm.state_frames >= FORWARD_HOLD_FRAMES;
}

/* ==================== 状态机节拍(10ms, TIM4 回调调用) ==================== */
void ModeFSM_Tick(void)
{
    /* A. 进转弯态: 前 SETTLE_FRAMES 帧(100ms) 向前直行走到路口中心, 再开始转弯.
     *    settling 期间读 IMU 让 yaw 收敛, ForwardTick 保持当前航向直行.
     * B. 切回循迹: 无延时, 转弯完成立刻循迹 */
    uint8_t settling = (g_mode_fsm.state_frames < SETTLE_FRAMES);

    switch (g_mode_fsm.state)
    {
    case STATE_NORMAL_TRACK:
        /* 循迹态: 直接循迹, 不读 IMU 省 CPU; B: 无延时立刻循迹 */
        Read_All_Track(g_sensor_data);
        follow_line(&g_line_controller, g_sensor_data, LINE_RAW_VALUE);
        /* 触发判据: 8全亮(十字)优先, 再左右4亮 */
        if      (all8_on())        enter_state(STATE_CROSS,       0.0f);
        else if (left4_all_on())   enter_state(STATE_TURN_LEFT,  +90.0f);
        else if (right4_all_on())  enter_state(STATE_TURN_RIGHT, -90.0f);
        break;

    case STATE_CROSS:
        /* 后期路线规划决策点: 决定左转/右转/直行; 默认直行 */
        enter_state(STATE_FORWARD, 0.0f);
        break;

    case STATE_TURN_LEFT:
    case STATE_TURN_RIGHT:
        /* 非循迹态: 读 IMU + 姿态解算 */
        Int_MPU6050_Tick();
        Attitude_Tick();
        if (settling) {
            Temp_Turn_Forward();   /* A: 向前直行 100ms 走到路口中心(固定 target=0) */
        } else {
            IMUTask_TurnTick();      /* 原地差速转弯 */
        }
        g_mode_fsm.state_frames++;
        if (!settling && is_turn_done()) enter_state(STATE_NORMAL_TRACK, 0.0f);
        break;

    case STATE_FORWARD:
        /* 非循迹态: 读 IMU + 姿态解算 + 直行纠偏穿过路口(本身直行, 无 settle) */
        Int_MPU6050_Tick();
        Attitude_Tick();
        IMUTask_ForwardTick();
        g_mode_fsm.state_frames++;
        if (is_forward_done()) enter_state(STATE_NORMAL_TRACK, 0.0f);
        break;
    }
}

void ModeFSM_Init(void)
{
    g_mode_fsm.state        = STATE_NORMAL_TRACK;
    g_mode_fsm.target_yaw   = 0.0f;
    g_mode_fsm.state_frames = 0u;
}

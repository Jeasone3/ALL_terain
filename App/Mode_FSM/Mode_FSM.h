#ifndef __MODE_FSM_H__
#define __MODE_FSM_H__

#include "main.h"
#include <stdint.h>

/**
 * @file Mode_FSM.h
 * @brief 循迹/IMU 角度闭环 状态机调度
 *
 * 状态流:
 *   NORMAL_TRACK --触发--> CROSS/TURN_LEFT/TURN_RIGHT
 *   CROSS        --默认--> FORWARD (路口内决策后期填)
 *   TURN_*       --|yaw-target|<5°--> NORMAL_TRACK
 *   FORWARD      --固定帧数--> NORMAL_TRACK
 *
 * NORMAL_TRACK 态不读 IMU(省 CPU); 非循迹态读 IMU + 跑角度闭环.
 * 进转弯/直行态调 Attitude_Reset() 归0 yaw 作为角度基准.
 */

/* 运行状态 */
typedef enum {
    STATE_NORMAL_TRACK = 0,   /* 普通巡线: 循迹开, IMU 停 */
    STATE_CROSS,             /* 十字路口过渡: 去向决策后期填, 默认进 FORWARD */
    STATE_TURN_LEFT,         /* 左转中: target = +90° (yaw 左转为正) */
    STATE_TURN_RIGHT,        /* 右转中: target = -90° */
    STATE_FORWARD            /* 路口直行纠偏: target = 0° */
} Run_State;

typedef struct {
    Run_State  state;          /* 当前状态 */
    float      target_yaw;     /* 进入转弯/直行时目标 yaw(度) */
    uint32_t   state_frames;   /* 当前状态已运行帧数(10ms/帧) */
} ModeFSM_t;

/* 全局状态机对象 */
extern ModeFSM_t g_mode_fsm;

/* 初始化状态机为 NORMAL_TRACK */
void ModeFSM_Init(void);

/* 10ms 周期入口: 替代原 TrackTask_Tick, 由 TIM4 回调调用.
 * 内部按状态分支调度循迹/IMU/角度闭环 */
void ModeFSM_Tick(void);

#endif /* __MODE_FSM_H__ */

#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__

#include "main.h"

/**
 * @file IMU_Task.h
 * @brief IMU 角度闭环控制(转弯原地差速 / 直行纠偏)
 *
 * 由 Mode_FSM 在非循迹态每 10ms 调用, 反馈取 g_euler.yaw.
 * 目标 yaw 由 Mode_FSM 切状态时通过 IMUTask_SetTarget 设定.
 */

/* 初始化转弯/直行 PID 参数(起点值, 需上电机实测整定) */
void IMUTask_Init(void);

/* 设当前目标 yaw(度): 左转 +90, 右转 -90, 直行 0. 同时清 PID 历史 */
void IMUTask_SetTarget(float target);

/* 原地差速转弯一帧: left=-out, right=+out, out=yaw PID 输出 */
void IMUTask_TurnTick(void);

/* 直行纠偏一帧: base±out, |yaw-target|<10° 死区不纠偏防抖 */
void IMUTask_ForwardTick(void);

/* 临时转弯前直行(固定 target=0): TURN 态 settling 期间向前走到路口中心 */
void Temp_Turn_Forward(void);

/* 两轮停速 */
void IMUTask_Stop(void);

#endif /* __IMU_TASK_H__ */

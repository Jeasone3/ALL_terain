#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__

#include "main.h"

/**
 * @file IMU_Task.h
 * @brief IMU 角度闭环控制(转弯原地差速 / 直行纠偏)
 *
 * 由运动状态机在 IMU 模式每 10 ms 互斥调用，反馈取本周期有效的 g_euler.yaw。
 * 目标由 IMUTask_SetTarget 设定；控制和完成判断统一使用最短角差。
 */

/**
 * @brief 初始化转弯和直行 PID，保留当前参数并清除历史。
 * @param 无。
 * @return 无。
 * @note 系统启动时调用一次；目标归零，不采样 IMU，也不写电机。
 */
void IMUTask_Init(void);

/**
 * @brief 设置当前动作的目标 yaw，并清除两种 PID 的历史。
 * @param target 目标角度，单位为度；与当前姿态基准一致，左转 +90°、右转 -90°。
 * @return 无。
 * @note 动作开始或切换控制器时调用；相对转弯目标应由状态机加到动作起始航向上。
 */
void IMUTask_SetTarget(float target);

/**
 * @brief 返回当前目标减当前 yaw 的最短角差。
 * @param 无。
 * @return [-180°, 180°] 内的角差，单位为度；正值左转、负值右转。
 * @note 不改变任何控制状态；状态机应先检查本周期 IMU 数据有效，再用于完成判断。
 */
float IMUTask_GetError(void);

/**
 * @brief 运行一帧原地转弯 PID，电机输出为 left=-out、right=+out。
 * @param 无。
 * @return 无。
 * @note 每 10 ms 调用一次；调用前 IMU 必须有效，且本周期不运行循迹或直行 PID。
 */
void IMUTask_TurnTick(void);

/**
 * @brief 运行一帧保持当前目标航向的直行 PID，使用 base±out 差速纠偏。
 * @param 无。
 * @return 无。
 * @note 最短角差小于 10° 时令 PID 输入误差归零；调用前保证 IMU 有效及控制器互斥。
 */
void IMUTask_ForwardTick(void);

/**
 * @brief 保留旧的临时前移接口，以固定 yaw=0° 为目标执行一帧直行。
 * @param 无。
 * @return 无。
 * @note 不修改当前动作目标、不清历史；所需前移方向须先成为姿态 0° 基准。
 */
void Temp_Turn_Forward(void);

/**
 * @brief 将两轮 PWM 设为零。
 * @param 无。
 * @return 无。
 * @note 不改目标、PID 或传感器；下一动作应由状态机重新设目标并清 PID 历史。
 */
void IMUTask_Stop(void);

#endif /* __IMU_TASK_H__ */

/**
 * @file Attitude.h
 * @author Jeason
 * @brief 六轴姿态解算及相对航向基准；输出角度与内部复位状态同步。
 * @version 0.1
 * @date 2026-09-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef __ATTITUDE_H__
#define __ATTITUDE_H__

#include "IMU_Config.h"

/* 欧拉角全局：yaw/pitch/roll，单位为度；由同一 10 ms 控制入口的 Attitude_Tick 更新。
   yaw 没有磁力计绝对参考，长时间保持航向会漂移；动作基准复位不等于零偏重新校准。
   设置 yaw 基准后输出可超出 ±180°，运动控制器需使用最短角差。 */
extern Euler_struct g_euler;

/* 当前安装方向：传感器 Z 上、Y 前、X 右；解算采用车体 X 前、Y 左、Z 上的右手坐标系。
   Attitude_Tick 对加速度和角速度统一映射为 (Y, -X, Z)，保留左转 yaw 增大的约定。
   若模块朝向不同，需核对三轴方向并使用一致的右手映射，不能只交换两个轴。 */

/**
 * @brief 初始化内部姿态与对外欧拉角，使两者使用相同的零基准。
 * @param 无。
 * @return 无。
 * @note 在 IMU 初始化、静止校准后调用一次；不读取传感器或驱动电机。
 */
void Attitude_Init(void);

/**
 * @brief 解算一帧六轴数据，并更新全局欧拉角。
 * @param 无。
 * @return 无。
 * @note 固定 10 ms 周期，在同一控制入口成功采样 IMU 后调用；ready=0 时保留旧输出。
 *       上层负责检查本周期数据有效，并按当前动作决定是否启用解算。
 */
void Attitude_Tick(void);

/**
 * @brief 重置内部四元数、PI 积分、yaw 偏置，并立即清零三项欧拉角。
 * @param 无。
 * @return 无。
 * @note 建立新动作角度基准时在控制上下文调用；复位后仍须取得新一帧有效 IMU 数据。
 */
void Attitude_Reset(void);


/**
 * @brief 重建姿态基准，把当前方向的输出 yaw 立即设为 deg。
 * @param deg 当前方向对应的 yaw，单位为度；后续输出为原始 yaw 加 deg。
 * @return 无。
 * @note 同时复位四元数和积分，pitch/roll 立即为 0°；不采样，不负责修改 PID 目标。
 */
void Attitude_SetYawOffset(float deg);

#endif /* __ATTITUDE_H__ */

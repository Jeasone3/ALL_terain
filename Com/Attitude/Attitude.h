#ifndef __ATTITUDE_H__
#define __ATTITUDE_H__

#include "IMU_Config.h"

/* 欧拉角全局：yaw/pitch/roll，单位度；TIM4 10ms 中断里由 Attitude_Tick 更新。
   yaw 无磁力计参考会漂移(每分钟几度)，循迹/转弯秒级场景够用；
   pitch/roll 有重力参考，无累积漂移 */
extern Euler_struct g_euler;

/* 安装方向约定(默认 Z上 X前 Y右, 模块水平贴装)：
   若模块朝向不同，烧录后看静止加速度三轴(≈±16384 的轴是垂直轴)，
   在 Attitude.c 的 Attitude_Tick 里调整轴序或翻转符号即可。 */

/* 初始化：四元数复位[1,0,0,0]、PI 积分清零、yaw 偏置清零。
   在 Int_MPU6050_Init/Calibrate 之后、主循环前调用一次 */
void Attitude_Init(void);

/* 10ms 周期调用：读 g_imu_data 做 Mahony 6 轴更新，写 g_euler。
   放 TIM4 中断里 Int_MPU6050_Tick 之后(同中断上下文，读 g_imu_data 无竞争) */
void Attitude_Tick(void);

/**
 * @brief 重置：四元数复位 + 清 PI 积分 + yaw 偏置清零(输出 yaw 归 0)。
         循迹丢线起点 / 定角转弯起点调用，消除累积漂移
   * 
   */
void Attitude_Reset(void);


/**
 * @brief 以当前为基准设 yaw 偏置：重置四元数并使输出 yaw = deg。
 * 
 * @param deg 是输出yaw = deg
 */
void Attitude_SetYawOffset(float deg);

#endif /* __ATTITUDE_H__ */

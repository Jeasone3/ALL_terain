/**
 * @file Int_MPU6050.h
 * @brief 用预设六轴数据与成功标志替代 IIC 总线，运动控制实现仍使用真实源码。
 */
#ifndef MOTION_TEST_MPU6050_H
#define MOTION_TEST_MPU6050_H
#include "IMU_Config.h"

extern Gyro_Accel_Struct g_imu_data;
extern uint8_t g_imu_ready;

/**
 * @brief 在当前测试帧发布预设的采样结果。
 * @param 无。
 * @return 无。
 * @note 实现记录采样序号，供姿态与电机 mock 检查本帧调用顺序。
 */
void Int_MPU6050_Tick(void);

#endif

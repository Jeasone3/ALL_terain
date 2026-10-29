#ifndef __IMU_CONFIG_H__
#define __IMU_CONFIG_H__

#include <stdint.h>

/* 当前量程：加速度 ±2g，角速度 ±2000°/s。 */
#define IMU_ACCEL_LSB_PER_G    16384.0f
#define IMU_GYRO_LSB_PER_DPS   16.4f

/* 传感器坐标系下已减零偏的陀螺数据，单位仍为 ADC 计数的等效值。
 * 零偏均值可能包含小数，必须用 float 保留，否则会留下持续积分的残余角速度。
 * 除以 IMU_GYRO_LSB_PER_DPS 后得到 °/s，车体轴向仍由模块安装映射确定。 */
typedef struct 
{
    float gyro_x;
    float gyro_y;
    float gyro_z;
}Gyro_struct;

/* 加速度保持同一传感器坐标系下的 16 位有符号 ADC 原始值。 */
typedef struct 
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
}Accel_struct;

typedef struct 
{
    Gyro_struct gyro;
    Accel_struct accel;
}Gyro_Accel_Struct;

/* 欧拉角需由后续姿态解算得到，单位为度。 */
typedef struct 
{
    float yaw;
    float pitch;
    float roll;
}Euler_struct;

/* 帧间 yaw 突变剔除阈值(度)：当前帧相对上一帧最终输出的最短角差 ≥ 此值时，
 * 把角差 clamp 到 ±此值内再叠加到上帧输出(限幅式，不丢弃整帧)。
 * 详见 Attitude.c 中 Attitude_Tick。 */
#define ATT_YAW_SPIKE_THRESH  5.0f


#endif /* __IMU_CONFIG_H__ */

#ifndef __IMU_CONFIG_H__
#define __IMU_CONFIG_H__

#include <stdint.h>

/* 当前量程：加速度 ±2g，角速度 ±2000°/s。 */
#define IMU_ACCEL_LSB_PER_G    16384.0f
#define IMU_GYRO_LSB_PER_DPS   16.4f

/* 传感器坐标系下的有符号 ADC 原始值；车体轴向取决于模块安装方向。 */
typedef struct 
{
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
}Gyro_struct;

/* 加速度与角速度使用同一传感器坐标系。 */
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


#endif /* __IMU_CONFIG_H__ */

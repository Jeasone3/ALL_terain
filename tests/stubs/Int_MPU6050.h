#ifndef TEST_INT_MPU6050_H
#define TEST_INT_MPU6050_H

#include "main.h"

extern volatile uint8_t g_imu_ready;
void Int_MPU6050_Tick(void);

#endif

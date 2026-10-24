/**
 * @file iic.h
 * @brief 真实 MPU6050 驱动的宿主总线接口；测试提供寄存器数据，不访问 GPIO。
 */
#ifndef MOTION_TEST_IIC_H
#define MOTION_TEST_IIC_H

#include "main.h"

#define ACK  0u
#define NACK 1u

void I2C_Start(void);
void I2C_Stop(void);
void I2C_SendByte(uint8_t byte);
uint8_t I2C_ReadByte(uint8_t ack);
uint8_t I2C_WaitAck(void);

/* 用测试时钟和中断标志替代校准流程中的 HAL/CMSIS 操作。 */
void HAL_Delay(uint32_t milliseconds);
void __enable_irq(void);

#endif

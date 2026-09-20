/**
 * @file iic.c
 * @author Jeason
 * @brief 软件模拟iic
 * @version 0.1
 * @date 2026-09-20
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef __I2C_H__
#define __I2C_H__

#include "gpio.h"
#include "Delay_us.h"

//宏定义
#define ACK  0
#define NACK 1
#define IIC_PORT GPIOB
#define SCL_PIN  MPU_SCL_Pin
#define SDA_PIN  MPU_SDA_Pin

//控制SCL，SDA的输出高低电平
#define SCL_HIGH  HAL_GPIO_WritePin(IIC_PORT,SCL_PIN,GPIO_PIN_SET);
#define SCL_LOW   HAL_GPIO_WritePin(IIC_PORT, SCL_PIN, GPIO_PIN_RESET);
#define SDA_HIGH  HAL_GPIO_WritePin(IIC_PORT, SDA_PIN, GPIO_PIN_SET);
#define SDA_LOW   HAL_GPIO_WritePin(IIC_PORT, SDA_PIN, GPIO_PIN_RESET);
//读入操作
#define READ_SDA  HAL_GPIO_ReadPin(IIC_PORT, SDA_PIN)
//延时
#define I2C_DELAY Delay_us(10)

void I2C_Start(void);
void I2C_Stop(void);
//主机发送应答/非应答
void I2C_ACK(void);
void I2C_NACK(void);

//主机等待从设备发来应答
uint8_t I2C_Wait4Ack(void);

//主机发送一个字节
void I2C_SendByte(uint8_t byte);

//主机读取
uint8_t I2C_ReadByte(void);

#endif   

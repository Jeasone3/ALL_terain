/**
 * @file iic.h
 * @author Jeason
 * @brief 软件模拟iic（朴素 bit-bang，单从机场景）
 * @version 0.2
 * @date 2026-09-20
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef COM_IIC_IIC_H
#define COM_IIC_IIC_H

#include "gpio.h"
#include "Delay_us.h"

/* SCL=PB4, SDA=PB3，均配置为开漏输出，依赖外部上拉电阻 */
#define SCL_HIGH  HAL_GPIO_WritePin(MPU_SCL_GPIO_Port, MPU_SCL_Pin, GPIO_PIN_SET)
#define SCL_LOW   HAL_GPIO_WritePin(MPU_SCL_GPIO_Port, MPU_SCL_Pin, GPIO_PIN_RESET)
#define SDA_HIGH  HAL_GPIO_WritePin(MPU_SDA_GPIO_Port, MPU_SDA_Pin, GPIO_PIN_SET)
#define SDA_LOW   HAL_GPIO_WritePin(MPU_SDA_GPIO_Port, MPU_SDA_Pin, GPIO_PIN_RESET)
#define READ_SDA  HAL_GPIO_ReadPin(MPU_SDA_GPIO_Port, MPU_SDA_Pin)

/* 半周期 2us，SCL 约 250kHz，在 MPU6050 支持的 400kHz 内 */
#define I2C_DELAY Delay_us(2U)

#define ACK  0
#define NACK 1

/* 产生起始 / 重复起始信号 */
void I2C_Start(void);
/* 产生停止信号 */
void I2C_Stop(void);
/* 发送一个字节，MSB first；应答由调用方用 I2C_WaitAck 读取 */
void I2C_SendByte(uint8_t byte);
/* 读取一个字节，MSB first；ack=ACK 读完回 ACK，否则回 NACK，末尾释放 SDA */
uint8_t I2C_ReadByte(uint8_t ack);
/* 等待从机应答，返回 ACK 或 NACK；从机不应答时 SDA 被上拉为高 → NACK，不阻塞 */
uint8_t I2C_WaitAck(void);

#endif /* COM_IIC_IIC_H */

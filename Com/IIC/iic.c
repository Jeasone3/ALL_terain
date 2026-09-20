/**
 * @file iic.c
 * @author Jeason
 * @brief 软件模拟iic（朴素 bit-bang，单从机场景）
 * @version 0.2
 * @date 2026-09-20
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "iic.h"

/* SCL 为高时 SDA 由高变低，产生起始或重复起始信号 */
void I2C_Start(void)
{
    SDA_HIGH; I2C_DELAY;
    SCL_HIGH; I2C_DELAY;
    SDA_LOW;  I2C_DELAY;
    SCL_LOW;  I2C_DELAY;
}

/* SCL 为高时 SDA 由低变高，产生停止信号 */
void I2C_Stop(void)
{
    SCL_LOW;  SDA_LOW; I2C_DELAY;
    SCL_HIGH; I2C_DELAY;
    SDA_HIGH; I2C_DELAY;
}

/* MSB first 发送 8 位数据，不处理应答（由调用方 I2C_WaitAck） */
void I2C_SendByte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        if (byte & 0x80) SDA_HIGH; else SDA_LOW;
        I2C_DELAY;
        SCL_HIGH; I2C_DELAY;
        SCL_LOW;
        byte <<= 1;
    }
}

/* MSB first 读取 8 位；读完按 ack 回应答，最后释放 SDA */
uint8_t I2C_ReadByte(uint8_t ack)
{
    uint8_t data = 0;

    SDA_HIGH;                       /* 释放 SDA，让从机驱动 */
    for (uint8_t i = 0; i < 8; i++)
    {
        I2C_DELAY;
        SCL_HIGH; I2C_DELAY;
        data <<= 1;
        if (READ_SDA) data |= 0x01;
        SCL_LOW;
    }
    if (ack == ACK) SDA_LOW; else SDA_HIGH;   /* 回应答 */
    I2C_DELAY;
    SCL_HIGH; I2C_DELAY;
    SCL_LOW;  I2C_DELAY;
    SDA_HIGH;                          /* 释放 SDA */
    return data;
}

/* 读取从机应答：SCL 高电平期间采样 SDA，低=ACK，高=NACK */
uint8_t I2C_WaitAck(void)
{
    uint8_t ack;

    SDA_HIGH; I2C_DELAY;               /* 释放 SDA，由从机驱动 */
    SCL_HIGH; I2C_DELAY;
    ack = (READ_SDA == GPIO_PIN_RESET) ? ACK : NACK;
    SCL_LOW; I2C_DELAY;
    return ack;
}

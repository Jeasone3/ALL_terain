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

#include "iic.h"

#define I2C_SCL_TIMEOUT_US 100U
#define I2C_ACK_TIMEOUT_US 100U

static uint8_t i2c_bus_error;
static uint8_t i2c_active;

/* 释放开漏 SCL，等待引脚真正变高。 */
static uint8_t I2C_RaiseSCL(void)
{
    SCL_HIGH;
    for (uint16_t elapsed = 0; elapsed < I2C_SCL_TIMEOUT_US; elapsed++)
    {
        if (READ_SCL == GPIO_PIN_SET)
        {
            return 1U;
        }
        Delay_us(1U);
    }

    i2c_bus_error = 1U;
    return 0U;
}

/* 从机停在传输中途时，最多补发 9 个时钟并产生停止信号。 */
static uint8_t I2C_ClearBus(void)
{
    SDA_HIGH;
    for (uint8_t pulse = 0U; pulse < 9U; pulse++)
    {
        if (READ_SDA == GPIO_PIN_SET)
        {
            break;
        }

        SCL_LOW;
        I2C_DELAY;
        if (I2C_RaiseSCL() == 0U)
        {
            SDA_HIGH;
            return 0U;
        }
        I2C_DELAY;
    }

    SCL_LOW;
    SDA_LOW;
    I2C_DELAY;
    if (I2C_RaiseSCL() == 0U)
    {
        SDA_HIGH;
        return 0U;
    }
    I2C_DELAY;
    SDA_HIGH;
    I2C_DELAY;

    return ((READ_SCL == GPIO_PIN_SET) && (READ_SDA == GPIO_PIN_SET)) ? 1U : 0U;
}

/**
 * @brief 发出起始信号
 * 
 */
void I2C_Start(void)
{
    uint8_t first_start = (i2c_active == 0U);

    if (first_start != 0U)
    {
        i2c_bus_error = 0U;
    }
    i2c_active = 1U;
    if (i2c_bus_error != 0U)
    {
        return;
    }

    SDA_HIGH;
    I2C_DELAY;
    if (I2C_RaiseSCL() == 0U)
    {
        return;
    }
    I2C_DELAY;
    if (READ_SDA != GPIO_PIN_SET)
    {
        if ((first_start == 0U) || (I2C_ClearBus() == 0U))
        {
            i2c_bus_error = 1U; /* 总线恢复失败，或重复起始时 SDA 被拉低。 */
            SCL_LOW;
            return;
        }
    }
    /* SCL 为高电平时拉低 SDA，产生起始或重复起始信号。 */
    SDA_LOW;
    I2C_DELAY;
    SCL_LOW;
    I2C_DELAY;
}

/**
 * @brief 发出关闭信号
 * 
 */
void I2C_Stop(void)
{
    SCL_LOW;
    SDA_LOW;
    I2C_DELAY;
    if (I2C_RaiseSCL() != 0U)
    {
        I2C_DELAY;
    }
    /* SCL 为高电平时释放 SDA，产生停止信号。 */
    SDA_HIGH;
    I2C_DELAY;
    if ((READ_SCL != GPIO_PIN_SET) || (READ_SDA != GPIO_PIN_SET))
    {
        i2c_bus_error = 1U; /* 停止后总线未恢复空闲，数据可能无效。 */
    }
    i2c_active = 0U;
}

/**
 * @brief 发出应答信号
 * 
 */
void I2C_ACK(void)
{
    SCL_LOW;
    SDA_LOW;
    I2C_DELAY;
    if (I2C_RaiseSCL() != 0U)
    {
        I2C_DELAY;
    }
    SCL_LOW;
    I2C_DELAY;
    SDA_HIGH;
}

/**
 * @brief 发出非应答信号
 * 
 */
void I2C_NACK(void)
{
    SCL_LOW;
    SDA_HIGH;
    I2C_DELAY;
    if (I2C_RaiseSCL() != 0U)
    {
        I2C_DELAY;
    }
    SCL_LOW;
    I2C_DELAY;
}

/**
 * @brief 等待应答信号
 * 
 * @return uint8_t 
 */
uint8_t I2C_Wait4Ack(void)
{
    uint8_t result = NACK;

    SCL_LOW;
    SDA_HIGH;              /* 主机释放 SDA，由从机驱动第 9 个时钟。 */
    I2C_DELAY;
    if ((i2c_bus_error != 0U) || (I2C_RaiseSCL() == 0U))
    {
        SCL_LOW;
        return NACK;
    }

    I2C_DELAY;
    for (uint16_t elapsed = 0; elapsed < I2C_ACK_TIMEOUT_US; elapsed++)
    {
        if (READ_SDA == GPIO_PIN_RESET)
        {
            result = ACK;
            break;
        }
        Delay_us(1U);
    }

    SCL_LOW;
    I2C_DELAY;
    return result;
}

/**
 * @brief 发送一个字节
 * 
 * @param byte 
 */
void I2C_SendByte(uint8_t byte)
{
    if (i2c_bus_error != 0U)
    {
        return;
    }

    for (uint8_t bit = 0; bit < 8U; bit++)
    {
        SCL_LOW;
        if ((byte & 0x80U) != 0U)
        {
            SDA_HIGH;
        }
        else
        {
            SDA_LOW;
        }
        I2C_DELAY;
        if (I2C_RaiseSCL() == 0U)
        {
            SCL_LOW;
            SDA_HIGH;
            return;
        }
        I2C_DELAY;
        SCL_LOW;
        I2C_DELAY;
        byte <<= 1;
    }
    SDA_HIGH;              /* 第 8 位后释放 SDA，供从机应答。 */
}

/**
 * @brief 接收一个字节
 * 
 * @return uint8_t 
 */
uint8_t I2C_ReadByte(void)
{
    uint8_t data = 0U;

    SCL_LOW;
    SDA_HIGH;              /* 读之前必须释放开漏 SDA。 */
    if (i2c_bus_error != 0U)
    {
        return 0xFFU;
    }

    for (uint8_t bit = 0; bit < 8U; bit++)
    {
        SCL_LOW;
        SDA_HIGH;
        I2C_DELAY;
        if (I2C_RaiseSCL() == 0U)
        {
            SCL_LOW;
            return 0xFFU;
        }
        I2C_DELAY;
        data <<= 1;
        if (READ_SDA == GPIO_PIN_SET)
        {
            data |= 0x01U;
        }
        SCL_LOW;
        I2C_DELAY;
    }

    return data;
}

uint8_t I2C_HasBusError(void)
{
    return i2c_bus_error;
}

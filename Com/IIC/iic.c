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


/**
 * @brief 发出起始信号
 * 
 */
void I2C_Start(void)
{
    /*将总线空闲*/
    SCL_HIGH;
    SDA_HIGH;
    I2C_DELAY;

    /* 时钟线高电平，数据线拉低。产生起始信号 */
    SDA_LOW;
    I2C_DELAY;

    /* 时钟线拉低，释放总线 */
    SCL_LOW;
    I2C_DELAY;

}

/**
 * @brief 发出关闭信号
 * 
 */
void I2C_Stop(void)
{
    /* 确保总线都是低电平 */
    SCL_LOW;
    SDA_LOW;
    I2C_DELAY;

    /* 时钟线拉高，开始采用 */
    SCL_HIGH;
    I2C_DELAY;

    /* 数据线拉高，产生一个终止信号 */
    SDA_HIGH;
    I2C_DELAY;
    
}

/**
 * @brief 发出应答信号
 * 
 */
void I2C_ACK(void)
{
    /* 准备发出信号 */
    SCL_LOW;
    SDA_HIGH;
    I2C_DELAY;

    /* 准备ACK信号 */
    SDA_LOW;
    I2C_DELAY;
    
    /* 采样ACK */
    SCL_HIGH;
    I2C_DELAY;

    /* 结束采样 */
    SCL_LOW;
    I2C_DELAY;

    /* 释放数据线 */
    SDA_HIGH;
}

/**
 * @brief 发出非应答信号
 * 
 */
void I2C_NACK(void)
{
    /* 准备发出NACK信号 */
    SCL_LOW;
    SDA_HIGH;
    I2C_DELAY;
    
    /* 开始采样 */
    SCL_HIGH;
    I2C_DELAY;

    /* 结束采样 */
    SCL_LOW;
    I2C_DELAY;
    
    //SDA没有拉低，一直处于释放状态
}

/**
 * @brief 等待应答信号
 * 
 * @return uint8_t 
 */
uint8_t I2C_Wait4Ack(void)
{
    //1.SDA拉高释放总线
    SCL_LOW;
    SDA_HIGH;
    I2C_DELAY;

    //2.scl拉高，数据采样
    SCL_HIGH;
    I2C_DELAY;

    //3.读取sda上的数据
    uint16_t ack = READ_SDA;

    //scl拉低结束数据采样
    SCL_LOW;
    I2C_DELAY;  

    return ack? NACK: ACK;
}

/**
 * @brief 发送一个字节
 * 
 * @param byte 
 */
void I2C_SendByte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        /* SCL拉低等待数据反转 */
        SCL_LOW;
        SDA_LOW;
        I2C_DELAY;

        /* 判断数据 高低 */
        if (byte & 0x80)
        {
            SDA_HIGH;
        }else{
            SDA_LOW;
        }
        I2C_DELAY;
        
        /* 采样 */
        SCL_HIGH;
        I2C_DELAY;

        /* 结束采样 */
        SCL_LOW;
        I2C_DELAY;

        //右移一位，准备下一个数据
        byte <<= 1;
    }
    
}

/**
 * @brief 接收一个字节
 * 
 * @return uint8_t 
 */
uint8_t I2C_ReadByte(void)
{
    //定义一个变量用来保存接收的数据
    uint8_t data = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        //scl拉低，等待数据翻转
        SCL_LOW;
        I2C_DELAY;

        //scl拉高，开始采样
        SCL_HIGH;
        I2C_DELAY;

        //数据采样
        data <<= 1;//先左移，新存入的永远再最低位
        if(READ_SDA){
            data |= 0x01;//存入最低位，每次左移一位
        }

        //scl拉低结束采样
        SCL_LOW;
        I2C_DELAY;
    }
    
    return data;
}

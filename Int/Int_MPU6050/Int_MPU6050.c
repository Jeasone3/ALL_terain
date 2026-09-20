#include "Int_MPU6050.h"

#define MPU6050_WHO_AM_I_VALUE 0x68U
#define MPU6050_FRAME_LENGTH   14U

static uint8_t mpu6050_ready = 0U;

static uint8_t MPU6050_Write_Reg(uint8_t reg, uint8_t value)
{
    I2C_Start();
    I2C_SendByte(MPU6050_ADDR_WRITE);
    if (I2C_Wait4Ack() != ACK) goto nack;
    I2C_SendByte(reg);
    if (I2C_Wait4Ack() != ACK) goto nack;
    I2C_SendByte(value);
    if (I2C_Wait4Ack() != ACK) goto nack;
    I2C_Stop();
    return I2C_HasBusError() ? 0U : 1U;

nack:
    I2C_Stop();
    return 0U;
}

static uint8_t MPU6050_Read_Regs(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    uint8_t i;

    if (buffer == 0 || length == 0U) return 0U;

    I2C_Start();
    I2C_SendByte(MPU6050_ADDR_WRITE);
    if (I2C_Wait4Ack() != ACK) goto nack;
    I2C_SendByte(reg);
    if (I2C_Wait4Ack() != ACK) goto nack;

    I2C_Start(); /* 重复起始信号，保持当前寄存器地址。 */
    I2C_SendByte(MPU6050_ADDR_READ);
    if (I2C_Wait4Ack() != ACK) goto nack;

    for (i = 0U; i < length; ++i)
    {
        buffer[i] = I2C_ReadByte();
        if (i + 1U < length) I2C_ACK();
        else I2C_NACK();
    }
    I2C_Stop();
    return I2C_HasBusError() ? 0U : 1U;

nack:
    I2C_Stop();
    return 0U;
}

static uint8_t MPU6050_Read_Reg(uint8_t reg, uint8_t *value)
{
    return MPU6050_Read_Regs(reg, value, 1U);
}

static uint8_t MPU6050_Write_Verified(uint8_t reg, uint8_t value)
{
    uint8_t actual;
    return MPU6050_Write_Reg(reg, value) &&
           MPU6050_Read_Reg(reg, &actual) && actual == value;
}

/* 将高字节在前的补码数据转换为有符号采样值。 */
static int16_t MPU6050_Sample(uint8_t high, uint8_t low)
{
    int32_t value = ((int32_t)high << 8) | low;
    if (value >= 0x8000L) value -= 0x10000L;
    return (int16_t)value;
}

uint8_t Int_MPU6050_Init(void)
{
    uint8_t value;
    uint16_t attempt;

    mpu6050_ready = 0U;
    if (!MPU6050_Read_Reg(MPU_DEVICE_ID_REG, &value) ||
        value != MPU6050_WHO_AM_I_VALUE) return 0U;

    if (!MPU6050_Write_Reg(MPU_PWR_MGMT1_REG, 0x80U)) return 0U;
    HAL_Delay(100U); /* 等待芯片复位完成后再访问寄存器。 */

    for (attempt = 0U; attempt < 100U; ++attempt)
    {
        if (MPU6050_Read_Reg(MPU_PWR_MGMT1_REG, &value) &&
            (value & 0x80U) == 0U) break;
        HAL_Delay(1U);
    }
    if (attempt == 100U) return 0U;

    /* 使用 X 轴陀螺仪 PLL 时钟，唤醒全部六轴，关闭 FIFO、辅助 IIC 主机和中断。
       低通滤波设为 3，对应约 42～44 Hz 带宽，适用于应用层 100 Hz 读取频率。
       内部采样率为 1 kHz，分频值为 1，输出采样率为 500 Hz。
       陀螺仪量程为 ±2000°/s，加速度计量程为 ±2g。 */
    if (!MPU6050_Write_Verified(MPU_PWR_MGMT1_REG, 0x01U) ||
        !MPU6050_Write_Verified(MPU_PWR_MGMT2_REG, 0x00U) ||
        !MPU6050_Write_Verified(MPU_USER_CTRL_REG, 0x00U) ||
        !MPU6050_Write_Verified(MPU_FIFO_EN_REG, 0x00U) ||
        !MPU6050_Write_Verified(MPU_INT_EN_REG, 0x00U) ||
        !MPU6050_Write_Verified(MPU_CFG_REG, 0x03U) ||
        !MPU6050_Write_Verified(MPU_SAMPLE_RATE_REG, 0x01U) ||
        !MPU6050_Write_Verified(MPU_GYRO_CFG_REG, 0x18U) ||
        !MPU6050_Write_Verified(MPU_ACCEL_CFG_REG, 0x00U)) return 0U;

    mpu6050_ready = 1U;
    return 1U;
}

uint8_t Int_MPU6050_Get_Data(Gyro_Accel_Struct *data)
{
    uint8_t frame[MPU6050_FRAME_LENGTH];
    Gyro_Accel_Struct sample;

    if (data == 0 || !mpu6050_ready ||
        !MPU6050_Read_Regs(MPU_ACCEL_XOUTH_REG, frame, MPU6050_FRAME_LENGTH))
        return 0U;

    sample.accel.accel_x = MPU6050_Sample(frame[0], frame[1]);
    sample.accel.accel_y = MPU6050_Sample(frame[2], frame[3]);
    sample.accel.accel_z = MPU6050_Sample(frame[4], frame[5]);
    /* frame[6..7] 为温度数据，六轴结构体不包含温度字段。 */
    sample.gyro.gyro_x = MPU6050_Sample(frame[8], frame[9]);
    sample.gyro.gyro_y = MPU6050_Sample(frame[10], frame[11]);
    sample.gyro.gyro_z = MPU6050_Sample(frame[12], frame[13]);
    *data = sample;
    return 1U;
}

uint8_t Int_MPU6050_Get_Gyro(Gyro_struct *gyro)
{
    Gyro_Accel_Struct data;
    if (gyro == 0 || !Int_MPU6050_Get_Data(&data)) return 0U;
    *gyro = data.gyro;
    return 1U;
}

uint8_t Int_MPU6050_Get_Accel(Accel_struct *accel)
{
    Gyro_Accel_Struct data;
    if (accel == 0 || !Int_MPU6050_Get_Data(&data)) return 0U;
    *accel = data.accel;
    return 1U;
}

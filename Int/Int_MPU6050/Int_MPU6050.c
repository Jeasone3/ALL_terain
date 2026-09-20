#include "Int_MPU6050.h"

#define MPU6050_WHO_AM_I_VALUE 0x68U
#define MPU6050_FRAME_LENGTH   14U

/* 供其他模块读取的六轴数据与通信状态；Tick 在 TIM4 中断里更新 */
Gyro_Accel_Struct g_imu_data = {0};
uint8_t g_imu_ready = 0;

/* 写一个寄存器；任一 ACK 失败即发停止并返回 0 */
static uint8_t MPU6050_Write_Reg(uint8_t reg, uint8_t value)
{
    I2C_Start();
    I2C_SendByte(MPU6050_ADDR_WRITE);
    if (I2C_WaitAck() != ACK) { I2C_Stop(); return 0; }
    I2C_SendByte(reg);
    if (I2C_WaitAck() != ACK) { I2C_Stop(); return 0; }
    I2C_SendByte(value);
    if (I2C_WaitAck() != ACK) { I2C_Stop(); return 0; }
    I2C_Stop();
    return 1;
}

/* 连续读 len 字节到 buf；重复起始后 burst read */
static uint8_t MPU6050_Read_Regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (buf == 0 || len == 0) return 0;

    I2C_Start();
    I2C_SendByte(MPU6050_ADDR_WRITE);
    if (I2C_WaitAck() != ACK) { I2C_Stop(); return 0; }
    I2C_SendByte(reg);
    if (I2C_WaitAck() != ACK) { I2C_Stop(); return 0; }

    I2C_Start();                    /* 重复起始，切换为读 */
    I2C_SendByte(MPU6050_ADDR_READ);
    if (I2C_WaitAck() != ACK) { I2C_Stop(); return 0; }

    for (uint8_t i = 0; i < len; i++)
        buf[i] = I2C_ReadByte(i + 1 < len ? ACK : NACK);
    I2C_Stop();
    return 1;
}

/* 高字节在前的补码转 int16（位模式即补码，直接组合即可） */
static int16_t MPU6050_Sample(uint8_t hi, uint8_t lo)
{
    return (int16_t)(((uint16_t)hi << 8) | lo);
}

uint8_t Int_MPU6050_Init(void)
{
    uint8_t id;

    g_imu_ready = 0;
    /* 读 WHO_AM_I 确认设备在线 */
    if (!MPU6050_Read_Regs(MPU_DEVICE_ID_REG, &id, 1) ||
        id != MPU6050_WHO_AM_I_VALUE)
        return 0;

    /* 复位，等待内部寄存器恢复默认 */
    if (!MPU6050_Write_Reg(MPU_PWR_MGMT1_REG, 0x80)) return 0;
    HAL_Delay(100);

    /* 时钟源=X 轴陀螺 PLL、唤醒；六轴唤醒；关 FIFO/辅助 IIC；关中断(轮询)
       DLPF=3(~42Hz)；分频=1→500Hz；陀螺±2000°/s；加速度±2g */
    if (!MPU6050_Write_Reg(MPU_PWR_MGMT1_REG, 0x01) ||
        !MPU6050_Write_Reg(MPU_PWR_MGMT2_REG, 0x00) ||
        !MPU6050_Write_Reg(MPU_USER_CTRL_REG, 0x00) ||
        !MPU6050_Write_Reg(MPU_FIFO_EN_REG,   0x00) ||
        !MPU6050_Write_Reg(MPU_INT_EN_REG,    0x00) ||
        !MPU6050_Write_Reg(MPU_CFG_REG,       0x03) ||
        !MPU6050_Write_Reg(MPU_SAMPLE_RATE_REG, 0x01) ||
        !MPU6050_Write_Reg(MPU_GYRO_CFG_REG,  0x18) ||
        !MPU6050_Write_Reg(MPU_ACCEL_CFG_REG, 0x00))
        return 0;

    g_imu_ready = 1;
    return 1;
}

uint8_t Int_MPU6050_Get_Data(Gyro_Accel_Struct *data)
{
    uint8_t frame[MPU6050_FRAME_LENGTH];
    Gyro_Accel_Struct sample;

    if (data == 0 || !g_imu_ready)
        return 0;
    if (!MPU6050_Read_Regs(MPU_ACCEL_XOUTH_REG, frame, MPU6050_FRAME_LENGTH))
        return 0;

    sample.accel.accel_x = MPU6050_Sample(frame[0],  frame[1]);
    sample.accel.accel_y = MPU6050_Sample(frame[2],  frame[3]);
    sample.accel.accel_z = MPU6050_Sample(frame[4],  frame[5]);
    /* frame[6..7] 是温度，六轴结构体不存 */
    sample.gyro.gyro_x = MPU6050_Sample(frame[8],  frame[9]);
    sample.gyro.gyro_y = MPU6050_Sample(frame[10], frame[11]);
    sample.gyro.gyro_z = MPU6050_Sample(frame[12], frame[13]);
    *data = sample;
    return 1;
}

/* 10ms 周期调用：ready 时读取六轴并刷新 g_imu_ready；失败后停止读取，
   由 main 主循环 1s 后重试 Int_MPU6050_Init 恢复 */
void Int_MPU6050_Tick(void)
{
    if (g_imu_ready)
        g_imu_ready = Int_MPU6050_Get_Data(&g_imu_data);
}

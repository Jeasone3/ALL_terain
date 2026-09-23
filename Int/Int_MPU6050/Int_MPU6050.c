#include "Int_MPU6050.h"

#define MPU6050_WHO_AM_I_VALUE 0x68U
#define MPU6050_FRAME_LENGTH   14U  //imu数据帧长度 即 ADC精度

/* 供其他模块读取的六轴数据与通信状态；Tick 在 TIM4 中断里更新 */
//加速度值和欧拉角
Gyro_Accel_Struct g_imu_data = {0};
//通信状态
/**
 * @brief IMU 通信状态 0 不能通讯，1=正常
 * 
 */
uint8_t g_imu_ready = 0; 


/* 陀螺零偏：Calibrate 静止取均值后写入，Get_Data 里减掉。初值 0 = 未校准，
   未校准时减 0 等于原始值，行为不变 */
static int16_t s_bias_gx = 0, s_bias_gy = 0, s_bias_gz = 0;



//--------------------------- 通讯 --------------------------------/

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

//假写真读
/**
 * @brief 连续读 len 字节到 buf；重复起始后 burst read
 * 
 * @param reg 寄存器地址
 * @param buf 数据传地址
 * @param len   长度
 * @return uint8_t 成功返回1 ， 失败返回0
 */
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
        buf[i] = I2C_ReadByte(i + 1 < len ? ACK : NACK); // 最后一个字节主机发送NACK ！！！
    I2C_Stop();
    return 1;
}

//-------------------------- 初始化 --------------------------------/

uint8_t Int_MPU6050_Init(void)
{
    uint8_t id;

    g_imu_ready = 0;
    /* 读 WHO_AM_I 确认设备在线  保证后续iic通讯正常*/
    if (!MPU6050_Read_Regs(MPU_DEVICE_ID_REG, &id, 1) ||
        id != MPU6050_WHO_AM_I_VALUE)
        return 0;

    /* 复位，等待内部寄存器恢复默认 */
    if (!MPU6050_Write_Reg(MPU_PWR_MGMT1_REG, 0x80)) return 0;   //将复位位置1 其余都置0
    HAL_Delay(100);                                             //延时保证设备复位成功

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

    g_imu_ready = 1; //可以通讯
    return 1;
}




//-------------------------- 校准 --------------------------------//

/* 高字节在前的补码转 int16（位模式即补码，直接组合即可） */
static int16_t MPU6050_Sample(uint8_t higth, uint8_t low)
{
    return (int16_t)(((uint16_t)higth << 8) | low);
}

/* 校准：静止时取均值，减去零偏 */
/* 静止判据阈值：前后帧加速度差 < 200 LSB（≈0.012g）视为静止；
   超时 3s 仍抖动则放弃校准，避免死等 */
#define IMU_STILL_THR        200
#define IMU_CALI_TIMEOUT_MS  3000U

/* 静止判据通过后，采 samples 帧陀螺取均值作为零偏。
   1) 先等"静止"：连续 100 帧加速度前后帧差 < IMU_STILL_THR 才认为车体稳定；
   2) 再采 n 帧陀螺求均值。每帧间隔 10ms 与 Tick 同节奏，避免重复采样；
      samples=0 取默认 100 帧≈1s。直接读寄存器而非 Get_Data，重新校准不会
      叠加旧偏置。写偏置时关中断，避免 Tick 读到半更新的 bias。
   成功返回 1；通信失败或超时返回 0（偏置保持旧值） */
   
   /**
    * @brief   零偏校准 
    * 
    * @param samples  校准帧数 取0 ->> 100帧 1秒
    * @return uint8_t 
    */
uint8_t Int_MPU6050_Calibrate(uint16_t samples)
{
    uint8_t frame[MPU6050_FRAME_LENGTH];                    //数据帧
    int32_t sx = 0, sy = 0, sz = 0;                         //采样值
    int16_t ax0, ay0, az0, ax1, ay1, az1, dax, day, daz;    
    uint16_t n = samples ? samples : 100U;                  //samples=0 ->> 100帧 1秒 
    uint16_t still = 0;                                     //静止计数
    uint32_t t0 = HAL_GetTick();                            //计时

    if (!g_imu_ready) return 0;                             //初始化失败 or 通信失败

    /* 1) 静止判据：连续 100 帧前后帧加速度差均小于阈值 */
    if (!MPU6050_Read_Regs(MPU_ACCEL_XOUTH_REG, frame, MPU6050_FRAME_LENGTH)) return 0;
    //获取加速度
    ax0 = MPU6050_Sample(frame[0], frame[1]);
    ay0 = MPU6050_Sample(frame[2], frame[3]);
    az0 = MPU6050_Sample(frame[4], frame[5]);
    
    // 静止判断
    while (still < 100U) {
        //设置超时等待
        if ((uint32_t)(HAL_GetTick() - t0) > IMU_CALI_TIMEOUT_MS) return 0;
        HAL_Delay(10);
        if (!MPU6050_Read_Regs(MPU_ACCEL_XOUTH_REG, frame, MPU6050_FRAME_LENGTH))
            return 0;
        ax1 = MPU6050_Sample(frame[0], frame[1]);
        ay1 = MPU6050_Sample(frame[2], frame[3]);
        az1 = MPU6050_Sample(frame[4], frame[5]);
        dax = ax1 - ax0; if (dax < 0) dax = -dax; //取绝对值
        day = ay1 - ay0; if (day < 0) day = -day;
        daz = az1 - az0; if (daz < 0) daz = -daz;
        if (dax < IMU_STILL_THR && day < IMU_STILL_THR && daz < IMU_STILL_THR)
            still++;
        else
            still = 0;
        ax0 = ax1; ay0 = ay1; az0 = az1;
    }

    /* 2) 静止确认后采 n 帧陀螺求均值 */
    for (uint16_t i = 0; i < n; i++) {
        if (!MPU6050_Read_Regs(MPU_ACCEL_XOUTH_REG, frame, MPU6050_FRAME_LENGTH))
            return 0;
        sx += MPU6050_Sample(frame[8],  frame[9]);
        sy += MPU6050_Sample(frame[10], frame[11]);
        sz += MPU6050_Sample(frame[12], frame[13]);
        HAL_Delay(10);
    }

    __disable_irq(); //关中断 避免 Tick 读到半更新的 bias
    s_bias_gx = (int16_t)(sx / n);
    s_bias_gy = (int16_t)(sy / n);
    s_bias_gz = (int16_t)(sz / n);
    __enable_irq();
    return 1;       //校准成功
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
    /* frame[6..7] 是温度，六轴结构体不存；陀螺减零偏见 Calibrate */
    sample.gyro.gyro_x = MPU6050_Sample(frame[8],  frame[9])   - s_bias_gx;
    sample.gyro.gyro_y = MPU6050_Sample(frame[10], frame[11]) - s_bias_gy;
    sample.gyro.gyro_z = MPU6050_Sample(frame[12], frame[13]) - s_bias_gz;
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

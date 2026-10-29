#ifndef __INT_MPU6050_H__
#define __INT_MPU6050_H__

#include "iic.h"
#include "IMU_Config.h"

/* 7 位地址：AD0 接低为 0x68，接高时改为 0x69 */
#define MPU6050_ADDR       0x68U
#define MPU6050_ADDR_WRITE ((uint8_t)(MPU6050_ADDR << 1U))
#define MPU6050_ADDR_READ  ((uint8_t)((MPU6050_ADDR << 1U) | 1U))

/* 实际使用的寄存器；未列出的请查阅 MPU6000/6050 Register Map */
#define MPU_SAMPLE_RATE_REG 0x19   /* 采样率分频：输出率=1kHz/(1+SMPLRT_DIV) */
#define MPU_CFG_REG         0x1A   /* DLPF 配置 + 外部同步 */
#define MPU_GYRO_CFG_REG    0x1B   /* 陀螺量程 FS_SEL */
#define MPU_ACCEL_CFG_REG   0x1C   /* 加速度量程 AFS_SEL */
#define MPU_FIFO_EN_REG     0x23   /* FIFO 使能 */
#define MPU_INT_EN_REG      0x38   /* 中断使能 */
#define MPU_ACCEL_XOUTH_REG 0x3B   /* 加速度 X 高字节，14 字节连续读起点 */
#define MPU_USER_CTRL_REG   0x6A   /* FIFO / 辅助 IIC 主控制 */
#define MPU_PWR_MGMT1_REG   0x6B   /* 电源管理 1：时钟源、睡眠、复位 */
#define MPU_PWR_MGMT2_REG   0x6C   /* 电源管理 2：单轴待机 */
#define MPU_DEVICE_ID_REG   0x75   /* WHO_AM_I，应回 0x68 */

/* 六轴测量值与通信状态：加速度为原始 ADC 值，陀螺为保留小数的已扣偏计数值。
   由 Int_MPU6050_Tick 在 TIM4 中断里更新，其他模块可直接读。 */
extern Gyro_Accel_Struct g_imu_data;
extern uint8_t g_imu_ready;

/* 成功返回 1；设备不存在、IIC 错误或配置失败返回 0 */
uint8_t Int_MPU6050_Init(void);

/* 连续读 14 字节，保证六轴数据来自同一采样帧；失败返回 0 */
uint8_t Int_MPU6050_Get_Data(Gyro_Accel_Struct *data);

/* 设备静止时调用：采 samples 帧陀螺取均值作为零偏；之后 Get_Data 返回的
   陀螺值已减零偏，均值和扣偏结果均保留小数。
   samples=0 取默认 100 帧(每帧间隔10ms≈1s)。成功返回 1 */
uint8_t Int_MPU6050_Calibrate(uint16_t samples);

/* 项目入口非阻塞校准结果。Step 只在唯一的 10 ms 控制入口调用。 */
typedef enum {
    IMU_CAL_BUSY = 0,
    IMU_CAL_DONE,
    IMU_CAL_READ_ERROR,
    IMU_CAL_TIMEOUT,
    IMU_CAL_CANCELLED
} IMU_CalibrationStatus;

/**
 * @brief 开始停车校准，只清空采样窗口，不读总线、不改变旧零偏。
 * @note 先让运动层输出零 PWM，再每 10 ms 调用一次 Step；不能同时调用普通 Tick。
 */
void Int_MPU6050_CalibrationStart(void);

/**
 * @brief 推进一帧校准，最多读取一次六轴数据，不包含延时和等待循环。
 * @param gravity_mean 成功时接收传感器坐标系的平均加速度，单位为原始计数。
 * @return 当前校准状态；成功才一次性更新三轴浮点零偏。
 * @note 连续静止约 1 秒后采集约 1 秒，采集中运动则重新等待；总期限为 500 帧。
 *       超时、通信失败或取消均保留旧零偏，完成后的后续 Step 不再读取总线。
 */
IMU_CalibrationStatus Int_MPU6050_CalibrationStep(Accel_struct *gravity_mean);

/** @brief 取消尚未提交的校准，防止停止或接管后继续采样并自动完成。 */
void Int_MPU6050_CalibrationCancel(void);

/* 10ms 周期调用：ready 时读取六轴并刷新 g_imu_ready；放定时中断里 */
void Int_MPU6050_Tick(void);

#endif /* __INT_MPU6050_H__ */

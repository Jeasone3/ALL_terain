/**
 * @file test_imu_bias.c
 * @brief 链接真实 MPU6050 驱动和姿态解算，检查小数零偏、静止漂移及重复校准。
 * @note 只替代 IIC 和时钟；通过真实 14 字节数据读取、校准和扣偏链路验证。
 */
#include "../../Int/Int_MPU6050/Int_MPU6050.h"
#include "Attitude.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL gyro bias line %d: %s\n", __LINE__, #condition); \
        exit(EXIT_FAILURE); \
    } \
} while (0)

/* 相邻 ADC 帧相差一个计数，真实静止均值分别为 20.5、-12.5、7.5 LSB。 */
static int16_t s_sensor_bias[3] = {20, -13, 7};
static uint8_t s_frame[14];
static uint8_t s_sent, s_register, s_read_index;
static uint8_t s_ack = ACK, s_force_high;
static unsigned int s_frame_count;
/* 注入一次事务的指定 ACK 阶段失败，并记录事务边界，检查完整重试。 */
static unsigned int s_ack_count, s_fail_ack, s_starts, s_stops;
static uint32_t s_time_ms, s_primask;
/* 模拟相邻帧的加速度变化，验证放宽后的静止阈值和超时后的再次校准。 */
static int16_t s_accel_step;

/**
 * @brief 把带符号 ADC 值放入传感器使用的高字节在前的数据帧。
 * @param offset 数据帧中的高字节位置。
 * @param value 需要发布的 16 位 ADC 值。
 */
static void put_sample(unsigned int offset, int16_t value)
{
    uint16_t bits = (uint16_t)value;
    s_frame[offset] = (uint8_t)(bits >> 8);
    s_frame[offset + 1u] = (uint8_t)bits;
}

void I2C_Start(void) { s_sent = 0u; ++s_starts; }
void I2C_Stop(void) { ++s_stops; }
uint8_t I2C_WaitAck(void)
{
    ++s_ack_count;
    if (s_ack_count == s_fail_ack) return NACK;
    return s_ack;
}
uint32_t HAL_GetTick(void) { return s_time_ms; }
void HAL_Delay(uint32_t milliseconds) { s_time_ms += milliseconds; }
uint32_t __get_PRIMASK(void) { return s_primask; }
void __disable_irq(void) { s_primask = 1u; }
void __enable_irq(void) { s_primask = 0u; }
void __set_PRIMASK(uint32_t mask) { s_primask = mask; }

/**
 * @brief 模拟寄存器寻址，并在读地址到达时生成一帧静止传感器数据。
 * @param byte 主机发送的地址或寄存器字节。
 * @note 加速度恒为水平重力；三轴陀螺在相邻整数间交替，均值带 0.5 LSB。
 */
void I2C_SendByte(uint8_t byte)
{
    if (s_sent == 0u && byte == MPU6050_ADDR_READ) {
        int16_t high = (int16_t)(s_force_high != 0u || (++s_frame_count & 1u) != 0u);
        s_read_index = 0u;
        put_sample(0u, (s_frame_count & 1u) != 0u ? s_accel_step : 0);
        put_sample(2u, 0);
        put_sample(4u, 16384);
        put_sample(8u, (int16_t)(s_sensor_bias[0] + high));
        put_sample(10u, (int16_t)(s_sensor_bias[1] + high));
        put_sample(12u, (int16_t)(s_sensor_bias[2] + high));
    } else if (s_sent == 1u) {
        s_register = byte;
    }
    s_sent++;
}

uint8_t I2C_ReadByte(uint8_t ack)
{
    CHECK(s_register == MPU_ACCEL_XOUTH_REG && s_read_index < sizeof(s_frame));
    CHECK(ack == (s_read_index == sizeof(s_frame) - 1u ? NACK : ACK));
    return s_frame[s_read_index++];
}

/**
 * @brief 检查扣偏后的三轴静止均值，以及十秒真实解算产生的 yaw 残差。
 * @note 仅零偏为理想常量的合成场景，不代表 MPU6050 的温漂或实车精度。
 */
static void check_stationary_bias(void)
{
    float sx = 0.0f, sy = 0.0f, sz = 0.0f;
    unsigned int frame;
    Attitude_Init();
    for (frame = 0u; frame < 1000u; frame++) {
        Int_MPU6050_Tick();
        CHECK(g_imu_ready != 0u);
        sx += g_imu_data.gyro.gyro_x;
        sy += g_imu_data.gyro.gyro_y;
        sz += g_imu_data.gyro.gyro_z;
        Attitude_Tick();
    }
    printf("stationary mean LSB: %.4f,%.4f,%.4f; yaw after 10s: %.5f deg\n",
           (double)(sx / 1000.0f), (double)(sy / 1000.0f), (double)(sz / 1000.0f),
           (double)g_euler.yaw);
    CHECK(fabsf(sx / 1000.0f) < 0.0001f);
    CHECK(fabsf(sy / 1000.0f) < 0.0001f);
    CHECK(fabsf(sz / 1000.0f) < 0.0001f);
    CHECK(fabsf(g_euler.yaw) < 0.02f);
}

/**
 * @brief 注入三个寻址阶段的偶发 NACK，验证 STOP 后从完整寻址重试。
 * @note 连续失败保留旧数据和 ready，恢复后无需重新初始化即可继续采样。
 */
static void check_read_retry(void)
{
    unsigned int phase;
    Gyro_Accel_Struct saved;
    for (phase = 1u; phase <= 3u; ++phase) {
        s_ack_count = s_starts = s_stops = 0u;
        s_fail_ack = phase;
        CHECK(Int_MPU6050_Tick() == 1u);
        CHECK(s_ack_count == phase + 3u && s_stops == 2u);
        CHECK(s_starts == (phase == 3u ? 4u : 3u));
        CHECK(s_read_index == 14u);
        CHECK(g_imu_ready == 1u);
    }
    s_fail_ack = 0u;
    saved = g_imu_data;
    s_ack = NACK;
    s_ack_count = s_starts = s_stops = 0u;
    CHECK(Int_MPU6050_Tick() == 0u);
    CHECK(s_ack_count == 2u && s_starts == 2u && s_stops == 2u);
    CHECK(g_imu_ready == 1u);
    CHECK(g_imu_data.accel.accel_x == saved.accel.accel_x);
    CHECK(g_imu_data.accel.accel_y == saved.accel.accel_y);
    CHECK(g_imu_data.accel.accel_z == saved.accel.accel_z);
    CHECK(g_imu_data.gyro.gyro_x == saved.gyro.gyro_x);
    CHECK(g_imu_data.gyro.gyro_y == saved.gyro.gyro_y);
    CHECK(g_imu_data.gyro.gyro_z == saved.gyro.gyro_z);
    s_ack = ACK;
    CHECK(Int_MPU6050_Tick() == 1u);
    g_imu_ready = 0u;
    s_ack_count = 0u;
    CHECK(Int_MPU6050_Tick() == 0u && s_ack_count == 0u);
    g_imu_ready = 1u;
    puts("PASS full transaction retry at each ACK stage and failed frame preservation");
}

int main(void)
{
    Gyro_Accel_Struct sample;
    g_imu_ready = 1u;
    check_read_retry();
    /* 399 LSB 可连续通过；等于 400 时不满足严格小于判据，约 3 秒后失败。
     * 超时后立即改为可接受的波动再调用，应在约 2 秒内完成，无需重新初始化。 */
    s_accel_step = 399;
    CHECK(Int_MPU6050_Calibrate(100u) == 1u);
    CHECK(Int_MPU6050_CalibrationTimedOut() == 0u);
    s_accel_step = 400;
    {
        uint32_t started = s_time_ms;
        CHECK(Int_MPU6050_Calibrate(100u) == 0u);
        CHECK(Int_MPU6050_CalibrationTimedOut() == 1u);
        CHECK(s_time_ms - started > 3000u && s_time_ms - started <= 3010u);
        s_accel_step = 300;
        started = s_time_ms;
        CHECK(Int_MPU6050_Calibrate(100u) == 1u);
        CHECK(Int_MPU6050_CalibrationTimedOut() == 0u);
        CHECK(s_time_ms - started == 2000u);
    }
    s_accel_step = 0;
    puts("PASS 400 LSB stillness boundary, 3s timeout and immediate recalibration");
    CHECK(Int_MPU6050_Calibrate(100u) != 0u);
    check_stationary_bias();

    /* 小于一个 ADC 计数的扣偏结果必须保留，不能当成角速度死区直接清零。 */
    s_force_high = 1u;
    CHECK(Int_MPU6050_Get_Data(&sample) != 0u);
    CHECK(fabsf(sample.gyro.gyro_z - 0.5f) < 0.0001f);
    s_force_high = 0u;

    /* 更换模拟零偏后重复校准，检查没有叠加上一轮扣偏结果。 */
    s_sensor_bias[0] = -31;
    s_sensor_bias[1] = 42;
    s_sensor_bias[2] = -19;
    CHECK(Int_MPU6050_Calibrate(100u) != 0u);
    check_stationary_bias();

    /* 校准通信失败必须保留旧零偏，恢复总线后仍能正确扣偏。 */
    s_ack = NACK;
    CHECK(Int_MPU6050_Calibrate(100u) == 0u);
    CHECK(Int_MPU6050_CalibrationTimedOut() == 0u);
    s_ack = ACK;
    check_stationary_bias();
    puts("PASS fractional gyro bias, slow-rate preservation, recalibration and failed calibration");
    return EXIT_SUCCESS;
}

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
static uint32_t s_time_ms, s_primask;
static int16_t s_accel_input[3] = {0, 0, 16384};
static int16_t s_gyro_extra[3];  /* 模拟真实转动，区别于固定零偏 */
static unsigned int s_read_frames;

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

void I2C_Start(void) { s_sent = 0u; }
void I2C_Stop(void) { }
uint8_t I2C_WaitAck(void) { return s_ack; }
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
        s_read_frames++;
        s_read_index = 0u;
        put_sample(0u, s_accel_input[0]);
        put_sample(2u, s_accel_input[1]);
        put_sample(4u, s_accel_input[2]);
        put_sample(8u, (int16_t)(s_sensor_bias[0] + high + s_gyro_extra[0]));
        put_sample(10u, (int16_t)(s_sensor_bias[1] + high + s_gyro_extra[1]));
        put_sample(12u, (int16_t)(s_sensor_bias[2] + high + s_gyro_extra[2]));
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

/** @brief 验证每步没有延时、重复读取或破坏调用者的中断屏蔽状态。 */
static IMU_CalibrationStatus calibration_step(Accel_struct *mean)
{
    unsigned int reads = s_read_frames;
    uint32_t time = s_time_ms, mask = s_primask;
    IMU_CalibrationStatus result = Int_MPU6050_CalibrationStep(mean);
    CHECK(s_read_frames == reads + 1u);
    CHECK(s_time_ms == time && s_primask == mask);
    return result;
}

/** @brief 覆盖固定窗口、余振、采集中运动、取消、失败和总截止边界。 */
static void test_nonblocking_calibration(void)
{
    Accel_struct mean;
    Gyro_Accel_Struct sample;
    unsigned int i, reads;
    int16_t old_z;
    s_sensor_bias[0] += 10; s_sensor_bias[1] -= 9; s_sensor_bias[2] += 8;
    Int_MPU6050_CalibrationStart();
    for (i = 0u; i < 200u; i++) CHECK(calibration_step(&mean) == IMU_CAL_BUSY);
    s_primask = 1u;
    CHECK(calibration_step(&mean) == IMU_CAL_DONE);
    s_primask = 0u;
    CHECK(mean.accel_x == 0 && mean.accel_y == 0 && mean.accel_z == 16384);
    reads = s_read_frames;
    CHECK(Int_MPU6050_CalibrationStep(&mean) == IMU_CAL_DONE);
    CHECK(s_read_frames == reads);
    check_stationary_bias();

    /* 模长接近 1g 的余振，仍须由相邻帧差阻止提前完成。 */
    Int_MPU6050_CalibrationStart();
    for (i = 0u; i < 20u; i++) {
        s_accel_input[0] = i & 1u ? 600 : -600;
        CHECK(calibration_step(&mean) == IMU_CAL_BUSY);
    }
    s_accel_input[0] = 0;
    for (i = 0u; i < 200u; i++) CHECK(calibration_step(&mean) == IMU_CAL_BUSY);
    CHECK(calibration_step(&mean) == IMU_CAL_DONE);

    /* 均值采集中转动，旧窗口必须丢弃，重采后才允许提交新偏置。 */
    Int_MPU6050_CalibrationStart();
    for (i = 0u; i < 170u; i++) CHECK(calibration_step(&mean) == IMU_CAL_BUSY);
    s_gyro_extra[0] = 100;
    CHECK(calibration_step(&mean) == IMU_CAL_BUSY);
    s_gyro_extra[0] = 0;
    s_sensor_bias[2] += 10;
    for (i = 0u; i < 199u; i++) CHECK(calibration_step(&mean) == IMU_CAL_BUSY);
    CHECK(calibration_step(&mean) == IMU_CAL_DONE);
    check_stationary_bias();

    /* 取消、通信失败或超时，均不能写入部分采集到的零偏。 */
    old_z = s_sensor_bias[2];
    for (i = 0u; i < 3u; i++) {
        unsigned int frame;
        s_sensor_bias[2] = (int16_t)(old_z + 10);
        Int_MPU6050_CalibrationStart();
        for (frame = 0u; frame < 150u; frame++) CHECK(calibration_step(&mean) == IMU_CAL_BUSY);
        if (i == 0u) {
            Int_MPU6050_CalibrationCancel();
            reads = s_read_frames;
            CHECK(Int_MPU6050_CalibrationStep(&mean) == IMU_CAL_CANCELLED);
            CHECK(s_read_frames == reads);
        } else if (i == 1u) {
            s_ack = NACK;
            CHECK(Int_MPU6050_CalibrationStep(&mean) == IMU_CAL_READ_ERROR);
            CHECK(g_imu_ready == 0u);
            s_ack = ACK; g_imu_ready = 1u;
        } else {
            s_accel_input[2] = 20000;
            for (frame = 150u; frame < 499u; frame++) CHECK(calibration_step(&mean) == IMU_CAL_BUSY);
            CHECK(calibration_step(&mean) == IMU_CAL_TIMEOUT);
            s_accel_input[2] = 16384;
        }
        s_force_high = 1u;
        CHECK(Int_MPU6050_Get_Data(&sample));
        CHECK(fabsf(sample.gyro.gyro_z - 10.5f) < 0.0001f);
        s_force_high = 0u;
        s_sensor_bias[2] = old_z;
    }

    /* 第 500 帧恰好采满时成功优先；真实角速度过大必须阻止静止判定。 */
    Int_MPU6050_CalibrationStart();
    s_gyro_extra[1] = 100;
    for (i = 0u; i < 300u; i++) CHECK(calibration_step(&mean) == IMU_CAL_BUSY);
    s_gyro_extra[1] = 0;
    for (i = 0u; i < 199u; i++) CHECK(calibration_step(&mean) == IMU_CAL_BUSY);
    CHECK(calibration_step(&mean) == IMU_CAL_DONE);
    check_stationary_bias();
    puts("PASS nonblocking calibration, settling, motion restart, cancellation, failure and deadline");

    /* 同一采样窗口的三轴平均重力必须传给真实对准器，不能误用最后一帧。 */
    {
        int16_t gravity[3] = {-4853, -8192, 13333}; /* pitch=30°、roll=20°的传感器投影 */
        uint8_t axis;
        Int_MPU6050_CalibrationStart();
        for (i = 0u; i < 201u; i++) {
            for (axis = 0u; axis < 3u; axis++)
                s_accel_input[axis] = (int16_t)(gravity[axis] + (i & 1u ? 10 : -10));
            CHECK(calibration_step(&mean) == (i == 200u ? IMU_CAL_DONE : IMU_CAL_BUSY));
        }
        CHECK(mean.accel_x == gravity[0] && mean.accel_y == gravity[1] && mean.accel_z == gravity[2]);
        CHECK(mean.accel_x != g_imu_data.accel.accel_x);
        CHECK(Attitude_AlignFromAccel(&mean));
        CHECK(fabsf(g_euler.pitch - 30.0f) < 0.01f && fabsf(g_euler.roll - 20.0f) < 0.01f);
        CHECK(g_euler.yaw == 0.0f && g_attitude_diagnostic.direction_ready == 1u);
        puts("PASS averaged gravity from real calibration initializes real attitude");
        s_accel_input[0] = s_accel_input[1] = 0;
        s_accel_input[2] = 16384;
    }
}

/** @brief 真实驱动验证新放宽区间可通过，超出新区间仍会超时且保留旧零偏。 */
static void test_relaxed_calibration_thresholds(void)
{
    Accel_struct mean;
    Gyro_Accel_Struct sample;
    unsigned int trial, frame;
    for (trial = 0u; trial < 2u; trial++) {
        /* 65 计数约 4°/s；配合 300 LSB 帧差及 0.92/1.08g，均在旧阈值之外。 */
        s_sensor_bias[2] += 65;
        s_accel_input[2] = (int16_t)lroundf((trial == 0u ? 1.08f : 0.92f)*IMU_ACCEL_LSB_PER_G);
        Int_MPU6050_CalibrationStart();
        for (frame = 0u; frame < 201u; frame++) {
            s_accel_input[0] = frame & 1u ? 150 : -150;
            CHECK(calibration_step(&mean) == (frame == 200u ? IMU_CAL_DONE : IMU_CAL_BUSY));
        }
        CHECK(Attitude_AlignFromAccel(&mean));
        CHECK(g_attitude_diagnostic.direction_ready == 1u);
        CHECK(fabsf(g_euler.pitch) < 0.01f && fabsf(g_euler.roll) < 0.01f);
    }
    s_accel_input[0] = 0;
    s_accel_input[2] = 16384;
    for (trial = 0u; trial < 3u; trial++) {
        Int_MPU6050_CalibrationStart();
        for (frame = 0u; frame < 500u; frame++) {
            if (trial == 0u) s_accel_input[0] = frame & 1u ? 450 : -450; /* 帧差 900 > 400 */
            if (trial == 1u) s_gyro_extra[2] = 100; /* 约 6.1°/s > 5°/s */
            if (trial == 2u) s_accel_input[2] = (int16_t)lroundf(1.12f*IMU_ACCEL_LSB_PER_G);
            CHECK(calibration_step(&mean) == (frame == 499u ? IMU_CAL_TIMEOUT : IMU_CAL_BUSY));
        }
        s_accel_input[0] = 0;
        s_accel_input[2] = 16384;
        s_gyro_extra[2] = 0;
        s_force_high = 1u;
        CHECK(Int_MPU6050_Get_Data(&sample));
        CHECK(fabsf(sample.gyro.gyro_z - 0.5f) < 0.0001f);
        s_force_high = 0u;
    }
    puts("PASS relaxed thresholds, matching gravity alignment and rejection beyond new limits");
}

int main(void)
{
    Gyro_Accel_Struct sample;
    g_imu_ready = 1u;
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
    s_ack = ACK;
    check_stationary_bias();
    puts("PASS fractional gyro bias, slow-rate preservation, recalibration and failed calibration");
    test_nonblocking_calibration();
    test_relaxed_calibration_thresholds();
    return EXIT_SUCCESS;
}

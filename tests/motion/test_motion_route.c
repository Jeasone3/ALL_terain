/**
 * @file test_motion_route.c
 * @brief 真实运动状态机、路线与 PID 的宿主集成测试。
 * @note 只替代外设采样、姿态输出和电机；Track_Task、IMU_Task 和 PID 使用工程源码。
 */
#include "Mode_FSM.h"
#include "Route.h"
#include "Debug_Log.h"
#include "Track_Task.h"
#include "IMU_Task.h"
#include "Attitude.h"
#include "Int_MPU6050.h"
#include "motor.h"
#include "Int_Beep.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL [%s] line %d: %s\n", s_case_name, __LINE__, #condition); \
        exit(EXIT_FAILURE); \
    } \
} while (0)
#define CLOSE(actual, expected) (fabsf((actual) - (expected)) < 0.001f)

#if ROUTE_DEBUG_STEP_LIMIT == 0u
#define TEST_ACTIVE_STEP_COUNT ROUTE_STEP_COUNT
#else
#define TEST_ACTIVE_STEP_COUNT ROUTE_DEBUG_STEP_LIMIT
#endif

Motor_Struct motorLeft;
Motor_Struct motorRight;
uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];
Gyro_Accel_Struct g_imu_data;
uint8_t g_imu_ready;
Euler_struct g_euler;

static RouteStep s_original_plan[ROUTE_STEP_COUNT];
static void calibrate_test_plan(void);
static int log_has(const char *message);
static const char *s_case_name;
static uint16_t s_sensor_input[GRAYSCALE_SENSOR_CHANNELS];
static float s_sample_yaw;
static uint8_t s_sample_ok;
static uint32_t s_irq_mask;
static unsigned s_frame;
static uint8_t s_beep_on;             /* 蜂鸣器替身当前电平状态 */
static unsigned s_beep_starts;        /* 本场景项目短鸣触发次数 */
static unsigned s_beep_ends;          /* 本场景提示音关闭次数 */
static unsigned s_beep_start_frame;   /* 最近一次打开时的控制帧编号 */
static unsigned s_beep_last_frames;   /* 最近一次提示音的实际持续帧数 */
static unsigned s_sensor_reads;
static unsigned s_imu_reads;
static unsigned s_attitude_ticks;
static unsigned s_resets;
static unsigned s_pid_calls;
static unsigned s_track_pid_calls;
static unsigned s_left_writes;
static unsigned s_right_writes;
static unsigned s_order;
static unsigned s_imu_order;
static unsigned s_attitude_order;
static unsigned s_reset_order;
static unsigned s_imu_frame;
static unsigned s_attitude_frame;
static uint8_t s_inside_control_frame;
static uint8_t s_submit_track_on_irq_restore;
static uint32_t s_milliseconds;
static char s_oled_rows[OLED_PAGE_COUNT][OLED_LINE_CHARS + 1u];

pid_real_t __real_PID_calc(pid_type_def *pid, pid_real_t ref, pid_real_t set);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/**
 * @brief 模拟打开蜂鸣器并记录时刻，用于检查每个指定路口只触发一次。
 * @param 无。
 * @return 无。
 * @note 不模拟发声硬件，仅保存真实 Route 调用的次数和控制帧。
 */
void Int_Beep_On(void)
{
    s_beep_on = 1u;
    ++s_beep_starts;
    s_beep_start_frame = s_frame;
}

/**
 * @brief 模拟关闭蜂鸣器，记录最近一次短鸣的持续时间。
 * @param 无。
 * @return 无。
 */
void Int_Beep_Off(void)
{
    if (s_beep_on) {
        ++s_beep_ends;
        s_beep_last_frames = s_frame - s_beep_start_frame;
    }
    s_beep_on = 0u;
}

/**
 * @brief 提供与真实 10 ms 控制帧同步的测试开机时间。
 * @param 无。
 * @return 当前测试开机毫秒数。
 * @note 由 tick 推进，不依赖宿主运行速度；日志生产函数仍使用真实 HAL_GetTick 调用。
 */
uint32_t HAL_GetTick(void)
{
    return s_milliseconds;
}

/**
 * @brief 将真实日志模块绘制的每一行保存为可断言的文本。
 * @param row OLED 行号，允许 0 至 7。
 * @param format 真实生产模块使用的 printf 格式。
 * @param ... 与格式对应的参数。
 * @return 无。
 * @note 不截断再断言，而是先完整格式化，检查超过 21 字符的布局错误与显示临界区错误。
 */
void OLED_PrintLine(uint8_t row, const char *format, ...)
{
    char rendered[128];
    int length;
    va_list arguments;
    CHECK(row < OLED_PAGE_COUNT && format != NULL);
    CHECK(s_irq_mask == 0u);
    va_start(arguments, format);
    length = vsnprintf(rendered, sizeof(rendered), format, arguments);
    va_end(arguments);
    CHECK(length >= 0 && length <= (int)OLED_LINE_CHARS);
    memset(s_oled_rows[row], 0, sizeof(s_oled_rows[row]));
    memcpy(s_oled_rows[row], rendered, (size_t)length + 1u);
}

/**
 * @brief 模拟 OLED 刷新成功并检查通信期间已恢复中断。
 * @param 无。
 * @return OLED_OK，表示本次刷新成功。
 * @note OLED 外设本身另有真实驱动测试；这里仅替代显示总线。
 */
OLED_Status OLED_Update(void)
{
    CHECK(s_irq_mask == 0u);
    return OLED_OK;
}

/**
 * @brief 返回模拟的 CMSIS 中断屏蔽状态。
 * @param 无。
 * @return 当前 PRIMASK，取 0 或 1。
 * @note 与生产代码调用的名称一致，用于验证短临界区恢复行为。
 */
uint32_t __get_PRIMASK(void)
{
    return s_irq_mask;
}

/**
 * @brief 模拟进入屏蔽中断的短临界区。
 * @param 无。
 * @return 无。
 * @note 不影响宿主运行，只改变供断言观察的 PRIMASK。
 */
void __disable_irq(void)
{
    s_irq_mask = 1u;
}

/**
 * @brief 模拟恢复进入临界区前的 PRIMASK。
 * @param mask 期望恢复的屏蔽状态，取 0 或 1。
 * @return 无。
 * @note 非法值直接让当前测试失败，避免测试替身掩盖生产代码错误。
 *       启用 hook 时，在控制帧首次恢复中断后模拟高优先级 ISR 提交循迹请求。
 *       先清 hook 再调用 Car_Track，防止其自身临界区恢复再次触发而无限递归。
 */
void __set_PRIMASK(uint32_t mask)
{
    CHECK(mask <= 1u);
    s_irq_mask = mask;
    if (s_inside_control_frame && s_submit_track_on_irq_restore && mask == 0u) {
        s_submit_track_on_irq_restore = 0u;
        Car_Track();
    }
}

/**
 * @brief 将测试设置的八路灰度复制为本帧传感器输出。
 * @param sensor_values 生产状态机提供的八路结果数组。
 * @return 无。
 * @note 记录次数和先后顺序，确认 IMU 模式也保留条件检测。
 */
void Read_All_Track(uint16_t *sensor_values)
{
    CHECK(sensor_values != NULL);
    memcpy(sensor_values, s_sensor_input, sizeof(s_sensor_input));
    ++s_sensor_reads;
    ++s_order;
}

/**
 * @brief 发布预设的本帧 IMU 采样成功标志。
 * @param 无。
 * @return 无。
 * @note 记录当前帧号；姿态替身禁止在没有本帧采样的情况下更新反馈。
 */
void Int_MPU6050_Tick(void)
{
    ++s_imu_reads;
    s_imu_frame = s_frame;
    s_imu_order = ++s_order;
    g_imu_ready = s_sample_ok;
}

/**
 * @brief 在成功采样后给出测试指定的姿态反馈。
 * @param 无。
 * @return 无。
 * @note 真实姿态解算在独立测试中验证；这里精确设置转弯边界与 NaN 故障输入。
 */
void Attitude_Tick(void)
{
    CHECK(g_imu_ready != 0u);
    CHECK(s_imu_frame == s_frame);
    CHECK(s_imu_order > s_reset_order);
    ++s_attitude_ticks;
    s_attitude_frame = s_frame;
    s_attitude_order = ++s_order;
    g_euler.yaw = s_sample_yaw;
}

/**
 * @brief 模拟建立动作的零航向基准，并记录动作初始化次数。
 * @param 无。
 * @return 无。
 * @note 复位不产生新采样；后续 PID 必须等待本帧采样与解算。
 */
void Attitude_Reset(void)
{
    ++s_resets;
    s_reset_order = ++s_order;
    g_euler.yaw = g_euler.pitch = g_euler.roll = 0.0f;
}

/**
 * @brief 记录电机输出而不接触真实外设。
 * @param motor 左轮或右轮的生产控制对象。
 * @param speed 带方向的 PWM，允许范围为 [-1000, 1000]。
 * @return 无。
 * @note 每帧验证两个轮子各写一次，禁止调用者提交 Car_* 时立即写电机。
 */
void Motor_SetSpeed(Motor_Struct *motor, int16_t speed)
{
    CHECK(speed >= -1000 && speed <= 1000);
    if (motor == &motorLeft) {
        ++s_left_writes;
    } else {
        CHECK(motor == &motorRight);
        ++s_right_writes;
    }
    motor->speed = speed;
    ++s_order;
}

/**
 * @brief 包装真实 PID 计算，以统计每帧实际运行的运动控制器。
 * @param pid 真实 PID 对象；循迹对象通过地址区分。
 * @param ref 本帧反馈值，沿用生产 PID 的量纲。
 * @param set 本帧设定值，沿用生产 PID 的量纲。
 * @return 真实 PID_calc 算出的输出，不替换控制数学。
 * @note GNU 链接器 --wrap=PID_calc 将调用送入本函数，最后仍执行真实实现。
 */
pid_real_t __wrap_PID_calc(pid_type_def *pid, pid_real_t ref, pid_real_t set)
{
    ++s_pid_calls;
    if (pid == &g_line_controller.pid) {
        ++s_track_pid_calls;
    } else if (s_inside_control_frame) {
        CHECK(s_imu_frame == s_frame);
        CHECK(s_attitude_frame == s_frame);
        CHECK(s_attitude_order > s_imu_order);
        CHECK(s_attitude_order > s_reset_order);
        CHECK(isfinite(g_euler.yaw));
    }
    ++s_order;
    return __real_PID_calc(pid, ref, set);
}

/**
 * @brief 为灰度替身选择本帧在线的通道。
 * @param mask bit0 至 bit7 分别对应 IN1 至 IN8，置位表示在线。
 * @return 无。
 * @note 与工程 LINE_RAW_VALUE 一致，左右路口分别为 0x0F 与 0xF0。
 */
static void set_sensors(uint8_t mask)
{
    unsigned i;
    for (i = 0u; i < GRAYSCALE_SENSOR_CHANNELS; ++i) {
        s_sensor_input[i] = (uint16_t)((mask & (1u << i)) ? LINE_RAW_VALUE : !LINE_RAW_VALUE);
    }
}

/**
 * @brief 初始化一个独立场景，使真实控制器和模拟外设从一致起点运行。
 * @param name 场景名称，失败输出与成功输出都使用此值。
 * @param automatic_route 1 开启短时间测试路线；0 只测试运动动作与条件检测。
 * @return 无。
 * @note 初始化期间不允许写电机；PID 参数来自真实生产初始化函数。
 */
static void begin_case(const char *name, uint8_t automatic_route)
{
    s_case_name = name;
    memcpy(g_route_plan, s_original_plan, sizeof(s_original_plan));
    memset(&motorLeft, 0, sizeof(motorLeft));
    memset(&motorRight, 0, sizeof(motorRight));
    memset(&g_imu_data, 0, sizeof(g_imu_data));
    memset(&g_euler, 0, sizeof(g_euler));
    s_sample_yaw = 0.0f;
    s_sample_ok = g_imu_ready = 1u;
    s_irq_mask = s_frame = 0u;
    s_sensor_reads = s_imu_reads = s_attitude_ticks = s_resets = 0u;
    s_pid_calls = s_track_pid_calls = s_left_writes = s_right_writes = 0u;
    s_order = s_imu_order = s_attitude_order = s_reset_order = 0u;
    s_imu_frame = s_attitude_frame = 0u;
    s_inside_control_frame = 0u;
    s_submit_track_on_irq_restore = 0u;
    s_milliseconds = 0u;
    memset(s_oled_rows, 0, sizeof(s_oled_rows));
    set_sensors(0x18u);
    line_following_init(&g_line_controller);
    IMUTask_Init();
    ModeFSM_Init();
    CHECK(!s_beep_on);
    s_beep_starts = s_beep_ends = s_beep_start_frame = s_beep_last_frames = 0u;
    CHECK(s_left_writes == 0u && s_right_writes == 0u);
    CHECK(g_mode_fsm.mode == MODE_TRACK && g_mode_fsm.status == CAR_STOPPED);
    CHECK(g_route.enabled == 0u);
    if (automatic_route) {
        calibrate_test_plan();
        CHECK(Route_Start());
    } else {
        Car_Track();
    }
}

/**
 * @brief 执行一个真实控制帧，并检查采样与运动控制互斥。
 * @param 无。
 * @return 无。
 * @note 每帧必须采样灰度；最多一次运动 PID，左右轮各只能写一次。
 */
static void tick(void)
{
    unsigned sensor_before = s_sensor_reads;
    unsigned pid_before = s_pid_calls;
    unsigned left_before = s_left_writes;
    unsigned right_before = s_right_writes;
    uint32_t irq_before = s_irq_mask;
    ++s_frame;
    s_milliseconds += 10u;
    s_inside_control_frame = 1u;
    ModeFSM_Tick();
    s_inside_control_frame = 0u;
    CHECK(s_sensor_reads == sensor_before + 1u);
    CHECK(s_pid_calls - pid_before <= 1u);
    CHECK(s_left_writes == left_before + 1u);
    CHECK(s_right_writes == right_before + 1u);
    CHECK(s_irq_mask == irq_before);
    if (g_mode_fsm.status == CAR_RUNNING) {
        CHECK(g_mode_fsm.imu_valid == (uint8_t)(g_mode_fsm.mode == MODE_IMU));
    }
    if (g_mode_fsm.imu_valid != 0u) {
        CHECK(isfinite(g_euler.yaw));
    }
}

/**
 * @brief 连续执行指定数量的真实 10 ms 控制帧。
 * @param frames 帧数；1 帧代表 10 ms 的控制时间。
 * @return 无。
 * @note 输入传感器与 yaw 保持调用前的设置，适合边界计时与防抖场景。
 */
static void ticks(unsigned frames)
{
    while (frames-- != 0u) {
        tick();
    }
}

/**
 * @brief 确认场景中的停车输出与故障信息一致。
 * @param status 预期 CAR_STOPPED 或 CAR_FAULT。
 * @param fault 预期故障原因；主动停车使用 CAR_FAULT_NONE。
 * @return 无。
 * @note 同时验证两个轮子为零，不能只观察枚举状态而漏掉残留 PWM。
 */
static void check_stopped(CarStatus status, CarFault fault)
{
    CHECK(g_mode_fsm.status == status);
    CHECK(g_mode_fsm.fault == fault);
    CHECK(motorLeft.speed == 0 && motorRight.speed == 0);
}

/**
 * @brief 验证默认循迹、延后生效、请求覆盖、停车优先与中断状态恢复。
 * @param 无。
 * @return 无。
 * @note Car_* 请求提交期间不能改变 PWM；停车后必须由下一帧的显式动作恢复。
 */
static void test_default_and_mailbox(void)
{
    ModeFSM_t snapshot;
    unsigned writes_before;
    uint32_t command_before;
    begin_case("default, mailbox and IRQ restore", 0u);
    tick();
    CHECK(s_imu_reads == 0u && s_attitude_ticks == 0u);
    CHECK(s_track_pid_calls == 1u && motorLeft.speed == 400 && motorRight.speed == 400);

    writes_before = s_left_writes;
    Car_Track();
    Car_ImuForward(500u);
    CHECK(s_left_writes == writes_before);
    CHECK(g_mode_fsm.mode == MODE_TRACK);
    tick();
    CHECK(g_mode_fsm.mode == MODE_IMU && g_mode_fsm.imu_state == IMU_FORWARD);
    CHECK(s_resets == 1u && s_imu_reads == 1u && s_attitude_ticks == 1u);

    command_before = g_mode_fsm.command_id;
    s_irq_mask = 1u;
    Car_TurnLeft90(MODE_TRACK);
    Car_Stop();
    Car_TurnRight90(MODE_IMU);
    CHECK(s_irq_mask == 1u);
    tick();
    check_stopped(CAR_STOPPED, CAR_FAULT_NONE);
    CHECK(g_mode_fsm.command_id == command_before + 1u);
    ModeFSM_GetSnapshot(&snapshot);
    ModeFSM_GetSnapshot(NULL);
    CHECK(snapshot.status == CAR_STOPPED && s_irq_mask == 1u);
    tick();
    check_stopped(CAR_STOPPED, CAR_FAULT_NONE);

    Car_ImuForward(500u);
    Car_Track();
    tick();
    CHECK(g_mode_fsm.mode == MODE_TRACK && g_mode_fsm.status == CAR_RUNNING);
    CHECK(s_irq_mask == 1u);
    s_irq_mask = 0u;
    command_before = g_mode_fsm.command_id;
    Car_Stop();
    /* 第一轮取出 STOP 后恢复中断，此时高优先级 ISR 提交 TRACK。
     * 已接受的停车必须在本帧落到电机，ISR 的新动作只能等下一帧执行。 */
    s_submit_track_on_irq_restore = 1u;
    tick();
    check_stopped(CAR_STOPPED, CAR_FAULT_NONE);
    CHECK(s_submit_track_on_irq_restore == 0u);
    CHECK(g_mode_fsm.command_id == command_before + 1u);
    tick();
    CHECK(g_mode_fsm.status == CAR_RUNNING && g_mode_fsm.mode == MODE_TRACK);
    CHECK(g_mode_fsm.command_id == command_before + 2u);
    CHECK(motorLeft.speed == 400 && motorRight.speed == 400);
    puts("PASS default, mailbox and IRQ restore");
}

/**
 * @brief 验证左右转分别衔接循迹或 IMU 直行，覆盖四种转弯去向。
 * @param 无。
 * @return 无。
 * @note 完成后的直行保留原来的 +/-90 度目标，不允许重新把姿态归零。
 */
static void test_turn_handoffs(void)
{
    unsigned direction;
    unsigned destination;
    for (direction = 0u; direction < 2u; ++direction) {
        for (destination = 0u; destination < 2u; ++destination) {
            float target = direction == 0u ? 90.0f : -90.0f;
            CarMode next = destination == 0u ? MODE_TRACK : MODE_IMU;
            unsigned imu_before;
            begin_case("left/right turn and both destinations", 0u);
            g_euler.yaw = 146.0f;
            if (direction == 0u) {
                Car_TurnLeft90(next);
            } else {
                Car_TurnRight90(next);
            }
            tick();
            CHECK(s_resets == 1u && CLOSE(g_mode_fsm.target_yaw, target));
            CHECK(g_mode_fsm.imu_state == (direction == 0u ? IMU_TURN_LEFT : IMU_TURN_RIGHT));
            CHECK(motorLeft.speed * (direction == 0u ? -1 : 1) > 0);
            CHECK(motorRight.speed == -motorLeft.speed);
            CHECK(abs(motorLeft.speed) == 720);
            CHECK(s_track_pid_calls == 0u);
            s_sample_yaw = target;
            tick();
            CHECK(g_mode_fsm.turn_count == 0u);
            CHECK(motorLeft.speed == 0 && motorRight.speed == 0);
            tick();
            CHECK(g_mode_fsm.turn_count == 1u && g_mode_fsm.mode == next);
            CHECK(g_mode_fsm.status == CAR_RUNNING && s_resets == 1u);
            imu_before = s_imu_reads;
            tick();
            if (next == MODE_IMU) {
                CHECK(g_mode_fsm.imu_state == IMU_FORWARD);
                CHECK(CLOSE(g_mode_fsm.target_yaw, target));
                CHECK(s_imu_reads == imu_before + 1u && s_track_pid_calls == 0u);
                CHECK(motorLeft.speed == 500 && motorRight.speed == 500);
                /* 角度反馈与目标相差完整一圈时也应保持直行。 */
                s_sample_yaw = target + 360.0f;
                tick();
                CHECK(motorLeft.speed == 500 && motorRight.speed == 500);
            } else {
                CHECK(s_imu_reads == imu_before);
                CHECK(s_track_pid_calls >= 1u);
            }
        }
    }
    puts("PASS four turn destinations and retained heading");
}

/**
 * @brief 用真实转弯 PID 验证末段最低输出、过冲纠偏、容差内停转与两帧交接。
 * @param 无。
 * @return 无。
 * @note 左右转分别覆盖还差 6.8°、恰好 5°、进入 4°容差和过冲 6°的反馈。
 *       250 PWM 是当前源码的试车值；断言只验证输出与状态，不证明轮子一定能转动。
 */
static void test_turn_minimum_drive(void)
{
    unsigned direction;
    for (direction = 0u; direction < 2u; ++direction) {
        float sign = direction == 0u ? 1.0f : -1.0f;
        begin_case("minimum turn drive and stopped confirmation in both directions", 0u);
        if (direction == 0u) Car_TurnLeft90(MODE_TRACK);
        else Car_TurnRight90(MODE_TRACK);
        tick();
        CHECK(abs(motorLeft.speed) == 720);

        s_sample_yaw = sign * 83.2f;
        ticks(100u); /* 固定反馈让积分充分累积，小输出仍须补足当前最低 250 PWM。 */
        CHECK(g_mode_fsm.turn_count == 0u && g_mode_fsm.status == CAR_RUNNING);
        CHECK(motorLeft.speed == (int16_t)(-sign * 250.0f));
        CHECK(motorRight.speed == -motorLeft.speed);

        s_sample_yaw = -sign * 86.0f;
        tick();
        CHECK(abs(motorLeft.speed) == 800); /* 大误差仍受原最大转弯 PWM 限制。 */
        s_sample_yaw = sign * 85.0f;
        tick();
        CHECK(g_mode_fsm.turn_count == 0u && abs(motorLeft.speed) == 250);
        s_sample_yaw = sign * 86.0f;
        tick();
        CHECK(g_mode_fsm.turn_count == 0u && motorLeft.speed == 0 && motorRight.speed == 0);

        s_sample_yaw = sign * 96.0f; /* 过冲后剩余角差反向，重新确认前先反向纠偏。 */
        tick();
        CHECK(g_mode_fsm.turn_count == 0u);
        CHECK(motorLeft.speed == (int16_t)(sign * 250.0f));
        CHECK(motorRight.speed == -motorLeft.speed);
        s_sample_yaw = sign * 86.0f;
        tick();
        CHECK(g_mode_fsm.turn_count == 0u && motorLeft.speed == 0 && motorRight.speed == 0);
        tick();
        CHECK(g_mode_fsm.turn_count == 1u && g_mode_fsm.mode == MODE_TRACK);
        CHECK(motorLeft.speed == 400 && motorRight.speed == 400);
        Car_Stop();
        tick();
        check_stopped(CAR_STOPPED, CAR_FAULT_NONE);
    }
    puts("PASS minimum turn drive, overshoot signs, zero-PWM confirmation and track handoff");
}

/**
 * @brief 验证连续动作重新建立基准、完成判据防抖以及 500 帧超时边界。
 * @param 无。
 * @return 无。
 * @note 当前源码容差严格小于 5 度，命中必须连续两帧；断续命中不能累计。
 */
static void test_turn_filter_and_timeout(void)
{
    begin_case("turn debounce and consecutive reset", 0u);
    Car_TurnLeft90(MODE_IMU);
    tick();
    s_sample_yaw = 90.0f;
    tick();
    s_sample_yaw = 85.0f;
    tick();
    CHECK(g_mode_fsm.turn_count == 0u);
    s_sample_yaw = 86.0f;
    tick();
    CHECK(g_mode_fsm.turn_count == 0u);
    tick();
    CHECK(g_mode_fsm.turn_count == 1u);
    s_sample_yaw = 0.0f;
    Car_TurnRight90(MODE_TRACK);
    tick();
    CHECK(s_resets == 2u && CLOSE(g_mode_fsm.target_yaw, -90.0f));
    CHECK(motorLeft.speed == 720 && motorRight.speed == -720);
    CHECK(g_mode_fsm.turn_count == 1u);
    s_sample_yaw = -450.0f;
    ticks(3u);
    CHECK(g_mode_fsm.turn_count == 2u && g_mode_fsm.mode == MODE_TRACK);

    begin_case("exact 500-frame timeout", 0u);
    Car_TurnLeft90(MODE_IMU);
    ticks(499u);
    CHECK(g_mode_fsm.status == CAR_RUNNING && g_mode_fsm.state_frames == 499u);
    tick();
    check_stopped(CAR_FAULT, CAR_FAULT_TURN_TIMEOUT);
    CHECK(g_mode_fsm.turn_count == 0u);
    tick();
    check_stopped(CAR_FAULT, CAR_FAULT_TURN_TIMEOUT);
    Car_Track();
    tick();
    CHECK(g_mode_fsm.status == CAR_RUNNING && g_mode_fsm.fault == CAR_FAULT_NONE);

    begin_case("confirmed completion on deadline frame", 0u);
    Car_TurnRight90(MODE_IMU);
    ticks(498u);
    s_sample_yaw = -90.0f;
    ticks(2u);
    CHECK(g_mode_fsm.status == CAR_RUNNING && g_mode_fsm.turn_count == 1u);
    CHECK(g_mode_fsm.imu_state == IMU_FORWARD && g_mode_fsm.state_frames == 1u);
    puts("PASS continuous completion filter, new baseline and timeout");
}

/**
 * @brief 验证转弯在 5 秒截止帧已经进入容差时，只补足剩余确认帧。
 * @param 无。
 * @return 无。
 * @note 最迟第 501 帧完成或故障；补帧期间离开 5° 容差即停车，未进容差不延长。
 *       复现目标 -90°、最后读数 -86°且仍停在 R90 的截止边界，不放宽角度或修改 PID。
 */
static void test_turn_deadline_confirmation(void)
{
    begin_case("first in-tolerance sample on deadline gets one confirmation frame", 0u);
    Car_TurnRight90(MODE_TRACK);
    ticks(499u);
    s_sample_yaw = -86.0f;
    tick();
    CHECK(g_mode_fsm.status == CAR_RUNNING && g_mode_fsm.state_frames == 500u);
    CHECK(g_mode_fsm.imu_state == IMU_TURN_RIGHT && g_mode_fsm.turn_count == 0u);
    tick();
    CHECK(g_mode_fsm.turn_count == 1u && g_mode_fsm.mode == MODE_TRACK);
    CHECK(g_mode_fsm.status == CAR_RUNNING && motorLeft.speed == 400 && motorRight.speed == 400);

    begin_case("two hits by deadline complete without an additional frame", 0u);
    Car_TurnRight90(MODE_TRACK);
    ticks(498u);
    s_sample_yaw = -86.0f;
    ticks(2u);
    CHECK(g_mode_fsm.state_frames == 1u && g_mode_fsm.mode == MODE_TRACK);
    CHECK(g_mode_fsm.status == CAR_RUNNING && g_mode_fsm.turn_count == 1u);

    begin_case("leaving tolerance in first extra frame faults immediately", 0u);
    Car_TurnRight90(MODE_TRACK);
    ticks(499u);
    s_sample_yaw = -86.0f;
    tick();
    s_sample_yaw = -84.9f;
    tick();
    check_stopped(CAR_FAULT, CAR_FAULT_TURN_TIMEOUT);
    CHECK(g_mode_fsm.state_frames == 501u && g_mode_fsm.turn_count == 0u);
    CHECK(DebugLog_Display() == OLED_OK && log_has("TURN_TO"));
    s_sample_yaw = -90.0f;
    ticks(10u);
    check_stopped(CAR_FAULT, CAR_FAULT_TURN_TIMEOUT);

    begin_case("interrupted confirmation on deadline cannot obtain grace", 0u);
    Car_TurnRight90(MODE_TRACK);
    ticks(498u);
    s_sample_yaw = -86.0f;
    tick();
    s_sample_yaw = -85.0f; /* 恰好 5° 不满足严格小于 5°，不能再开始新一轮确认。 */
    tick();
    check_stopped(CAR_FAULT, CAR_FAULT_TURN_TIMEOUT);
    CHECK(g_mode_fsm.state_frames == 500u && g_mode_fsm.turn_count == 0u);

    begin_case("opposite yaw sign cannot receive completion grace", 0u);
    Car_TurnRight90(MODE_TRACK);
    s_sample_yaw = 86.0f;
    ticks(500u);
    check_stopped(CAR_FAULT, CAR_FAULT_TURN_TIMEOUT);
    CHECK(g_mode_fsm.state_frames == 500u && g_mode_fsm.turn_count == 0u);
    puts("PASS bounded deadline confirmation at -86 degrees and immediate fault on lost tolerance");
}

/**
 * @brief 验证 IMU 本帧读取失败、NaN、Inf、非法指令及显式恢复。
 * @param 无。
 * @return 无。
 * @note 旧 yaw 恰好为目标也不能绕过有效采样；故障帧不允许运动 PID 输出。
 */
static void test_faults_and_recovery(void)
{
    unsigned pid_before;
    unsigned sample_before;
    begin_case("fresh IMU failure cannot reuse old yaw", 1u);
    s_sample_yaw = 15.0f;
    Car_ImuForward(500u);
    tick();
    CHECK(motorLeft.speed > motorRight.speed && motorLeft.speed <= 1000 && motorRight.speed >= 0);
    g_euler.yaw = 0.0f;
    s_sample_ok = 0u;
    pid_before = s_pid_calls;
    sample_before = s_imu_reads;
    tick();
    CHECK(s_imu_reads == sample_before + 1u);
    CHECK(s_pid_calls == pid_before);
    check_stopped(CAR_FAULT, CAR_FAULT_IMU);
    set_sensors(0x0Fu);
    ticks(4u);
    check_stopped(CAR_FAULT, CAR_FAULT_IMU);
    CHECK(g_route.state == ROUTE_ABORTED);
    s_sample_ok = g_imu_ready = 1u;
    Car_ImuForward(500u);
    tick();
    CHECK(g_mode_fsm.status == CAR_RUNNING && g_mode_fsm.fault == CAR_FAULT_NONE);

    s_sample_yaw = NAN;
    pid_before = s_pid_calls;
    tick();
    check_stopped(CAR_FAULT, CAR_FAULT_IMU);
    CHECK(s_pid_calls == pid_before);
    s_sample_yaw = INFINITY;
    Car_TurnLeft90(MODE_IMU);
    tick();
    check_stopped(CAR_FAULT, CAR_FAULT_IMU);
    s_sample_yaw = 0.0f;
    Car_TurnRight90((CarMode)99);
    tick();
    check_stopped(CAR_FAULT, CAR_FAULT_COMMAND);
    Car_Track();
    tick();
    CHECK(g_mode_fsm.status == CAR_RUNNING && g_mode_fsm.mode == MODE_TRACK);
    Car_Stop();
    tick();
    ticks(4u);
    check_stopped(CAR_STOPPED, CAR_FAULT_NONE);

    begin_case("failed project cannot restart itself", 1u);
    tick();
    set_sensors(0xF0u);
    ticks(3u);
    set_sensors(0x18u);
    ticks(3u);
    CHECK(g_route.state == ROUTE_OBSTACLE);
    s_sample_ok = 0u;
    tick();
    check_stopped(CAR_FAULT, CAR_FAULT_IMU);
    s_sample_ok = g_imu_ready = 1u;
    ticks(85u);
    check_stopped(CAR_FAULT, CAR_FAULT_IMU);
    CHECK(g_route.state == ROUTE_ABORTED && g_mode_fsm.turn_count == 0u);
    puts("PASS fresh data failures, invalid commands and explicit recovery");
}

/**
 * @brief 用真实直行 PID 验证当前 1° 死区的两侧边界和纠偏方向。
 * @param 无。
 * @return 无。
 * @note 每个输入前重新设置目标并清除 PID 历史，避免微分残留影响死区断言。
 *       误差恰好为 +/-1° 时必须开始纠偏；死区内原始角差仍可供状态观察。
 */
static void test_real_pid_forward_deadband(void)
{
    static const float errors[] = {-1.01f, -1.0f, -0.99f, 0.0f, 0.99f, 1.0f, 1.01f};
    unsigned i;
    begin_case("real PID 1-degree forward deadband", 0u);
    for (i = 0u; i < sizeof(errors) / sizeof(errors[0]); ++i) {
        float error = errors[i];
        IMUTask_SetTarget(0.0f);
        g_euler.yaw = -error;
        CHECK(CLOSE(IMUTask_GetError(), error));
        IMUTask_ForwardTick(500u);
        if (fabsf(error) < 1.0f) {
            CHECK(motorLeft.speed == 500 && motorRight.speed == 500);
        } else if (error > 0.0f) {
            CHECK(motorLeft.speed < 500 && motorRight.speed > 500);
        } else {
            CHECK(motorLeft.speed > 500 && motorRight.speed < 500);
        }
    }
    puts("PASS real PID 1-degree forward deadband boundaries and motor signs");
}

/**
 * @brief 用真实 IMU PID 验证跨 +/-180 度时走最短角差及左右输出符号。
 * @param 无。
 * @return 无。
 * @note 直接调用控制器测试角度数学，不通过姿态 mock 的复位逻辑。
 */
static void test_real_pid_angle_wrap(void)
{
    begin_case("real PID shortest angle and motor direction", 0u);
    IMUTask_SetTarget(170.0f);
    g_euler.yaw = -170.0f;
    CHECK(CLOSE(IMUTask_GetError(), -20.0f));
    IMUTask_TurnTick();
    CHECK(motorLeft.speed > 0 && motorRight.speed < 0);
    CHECK(motorRight.speed == -motorLeft.speed);
    IMUTask_SetTarget(-170.0f);
    g_euler.yaw = 170.0f;
    CHECK(CLOSE(IMUTask_GetError(), 20.0f));
    IMUTask_TurnTick();
    CHECK(motorLeft.speed < 0 && motorRight.speed > 0);
    IMUTask_SetTarget(170.0f);
    g_euler.yaw = -170.0f;
    IMUTask_ForwardTick(500u);
    CHECK(motorLeft.speed > motorRight.speed);
    CHECK(motorLeft.speed <= 1000 && motorRight.speed >= 0);
    IMUTask_SetTarget(-170.0f);
    g_euler.yaw = 170.0f;
    IMUTask_ForwardTick(500u);
    CHECK(motorLeft.speed < motorRight.speed);
    CHECK(motorLeft.speed >= 0 && motorRight.speed <= 1000);
    IMUTask_SetTarget(10000.0f);
    g_euler.yaw = 0.0f;
    CHECK(CLOSE(IMUTask_GetError(), -80.0f));
    IMUTask_TurnTick();
    CHECK(abs(motorLeft.speed) <= 800 && abs(motorRight.speed) <= 800);
    puts("PASS real PID shortest-angle feedback and motor signs");
}

/**
 * @brief 验证所有路口两帧确认、类型抖动、单次锁存与有线/无线条件事件。
 * @param 无。
 * @return 无。
 * @note 关闭示例决策只保留检测，防止路线动作影响防抖边界的观察。
 */
static void test_route_conditions(void)
{
    begin_case("cross two-frame confirmation and debounce", 0u);
    set_sensors(0xFFu);
    tick();
    CHECK(g_route.junction == JUNCTION_NONE && !g_route.junction_event);
    /* 缺一路在线会改变类型，之前的十字候选帧不能累计。 */
    set_sensors(0xFEu);
    tick();
    set_sensors(0xFFu);
    tick();
    CHECK(g_route.junction == JUNCTION_NONE && !g_route.junction_event);
    tick();
    CHECK(g_route.junction == JUNCTION_CROSS && g_route.junction_event);
    tick();
    CHECK(!g_route.junction_event);
    set_sensors(0x18u);
    tick();
    CHECK(g_route.junction == JUNCTION_CROSS);
    set_sensors(0xFFu);
    ticks(3u);
    CHECK(g_route.junction == JUNCTION_CROSS && !g_route.junction_event);
    set_sensors(0x18u);
    ticks(2u);
    CHECK(g_route.junction == JUNCTION_NONE);
    set_sensors(0xFFu);
    tick();
    CHECK(g_route.junction == JUNCTION_NONE && !g_route.junction_event);
    tick();
    CHECK(g_route.junction == JUNCTION_CROSS && g_route.junction_event);

    begin_case("condition debounce and junction latch", 0u);
    set_sensors(0x0Fu);
    tick();
    CHECK(g_route.junction == JUNCTION_NONE && !g_route.junction_event);
    set_sensors(0xF0u);
    tick();
    CHECK(g_route.junction == JUNCTION_NONE && !g_route.junction_event);
    set_sensors(0x0Fu);
    tick();
    CHECK(!g_route.junction_event);
    tick();
    CHECK(g_route.junction == JUNCTION_LEFT && g_route.junction_event);
    tick();
    CHECK(!g_route.junction_event);
    set_sensors(0xFFu);
    tick();
    CHECK(g_route.junction == JUNCTION_LEFT && !g_route.junction_event);
    tick();
    CHECK(g_route.junction == JUNCTION_CROSS && !g_route.junction_event);
    set_sensors(0x18u);
    tick();
    set_sensors(0xF0u);
    ticks(2u);
    CHECK(g_route.junction == JUNCTION_RIGHT && !g_route.junction_event);
    set_sensors(0x18u);
    ticks(2u);
    CHECK(g_route.junction == JUNCTION_NONE);
    set_sensors(0xF0u);
    ticks(2u);
    CHECK(g_route.junction == JUNCTION_RIGHT && g_route.junction_event);

    begin_case("line-found event in IMU and stopped modes", 0u);
    set_sensors(0u);
    Car_ImuForward(500u);
    ticks(3u);
    CHECK(!g_route.line_found);
    set_sensors(0x01u);
    tick();
    CHECK(!g_route.line_found && !g_route.line_found_event);
    tick();
    CHECK(g_route.line_found && g_route.line_found_event);
    CHECK(s_track_pid_calls == 0u);
    tick();
    CHECK(!g_route.line_found_event);
    Car_Stop();
    set_sensors(0u);
    tick();
    CHECK(g_route.line_found && !g_route.line_lost_event);
    tick();
    CHECK(!g_route.line_found && g_route.line_lost_event);
    check_stopped(CAR_STOPPED, CAR_FAULT_NONE);
    Route_Tick(NULL, 0u);
    CHECK(!g_route.junction_event && !g_route.line_found_event && !g_route.line_lost_event);
    puts("PASS junction confirmation, latch and line events across modes");
}

/**
 * @brief 给当前路线注入短时间，缩短取消、恢复与故障场景的宿主运行帧数。
 * @param 无。
 * @return 无。
 * @note 只改测试进程内的 RAM；生产 2000/4000/5000 ms 另有完整边界场景验证。
 *       末段只右转后持续循迹，入口与通行参数保持零，不能注入虚构项目时长。
 */
static void calibrate_test_plan(void)
{
    unsigned i;
    for (i = 0u; i < ROUTE_STEP_COUNT; ++i) {
        if (g_route_plan[i].action != ROUTE_ACTION_RIGHT_TRACK) {
            g_route_plan[i].entry_ms = 30u;
            g_route_plan[i].pass_ms = 50u;
        }
        g_route_plan[i].timeout_ms = 1000u; /* 大于真实的 500 ms 前移，其他短场景仍有独立截止边界。 */
    }
}

/**
 * @brief 提供符合当前段的路口和转弯反馈，执行一帧真实控制。
 * @param 无。
 * @return 无。
 * @note IMU 通行阶段持续给宽线；只有 RECOVER 提供中心窄线，不能靠项目宽线推进。
 */
static void drive_route_frame(void)
{
    const RouteStep *step = &g_route_plan[g_route.step_index];
    s_sample_yaw = 0.0f;
    set_sensors(0x18u);
    if (g_route.state == ROUTE_FOLLOW) {
        set_sensors(step->trigger_junction == JUNCTION_RIGHT ? 0xF0u :
                    step->trigger_junction == JUNCTION_LEFT ? 0x0Fu : 0xFFu);
    } else if (g_route.state == ROUTE_TURN) {
        s_sample_yaw = step->action == ROUTE_ACTION_LEFT ? 90.0f : -90.0f;
    } else if (g_route.state == ROUTE_FINAL_TURN || g_route.state == ROUTE_FINAL_FORWARD) {
        s_sample_yaw = 90.0f;
    } else if (g_route.state == ROUTE_OBSTACLE) {
        set_sensors(0xFFu);
    }
    tick();
}

/**
 * @brief 逐帧运行到指定段和阶段，超出预算则失败。
 * @param index 目标路线段下标。
 * @param state 目标阶段。
 * @return 无。
 */
static void reach_route_state(unsigned index, RouteState state)
{
    unsigned budget = 3000u;
    while ((g_route.step_index != index || g_route.state != state) && budget-- != 0u) {
        drive_route_frame();
    }
    CHECK(budget != 0u);
}

/**
 * @brief 验证只检查本次启用范围，并在调试截止段恢复持续循迹。
 * @param 无。
 * @return 无。
 * @note 同一源码按截止 1 至 9 和全部段编译；未启用的第二段故意保留非法参数。
 */
static void test_route_debug_limit(void)
{
    uint32_t command;
    begin_case("only enabled route prefix is validated", 0u);
    Car_Stop();
    tick();
    calibrate_test_plan();
    g_route_plan[1].pass_ms = 0u;
    g_route_plan[1].base_pwm = 0u;
    if (ROUTE_DEBUG_STEP_LIMIT == 1u) {
        CHECK(Route_IsCalibrated() && Route_Start());
        reach_route_state(0u, ROUTE_CRUISE);
        CHECK(g_mode_fsm.mode == MODE_TRACK && g_mode_fsm.turn_count == 0u);
        CHECK(s_beep_starts == 1u);
        command = g_mode_fsm.command_id;
        set_sensors(0xFFu);
        ticks(1000u);
        CHECK(g_route.state == ROUTE_CRUISE && g_route.step_index == 0u);
        CHECK(g_mode_fsm.status == CAR_RUNNING && g_mode_fsm.command_id == command);
        CHECK(!s_beep_on && s_beep_starts == 1u && s_beep_last_frames == 10u);
    } else {
        CHECK(!Route_IsCalibrated() && !Route_Start());
        CHECK(g_route.state == ROUTE_UNCALIBRATED && g_route.step_index == 1u);
        CHECK(DebugLog_Display() == OLED_OK && log_has("WAIT_CAL"));
        ticks(20u);
        check_stopped(CAR_STOPPED, CAR_FAULT_NONE);
        CHECK(s_beep_starts == 0u);
    }
    if (TEST_ACTIVE_STEP_COUNT == ROUTE_STEP_COUNT) {
        begin_case("track-only turns require zero project parameters", 0u);
        Car_Stop();
        tick();
        CHECK(Route_IsCalibrated()); /* 第 4、5 段零参数有效，普通项目仍要求非零入口与通行。 */
        g_route_plan[3].pass_ms = 10u;
        CHECK(!Route_Start() && g_route.step_index == 3u);
        g_route_plan[3].pass_ms = 0u;
        g_route_plan[4].base_pwm = 500u;
        CHECK(!Route_Start() && g_route.step_index == 4u);
        g_route_plan[4].base_pwm = 0u;
        g_route_plan[2].pass_ms = 0u;
        CHECK(!Route_Start() && g_route.step_index == 2u);
    }
    puts("PASS enabled prefix validation and debug cutoff");
}

/**
 * @brief 按当前生产时间验证前两段顺序、灰度对正、单次请求和可选调试截止。
 * @param 无。
 * @return 无。
 * @note 精确观察命令序号、控制器调用次数与完成帧；已调试段的 PWM 和第二段通行时间
 *       读取启动时保存的实际表项，允许用户继续标定；新增段另按本次指定参数逐项断言。
 */
static void test_route_steps(void)
{
    static const uint8_t invalid_masks[] = {0u, 0x01u, 0x28u, 0x3Cu};
    uint32_t command;
    unsigned i, track_before, imu_before, turn_done_frame, project_frame;
    unsigned second_pass_frames = s_original_plan[1].pass_ms / 10u;
    begin_case("production two-segment sequence and exact durations", 1u);
    memcpy(g_route_plan, s_original_plan, sizeof(s_original_plan));
    CHECK(Route_IsCalibrated());
    CHECK(g_route_plan[0].base_pwm == 600u);
    CHECK(g_route_plan[0].entry_ms == 2000u && g_route_plan[1].entry_ms == 2000u);
    CHECK(g_route_plan[0].pass_ms == 4000u);
    tick();
    CHECK(g_route.state == ROUTE_FOLLOW && g_mode_fsm.mode == MODE_TRACK);
    CHECK(s_beep_starts == 0u && g_mode_fsm.command_id == 1u);

    set_sensors(0xFFu);
    ticks(3u);
    CHECK(g_route.state == ROUTE_FOLLOW && s_beep_starts == 0u);
    set_sensors(0xF0u);
    ticks(3u); /* 同一宽区域从十字变右口，不能产生第二个新事件。 */
    CHECK(g_route.state == ROUTE_FOLLOW);
    set_sensors(0x18u);
    ticks(3u);
    set_sensors(0x0Fu);
    ticks(3u);
    CHECK(g_route.state == ROUTE_FOLLOW && s_beep_starts == 0u);
    set_sensors(0x18u);
    ticks(3u);
    set_sensors(0xF0u);
    tick();
    CHECK(g_route.state == ROUTE_FOLLOW && s_beep_starts == 0u);
    tick();
    CHECK(g_route.state == ROUTE_ENTRY && g_route.state_frames == 0u);
    CHECK(s_beep_on && s_beep_starts == 1u);
    CHECK(g_mode_fsm.mode == MODE_TRACK && g_mode_fsm.command_id == 1u);
    CHECK(g_mode_fsm.turn_count == 0u && s_resets == 0u);

    track_before = s_track_pid_calls;
    imu_before = s_imu_reads;
    set_sensors(0x20u); /* 入口偏右仍由真实灰度 PID 调整，不是原地等待。 */
    tick();
    CHECK(motorLeft.speed > motorRight.speed);
    CHECK(s_track_pid_calls == track_before + 1u && s_imu_reads == imu_before);
    set_sensors(0x18u);
    ticks(198u);
    CHECK(g_route.state == ROUTE_ENTRY && g_route.state_frames == 199u);
    CHECK(g_mode_fsm.command_id == 1u);
    tick();
    CHECK(g_route.state == ROUTE_OBSTACLE && g_route.state_frames == 0u);
    CHECK(g_mode_fsm.mode == MODE_IMU && g_mode_fsm.base_pwm == 600u);
    CHECK(g_mode_fsm.command_id == 2u && s_resets == 1u);
    CHECK(!s_beep_on && s_beep_last_frames == 10u);

    set_sensors(0xFFu);
    track_before = s_track_pid_calls;
    ticks(399u);
    CHECK(g_route.state == ROUTE_OBSTACLE && g_route.state_frames == 399u);
    CHECK(g_mode_fsm.command_id == 2u && s_track_pid_calls == track_before);
    CHECK(!g_route.junction_event && s_resets == 1u);
    tick();
    CHECK(g_route.state == ROUTE_RECOVER && g_mode_fsm.mode == MODE_TRACK);
    CHECK(g_mode_fsm.command_id == 3u);
    command = g_mode_fsm.command_id;
    ticks(20u);
    CHECK(g_route.state == ROUTE_RECOVER && g_route.step_index == 0u);
    for (i = 0u; i < sizeof(invalid_masks) / sizeof(invalid_masks[0]); ++i) {
        set_sensors(invalid_masks[i]);
        ticks(3u);
        CHECK(g_route.state == ROUTE_RECOVER && !g_route.junction_event);
    }
    set_sensors(0x18u);
    tick();
    CHECK(g_route.state == ROUTE_RECOVER);
    set_sensors(0u);
    tick();
    set_sensors(0x18u);
    ticks(2u);
    CHECK(g_route.state == ROUTE_FOLLOW && g_route.step_index == 1u);
    CHECK(g_mode_fsm.command_id == command);

    set_sensors(0xF0u);
    ticks(2u);
    CHECK(g_route.state == ROUTE_FOLLOW);
    set_sensors(0xFFu);
    tick();
    CHECK(g_route.state == ROUTE_FOLLOW);
    tick();
    CHECK(g_route.state == ROUTE_APPROACH && g_mode_fsm.mode == MODE_IMU);
    CHECK(!g_route.junction_event); /* 先右口后十字时，新事件锁存不能挡住已确认的目标十字。 */
    CHECK(g_mode_fsm.base_pwm == 500u && g_mode_fsm.command_id == 4u);
    set_sensors(0x18u);
    ticks(49u);
    CHECK(g_route.state == ROUTE_APPROACH && g_mode_fsm.imu_state == IMU_FORWARD);
    tick();
    CHECK(g_route.state == ROUTE_TURN && g_mode_fsm.imu_state == IMU_TURN_RIGHT);
    CHECK(CLOSE(g_mode_fsm.target_yaw, -90.0f) && g_mode_fsm.command_id == 5u);
    ticks(200u); /* 转动尚未完成的 2 秒不能消耗第二段入口的 2 秒。 */
    CHECK(g_route.state == ROUTE_TURN && g_mode_fsm.turn_count == 0u);
    s_sample_yaw = -90.0f;
    tick();
    CHECK(g_mode_fsm.turn_count == 0u);
    tick();
    CHECK(g_mode_fsm.turn_count == 1u && g_mode_fsm.mode == MODE_TRACK);
    CHECK(g_route.state == ROUTE_TURN);
    turn_done_frame = s_frame;
    s_sample_yaw = 0.0f;
    tick();
    CHECK(g_route.state == ROUTE_ENTRY && g_route.state_frames == 1u);
    CHECK(g_mode_fsm.command_id == 5u); /* 转弯完成自动接循迹，不重复提交 TRACK。 */
    ticks(198u);
    CHECK(g_route.state == ROUTE_ENTRY && g_route.state_frames == 199u);
    tick();
    CHECK(s_frame - turn_done_frame == 200u);
    CHECK(g_route.state == ROUTE_OBSTACLE && g_mode_fsm.base_pwm == s_original_plan[1].base_pwm);
    CHECK(g_mode_fsm.command_id == 6u && s_resets == 4u);
    project_frame = s_frame;
    set_sensors(0xFFu);
    ticks(second_pass_frames - 1u);
    CHECK(g_route.state == ROUTE_OBSTACLE && g_mode_fsm.command_id == 6u);
    tick();
    CHECK(s_frame - project_frame == second_pass_frames);
    if (TEST_ACTIVE_STEP_COUNT > 2u) {
        CHECK(g_route.state == ROUTE_RECOVER && g_route.step_index == 1u);
        CHECK(g_mode_fsm.mode == MODE_TRACK && g_mode_fsm.command_id == 7u);
        CHECK(g_mode_fsm.turn_count == 1u && s_beep_starts == 2u);
        puts("PASS current first two segments: configured PWM, exact entry/passage boundaries and turn handoff");
        return;
    }
    CHECK(g_route.state == ROUTE_CRUISE && g_route.step_index == 1u);
    CHECK(g_mode_fsm.mode == MODE_TRACK && g_mode_fsm.status == CAR_RUNNING);
    CHECK(g_mode_fsm.command_id == 7u && g_route.enabled);
    CHECK(DebugLog_Display() == OLED_OK && log_has("DEBUG_END"));
    CHECK(strstr(s_oled_rows[0], "R:CRUISE") != NULL);
    command = g_mode_fsm.command_id;
    ticks(1500u); /* 超过原第二段 13 秒上限，截止后的循迹不能再被路线超时停止。 */
    CHECK(g_route.state == ROUTE_CRUISE && g_mode_fsm.status == CAR_RUNNING);
    CHECK(g_mode_fsm.command_id == command && s_beep_starts == 2u);
    CHECK(g_mode_fsm.turn_count == 1u && !g_route.junction_event);
    set_sensors(0x18u);
    ticks(100u);
    CHECK(motorLeft.speed == 400 && motorRight.speed == 400);
    puts("PASS exact first two segment durations, cross after right edge, 500 ms approach and cruise");
}

/**
 * @brief 验证减速带后的十字直行、2 秒循迹、按当前 PWM 的 5 秒 IMU，以及下一十字右转。
 * @param 无。
 * @return 无；断言失败立即结束测试。
 * @note 前两段使用短场景抵达减速带出口，随后恢复生产参数，逐帧核对新增阶段边界。
 *       两次恢复持续给宽线，证明不能把项目图案当下一十字；截止 4 保留持续循迹。
 */
static void test_route_extension(void)
{
    uint32_t command, turns;
    unsigned track_before, imu_before;
    if (TEST_ACTIVE_STEP_COUNT < 3u) return;

    begin_case("cross straight, exact 2s track and 5s IMU then terminal right turn", 1u);
    reach_route_state(1u, ROUTE_RECOVER);
    memcpy(g_route_plan, s_original_plan, sizeof(s_original_plan));
    CHECK(g_route_plan[2].action == ROUTE_ACTION_STRAIGHT);
    CHECK(g_route_plan[2].trigger_junction == JUNCTION_CROSS);
    CHECK(g_route_plan[2].entry_ms == 2000u && g_route_plan[2].pass_ms == 5000u);
    CHECK(g_route_plan[3].action == ROUTE_ACTION_RIGHT_TRACK);
    CHECK(g_route_plan[3].entry_ms == 0u && g_route_plan[3].pass_ms == 0u);
    CHECK(g_route_plan[3].base_pwm == 0u && g_route_plan[3].trigger_junction == JUNCTION_CROSS);
    command = g_mode_fsm.command_id;
    turns = g_mode_fsm.turn_count;
    set_sensors(0xFFu);
    ticks(20u);
    CHECK(g_route.state == ROUTE_RECOVER && g_route.step_index == 1u);
    CHECK(g_mode_fsm.command_id == command && s_beep_starts == 2u);
    set_sensors(0x18u);
    tick();
    CHECK(g_route.state == ROUTE_RECOVER);
    tick();
    CHECK(g_route.state == ROUTE_FOLLOW && g_route.step_index == 2u);

    set_sensors(0x0Fu);
    ticks(2u); /* 先左侧锁存，再稳定十字，仍须触发直行入口。 */
    CHECK(g_route.state == ROUTE_FOLLOW && s_beep_starts == 2u);
    set_sensors(0xFFu);
    tick();
    CHECK(g_route.state == ROUTE_FOLLOW);
    tick();
    CHECK(g_route.state == ROUTE_ENTRY && g_route.state_frames == 0u);
    CHECK(!g_route.junction_event && s_beep_on && s_beep_starts == 3u);
    CHECK(g_mode_fsm.command_id == command && g_mode_fsm.turn_count == turns);
    track_before = s_track_pid_calls;
    set_sensors(0x20u);
    tick();
    CHECK(motorLeft.speed > motorRight.speed && s_track_pid_calls == track_before + 1u);
    ticks(8u);
    CHECK(s_beep_on);
    tick();
    CHECK(!s_beep_on && s_beep_ends == 3u && s_beep_last_frames == 10u);
    set_sensors(0xFFu);
    ticks(189u);
    CHECK(g_route.state == ROUTE_ENTRY && g_route.state_frames == 199u);
    CHECK(g_mode_fsm.command_id == command && s_beep_starts == 3u);
    tick();
    CHECK(g_route.state == ROUTE_OBSTACLE && g_route.state_frames == 0u);
    CHECK(g_mode_fsm.mode == MODE_IMU && g_mode_fsm.base_pwm == s_original_plan[2].base_pwm);
    CHECK(g_mode_fsm.command_id == command + 1u);
    track_before = s_track_pid_calls;
    imu_before = s_imu_reads;
    ticks(499u);
    CHECK(g_route.state == ROUTE_OBSTACLE && g_route.state_frames == 499u);
    CHECK(s_track_pid_calls == track_before && s_imu_reads == imu_before + 499u);
    CHECK(g_mode_fsm.command_id == command + 1u && !g_route.junction_event);
    tick(); /* 仍为全黑宽线，5000 ms 到时必须立即恢复灰度循迹。 */
    CHECK(g_mode_fsm.mode == MODE_TRACK && g_mode_fsm.command_id == command + 2u);
    CHECK(s_track_pid_calls == track_before + 1u && s_imu_reads == imu_before + 499u);

    if (TEST_ACTIVE_STEP_COUNT == 3u) {
        CHECK(g_route.state == ROUTE_CRUISE && g_route.step_index == 2u);
        command = g_mode_fsm.command_id;
        ticks(1500u);
        CHECK(g_route.state == ROUTE_CRUISE && g_mode_fsm.status == CAR_RUNNING);
        CHECK(g_mode_fsm.command_id == command && g_mode_fsm.turn_count == turns);
        CHECK(s_beep_starts == 3u);
        puts("PASS third-segment cutoff after exact 2000/5000 ms and three junction pulses");
        return;
    }
    CHECK(g_route.state == ROUTE_RECOVER && g_route.step_index == 2u);
    ticks(20u);
    CHECK(g_route.state == ROUTE_RECOVER && s_beep_starts == 3u);
    set_sensors(0x18u);
    tick();
    CHECK(g_route.state == ROUTE_RECOVER);
    set_sensors(0xFFu);
    ticks(2u); /* 窄线不足两帧，重新遇宽线仍不能启动末尾右转。 */
    CHECK(g_route.state == ROUTE_RECOVER && s_beep_starts == 3u);
    set_sensors(0x18u);
    ticks(2u);
    CHECK(g_route.state == ROUTE_FOLLOW && g_route.step_index == 3u);
    set_sensors(0xF0u);
    ticks(2u);
    CHECK(g_route.state == ROUTE_FOLLOW && s_beep_starts == 3u);
    command = g_mode_fsm.command_id;
    set_sensors(0xFFu);
    tick();
    CHECK(g_route.state == ROUTE_FOLLOW);
    tick();
    CHECK(g_route.state == ROUTE_APPROACH && s_beep_on && s_beep_starts == 4u);
    CHECK(g_mode_fsm.command_id == command + 1u && g_mode_fsm.base_pwm == 500u);
    ticks(9u);
    CHECK(s_beep_on);
    tick();
    CHECK(!s_beep_on && s_beep_ends == 4u && s_beep_last_frames == 10u);
    ticks(39u);
    CHECK(g_route.state == ROUTE_APPROACH && g_route.state_frames == 49u);
    CHECK(g_mode_fsm.command_id == command + 1u);
    tick();
    CHECK(g_route.state == ROUTE_TURN && g_mode_fsm.command_id == command + 2u);
    CHECK(g_mode_fsm.imu_state == IMU_TURN_RIGHT && CLOSE(g_mode_fsm.target_yaw, -90.0f));
    ticks(20u);
    CHECK(g_route.state == ROUTE_TURN && g_mode_fsm.turn_count == turns);
    s_sample_yaw = -90.0f;
    ticks(2u);
    CHECK(g_mode_fsm.turn_count == turns + 1u && g_mode_fsm.mode == MODE_TRACK);
    tick();
    CHECK(g_mode_fsm.command_id == command + 2u); /* 不重复请求循迹，更不能再发项目 IMU。 */
    if (TEST_ACTIVE_STEP_COUNT > 4u) {
        CHECK(g_route.state == ROUTE_RECOVER && g_route.step_index == 3u);
        ticks(20u);
        CHECK(g_route.state == ROUTE_RECOVER && s_beep_starts == 4u);
        CHECK(g_mode_fsm.command_id == command + 2u);
        set_sensors(0x18u);
        ticks(2u);
        CHECK(g_route.state == ROUTE_FOLLOW && g_route.step_index == 4u);
        puts("PASS preserved third/fourth segments and narrow-line handoff to the added right corner");
        return;
    }
    CHECK(g_route.state == ROUTE_CRUISE && g_route.step_index == 3u);
    CHECK(DebugLog_Display() == OLED_OK && log_has("DEBUG_END"));
    CHECK(strstr(s_oled_rows[0], "R:CRUISE") != NULL);
    track_before = s_track_pid_calls;
    imu_before = s_imu_reads;
    ticks(1500u); /* 持续十字且超过路线超时：保持循迹，不重复转向、鸣叫或进入项目。 */
    CHECK(g_route.state == ROUTE_CRUISE && g_mode_fsm.status == CAR_RUNNING);
    CHECK(g_mode_fsm.command_id == command + 2u && g_mode_fsm.turn_count == turns + 1u);
    CHECK(s_track_pid_calls == track_before + 1500u && s_imu_reads == imu_before);
    CHECK(s_beep_starts == 4u && !g_route.junction_event);
    set_sensors(0x18u);
    ticks(10u);
    CHECK(motorLeft.speed == 400 && motorRight.speed == 400);
    puts("PASS third/fourth crosses: one pulse each, exact 2000/5000 ms at configured PWM and right-to-cruise");
}

/**
 * @brief 按生产参数逐段验证新增右弯、十字右转、十字直行和十字左转的完整时序。
 * @param 无。
 * @return 无；任何 PWM、帧数、动作次数或取消边界错误都由 CHECK 终止测试。
 * @note 前四段用短场景抵达恢复阶段，再恢复真实路线表；新增路段逐帧使用 1/2 秒入口
 *       和 4/5/6 秒通行。转弯先给未完成反馈，再连续两帧到位，不把转动时间算进入口。
 *       第 4、5 段均须恢复窄线才接下段；调试截止后持续循迹 15 秒，无后续动作或超时。
 */
static void test_route_later_steps(void)
{
    static const RouteStep expected[] = {
        {ROUTE_ACTION_RIGHT_TRACK, 5u, 0u, 0u, 0u, 10000u, JUNCTION_RIGHT},
        {ROUTE_ACTION_RIGHT, 6u, 500u, 1000u, 5000u, 10000u, JUNCTION_CROSS},
        {ROUTE_ACTION_STRAIGHT, 7u, 500u, 2000u, 4000u, 10000u, JUNCTION_CROSS},
        {ROUTE_ACTION_LEFT, 8u, 600u, 1000u, 6000u, 10000u, JUNCTION_CROSS}
    };
    unsigned index;
    if (TEST_ACTIVE_STEP_COUNT < 5u) return;

    begin_case("production right corner, cross right, straight and cross left", 1u);
    reach_route_state(3u, ROUTE_RECOVER);
    memcpy(g_route_plan, s_original_plan, sizeof(s_original_plan));
    set_sensors(0xFFu);
    ticks(20u);
    CHECK(g_route.state == ROUTE_RECOVER && g_route.step_index == 3u);
    CHECK(s_beep_starts == 4u);
    set_sensors(0x18u);
    ticks(2u);

    for (index = 4u; index < TEST_ACTIVE_STEP_COUNT && index < 8u; ++index) {
        const RouteStep *step = &g_route_plan[index];
        const RouteStep *planned = &expected[index - 4u];
        uint32_t command = g_mode_fsm.command_id;
        uint32_t turns = g_mode_fsm.turn_count;
        unsigned beep_before = s_beep_starts;
        unsigned expected_commands = 0u;
        unsigned entry_start_frame, track_before, imu_before;
        CHECK(g_route.state == ROUTE_FOLLOW && g_route.step_index == index);
        CHECK(step->action == planned->action && step->project_id == planned->project_id);
        CHECK(step->base_pwm == planned->base_pwm && step->entry_ms == planned->entry_ms);
        CHECK(step->pass_ms == planned->pass_ms && step->timeout_ms == planned->timeout_ms);
        CHECK(step->trigger_junction == planned->trigger_junction);

        s_sample_yaw = 0.0f;
        if (step->trigger_junction == JUNCTION_RIGHT) {
            set_sensors(0xFFu);
            ticks(2u);
            set_sensors(0xF0u);
            ticks(2u); /* 同一宽区从十字变右口，仍须离开后再产生新的右口事件。 */
            CHECK(g_route.state == ROUTE_FOLLOW && s_beep_starts == beep_before);
            set_sensors(0x18u);
            ticks(2u);
            set_sensors(0xF0u);
        } else {
            set_sensors(0xF0u);
            ticks(2u); /* 单侧先锁存，随后稳定十字仍能触发目标动作。 */
            CHECK(g_route.state == ROUTE_FOLLOW && s_beep_starts == beep_before);
            set_sensors(0xFFu);
        }
        tick();
        CHECK(g_route.state == ROUTE_FOLLOW && g_mode_fsm.command_id == command);
        tick();
        entry_start_frame = s_frame;
        CHECK(s_beep_on && s_beep_starts == beep_before + 1u);
        CHECK(g_mode_fsm.turn_count == turns);
        CHECK(g_route.state == (step->action == ROUTE_ACTION_STRAIGHT ? ROUTE_ENTRY : ROUTE_APPROACH));
        ticks(9u);
        CHECK(s_beep_on && s_beep_starts == beep_before + 1u);
        tick();
        CHECK(!s_beep_on && s_beep_last_frames == 10u && s_beep_ends == s_beep_starts);

        if (step->action != ROUTE_ACTION_STRAIGHT) {
            CHECK(g_mode_fsm.command_id == command + 1u && g_mode_fsm.base_pwm == 500u);
            ticks(39u);
            CHECK(g_route.state == ROUTE_APPROACH && g_route.state_frames == 49u);
            tick();
            expected_commands = 2u;
            CHECK(g_route.state == ROUTE_TURN && g_mode_fsm.command_id == command + 2u);
            CHECK(g_mode_fsm.imu_state == (step->action == ROUTE_ACTION_LEFT ? IMU_TURN_LEFT : IMU_TURN_RIGHT));
            CHECK(CLOSE(g_mode_fsm.target_yaw, step->action == ROUTE_ACTION_LEFT ? 90.0f : -90.0f));
            ticks(20u);
            CHECK(g_route.state == ROUTE_TURN && g_mode_fsm.turn_count == turns);
            s_sample_yaw = step->action == ROUTE_ACTION_LEFT ? 90.0f : -90.0f;
            tick();
            CHECK(g_mode_fsm.turn_count == turns);
            tick();
            CHECK(g_mode_fsm.turn_count == turns + 1u && g_mode_fsm.mode == MODE_TRACK);
            entry_start_frame = s_frame;
            s_sample_yaw = 0.0f;
            tick();
            CHECK(g_mode_fsm.command_id == command + 2u); /* 转完衔接循迹不重复发 TRACK。 */
        } else {
            CHECK(g_mode_fsm.command_id == command && g_mode_fsm.turn_count == turns);
        }

        if (step->action != ROUTE_ACTION_RIGHT_TRACK) {
            unsigned entry_frames = planned->entry_ms / 10u;
            unsigned pass_frames = planned->pass_ms / 10u;
            CHECK(g_route.state == ROUTE_ENTRY && g_mode_fsm.mode == MODE_TRACK);
            track_before = s_track_pid_calls;
            set_sensors(0x20u);
            tick();
            CHECK(s_track_pid_calls == track_before + 1u && motorLeft.speed > motorRight.speed);
            ticks(entry_frames - 1u - g_route.state_frames);
            CHECK(g_route.state == ROUTE_ENTRY && g_route.state_frames == entry_frames - 1u);
            CHECK(g_mode_fsm.command_id == command + expected_commands);
            tick();
            CHECK(s_frame - entry_start_frame == entry_frames);
            CHECK(g_route.state == ROUTE_OBSTACLE && g_route.state_frames == 0u);
            CHECK(g_mode_fsm.mode == MODE_IMU && g_mode_fsm.base_pwm == planned->base_pwm);
            CHECK(g_mode_fsm.command_id == command + expected_commands + 1u);
            CHECK(motorLeft.speed == planned->base_pwm && motorRight.speed == planned->base_pwm);
            track_before = s_track_pid_calls;
            imu_before = s_imu_reads;
            set_sensors(0xFFu);
            ticks(pass_frames - 1u);
            CHECK(g_route.state == ROUTE_OBSTACLE && g_route.state_frames == pass_frames - 1u);
            CHECK(g_mode_fsm.command_id == command + expected_commands + 1u);
            CHECK(s_track_pid_calls == track_before && s_imu_reads == imu_before + pass_frames - 1u);
            tick(); /* 计时到达即循迹，不依赖宽线变窄，也不额外读取一帧 IMU。 */
            expected_commands += 2u;
            CHECK(s_track_pid_calls == track_before + 1u && s_imu_reads == imu_before + pass_frames - 1u);
        }
        CHECK(g_mode_fsm.mode == MODE_TRACK && g_mode_fsm.status == CAR_RUNNING);
        CHECK(g_mode_fsm.command_id == command + expected_commands);
        CHECK(s_beep_starts == beep_before + 1u && !s_beep_on);
        CHECK(g_mode_fsm.turn_count == turns + (step->action == ROUTE_ACTION_STRAIGHT ? 0u : 1u));

        if (index + 1u == TEST_ACTIVE_STEP_COUNT) {
            CHECK(g_route.state == ROUTE_CRUISE && g_route.step_index == index);
            CHECK(DebugLog_Display() == OLED_OK && log_has("DEBUG_END"));
            track_before = s_track_pid_calls;
            imu_before = s_imu_reads;
            set_sensors(0xFFu);
            ticks(1500u);
            CHECK(g_route.state == ROUTE_CRUISE && g_mode_fsm.status == CAR_RUNNING);
            CHECK(g_mode_fsm.command_id == command + expected_commands);
            CHECK(s_track_pid_calls == track_before + 1500u && s_imu_reads == imu_before);
            CHECK(s_beep_starts == beep_before + 1u && !g_route.junction_event);
            set_sensors(0x18u);
            ticks(10u);
            CHECK(motorLeft.speed == 400 && motorRight.speed == 400);
        } else {
            CHECK(g_route.state == ROUTE_RECOVER && g_route.step_index == index);
            set_sensors(0xFFu);
            ticks(20u);
            CHECK(g_route.state == ROUTE_RECOVER && s_beep_starts == beep_before + 1u);
            CHECK(g_mode_fsm.command_id == command + expected_commands);
            set_sensors(0x18u);
            tick();
            set_sensors(0xFFu);
            tick(); /* 只有一帧窄线不准接收下一路口。 */
            CHECK(g_route.state == ROUTE_RECOVER);
            set_sensors(0x18u);
            tick();
            CHECK(g_route.state == ROUTE_RECOVER);
            tick();
            CHECK(g_route.state == ROUTE_FOLLOW && g_route.step_index == index + 1u);
            CHECK(g_mode_fsm.command_id == command + expected_commands);
        }
    }
    puts("PASS added right corner and three crosses: PWM500/500/600, exact entry/pass timing and cutoff");
}

/**
 * @brief 验证第 9 段的 1 秒循迹、5 秒 IMU，以及末尾原地左转和 3 秒直行停车。
 * @param 无。
 * @return 无；帧边界、动作次数或控制模式错误时由 CHECK 立即结束测试。
 * @note 前八段用短时间场景抵达恢复阶段，再恢复真实路线表；第九段使用生产参数。
 *       恢复阶段须连续两帧中心窄线才接收十字；入口从实际转完计时，不重复发 TRACK。
 *       截止 8 保持原持续循迹；默认 9 和全部段追加原地左转，按完成帧计直行 300 帧后停车。
 */
static void test_route_ninth_step(void)
{
    uint32_t command, turns;
    unsigned beep_before, track_before, imu_before, turn_done_frame, budget;
    if (TEST_ACTIVE_STEP_COUNT < 8u) return;
    begin_case("ninth cross left, 100ms beep, 1s track and PWM600 for 5s", 1u);
    if (TEST_ACTIVE_STEP_COUNT == 8u) {
        reach_route_state(7u, ROUTE_CRUISE);
        command = g_mode_fsm.command_id;
        beep_before = s_beep_starts;
        set_sensors(0xFFu);
        ticks(1500u);
        CHECK(g_route.state == ROUTE_CRUISE && g_route.step_index == 7u);
        CHECK(g_mode_fsm.status == CAR_RUNNING && g_mode_fsm.command_id == command);
        CHECK(s_beep_starts == beep_before);
        puts("PASS eighth-segment cutoff retains cruise and does not start ninth step");
        return;
    }
    reach_route_state(7u, ROUTE_RECOVER);
    memcpy(g_route_plan, s_original_plan, sizeof(s_original_plan));
    CHECK(Route_IsCalibrated());
    CHECK(g_route_plan[8].action == ROUTE_ACTION_LEFT && g_route_plan[8].project_id == 9u);
    CHECK(g_route_plan[8].trigger_junction == JUNCTION_CROSS && g_route_plan[8].base_pwm == 600u);
    CHECK(g_route_plan[8].entry_ms == 1000u && g_route_plan[8].pass_ms == 5000u);
    command = g_mode_fsm.command_id;
    turns = g_mode_fsm.turn_count;
    beep_before = s_beep_starts;
    set_sensors(0xFFu);
    ticks(20u);
    CHECK(g_route.state == ROUTE_RECOVER && g_route.step_index == 7u);
    CHECK(g_mode_fsm.command_id == command && s_beep_starts == beep_before);
    set_sensors(0x18u);
    tick();
    CHECK(g_route.state == ROUTE_RECOVER);
    set_sensors(0xFFu);
    tick(); /* 一帧窄线后又遇宽线，不能把原项目图案当第九段十字。 */
    CHECK(g_route.state == ROUTE_RECOVER);
    set_sensors(0x18u);
    ticks(2u);
    CHECK(g_route.state == ROUTE_FOLLOW && g_route.step_index == 8u);
    CHECK(g_mode_fsm.command_id == command);

    s_sample_yaw = 0.0f;
    set_sensors(0x0Fu);
    ticks(2u);
    CHECK(g_route.state == ROUTE_FOLLOW && s_beep_starts == beep_before);
    set_sensors(0xFFu);
    tick();
    CHECK(g_route.state == ROUTE_FOLLOW);
    tick();
    CHECK(g_route.state == ROUTE_APPROACH && !g_route.junction_event);
    CHECK(s_beep_on && s_beep_starts == beep_before + 1u);
    CHECK(g_mode_fsm.command_id == command + 1u && g_mode_fsm.imu_state == IMU_FORWARD);
    ticks(9u);
    CHECK(s_beep_on && s_beep_starts == beep_before + 1u);
    tick();
    CHECK(!s_beep_on && s_beep_last_frames == 10u);
    budget = 200u;
    while (g_route.state == ROUTE_APPROACH && budget-- != 0u) {
        CHECK(g_mode_fsm.command_id == command + 1u);
        tick();
    }
    CHECK(budget != 0u && g_route.state == ROUTE_TURN);
    CHECK(g_mode_fsm.imu_state == IMU_TURN_LEFT && CLOSE(g_mode_fsm.target_yaw, 90.0f));
    CHECK(g_mode_fsm.command_id == command + 2u);
    ticks(20u);
    CHECK(g_route.state == ROUTE_TURN && g_mode_fsm.turn_count == turns);
    s_sample_yaw = 90.0f;
    tick();
    CHECK(g_mode_fsm.turn_count == turns);
    tick();
    CHECK(g_mode_fsm.turn_count == turns + 1u && g_mode_fsm.mode == MODE_TRACK);
    turn_done_frame = s_frame;
    s_sample_yaw = 0.0f;
    tick();
    CHECK(g_route.state == ROUTE_ENTRY && g_route.state_frames == 1u);
    CHECK(g_mode_fsm.command_id == command + 2u);
    track_before = s_track_pid_calls;
    set_sensors(0x20u);
    tick();
    CHECK(s_track_pid_calls == track_before + 1u && motorLeft.speed > motorRight.speed);
    ticks(97u);
    CHECK(g_route.state == ROUTE_ENTRY && g_route.state_frames == 99u);
    CHECK(g_mode_fsm.command_id == command + 2u);
    tick();
    CHECK(s_frame - turn_done_frame == 100u);
    CHECK(g_route.state == ROUTE_OBSTACLE && g_route.state_frames == 0u);
    CHECK(g_mode_fsm.mode == MODE_IMU && g_mode_fsm.base_pwm == 600u);
    CHECK(g_mode_fsm.command_id == command + 3u);
    CHECK(motorLeft.speed == 600 && motorRight.speed == 600);
    track_before = s_track_pid_calls;
    imu_before = s_imu_reads;
    set_sensors(0xFFu);
    ticks(499u);
    CHECK(g_route.state == ROUTE_OBSTACLE && g_route.state_frames == 499u);
    CHECK(g_mode_fsm.command_id == command + 3u && s_track_pid_calls == track_before);
    tick();
    CHECK(g_route.state == ROUTE_FINAL_TURN && g_route.step_index == 8u);
    CHECK(g_mode_fsm.mode == MODE_IMU && g_mode_fsm.imu_state == IMU_TURN_LEFT);
    CHECK(g_mode_fsm.command_id == command + 4u && CLOSE(g_mode_fsm.target_yaw, 90.0f));
    CHECK(motorLeft.speed < 0 && motorRight.speed == -motorLeft.speed);
    CHECK(s_track_pid_calls == track_before && s_imu_reads == imu_before + 500u);
    CHECK(DebugLog_Display() == OLED_OK && strstr(s_oled_rows[0], "R:END_L") != NULL);
    ticks(20u); /* 转动等待不计入后面的 3 秒，也不能重复提交左转。 */
    CHECK(g_route.state == ROUTE_FINAL_TURN && g_mode_fsm.command_id == command + 4u);
    CHECK(g_mode_fsm.turn_count == turns + 1u);
    s_sample_yaw = 90.0f;
    tick();
    CHECK(g_mode_fsm.turn_count == turns + 1u && motorLeft.speed == 0 && motorRight.speed == 0);
    tick();
    CHECK(g_mode_fsm.turn_count == turns + 2u && g_mode_fsm.imu_state == IMU_FORWARD);
    CHECK(g_mode_fsm.base_pwm == 500u && CLOSE(g_mode_fsm.target_yaw, 90.0f));
    CHECK(motorLeft.speed == 500 && motorRight.speed == 500);
    turn_done_frame = s_frame;
    imu_before = s_imu_reads;
    tick();
    CHECK(g_route.state == ROUTE_FINAL_FORWARD && g_route.state_frames == 1u);
    CHECK(DebugLog_Display() == OLED_OK && strstr(s_oled_rows[0], "R:END_F") != NULL);
    ticks(298u);
    CHECK(s_frame - turn_done_frame == 299u && g_route.state_frames == 299u);
    CHECK(g_mode_fsm.status == CAR_RUNNING && g_mode_fsm.command_id == command + 4u);
    CHECK(s_track_pid_calls == track_before && s_imu_reads == imu_before + 299u);
    CHECK(motorLeft.speed == 500 && motorRight.speed == 500);
    tick();
    CHECK(s_frame - turn_done_frame == 300u);
    CHECK(g_route.state == ROUTE_FINISHED && !g_route.enabled && g_route.step_index == 8u);
    check_stopped(CAR_STOPPED, CAR_FAULT_NONE);
    CHECK(g_mode_fsm.command_id == command + 5u && s_imu_reads == imu_before + 299u);
    CHECK(DebugLog_Display() == OLED_OK && log_has("ROUTE_DONE"));
    CHECK(strstr(s_oled_rows[0], "R:DONE") != NULL);
    ticks(1500u);
    CHECK(g_route.state == ROUTE_FINISHED && g_mode_fsm.command_id == command + 5u);
    check_stopped(CAR_STOPPED, CAR_FAULT_NONE);
    CHECK(s_beep_starts == beep_before + 1u && !g_route.junction_event);
    Route_Enable(1u);
    ticks(10u);
    CHECK(g_route.state == ROUTE_FINISHED && g_mode_fsm.command_id == command + 5u);
    check_stopped(CAR_STOPPED, CAR_FAULT_NONE);
    CHECK(Route_Start()); /* 正常完成后也只能显式从起点重新启动。 */
    tick();
    CHECK(g_route.state == ROUTE_FOLLOW && g_route.step_index == 0u);
    puts("PASS ninth step: exact 1s/5s, in-place left turn, retained heading, exact 3s and stopped finish");
}

/**
 * @brief 验证收尾左转和直行阶段的停车、外部接管、禁用、IMU 故障及转弯超时。
 * @param 无。
 * @return 无；检查实际两轮输出、请求次数和中止后不自动续跑。
 * @note 使用真实 Route/FSM，前九段缩短时间；末尾 3 秒仍按生产常量执行。
 */
static void test_route_finish_interrupts(void)
{
    static const RouteState phases[] = {ROUTE_FINAL_TURN, ROUTE_FINAL_FORWARD};
    unsigned phase, action;
    uint32_t command;
    if (TEST_ACTIVE_STEP_COUNT < ROUTE_STEP_COUNT) return;
    for (phase = 0u; phase < sizeof(phases) / sizeof(phases[0]); ++phase) {
        for (action = 0u; action < 4u; ++action) {
            begin_case("final turn/forward can be interrupted without resuming", 1u);
            reach_route_state(8u, phases[phase]);
            if (action == 0u) Car_Stop();
            else if (action == 1u) Car_Track();
            else if (action == 2u) Route_Enable(0u);
            else s_sample_ok = 0u;
            tick();
            if (action == 3u) {
                check_stopped(CAR_FAULT, CAR_FAULT_IMU);
                s_sample_ok = g_imu_ready = 1u;
                tick(); /* 运动层采样失败后，路线在下一帧观察故障并取消。 */
            }
            CHECK(g_route.state == ROUTE_ABORTED && g_route.step_index == 8u);
            if (action == 0u) check_stopped(CAR_STOPPED, CAR_FAULT_NONE);
            else if (action == 1u) CHECK(g_mode_fsm.mode == MODE_TRACK && g_mode_fsm.status == CAR_RUNNING);
            else if (action == 2u) CHECK(g_mode_fsm.mode == MODE_IMU && g_mode_fsm.status == CAR_RUNNING);
            command = g_mode_fsm.command_id;
            Route_Enable(1u);
            s_sample_yaw = 90.0f;
            ticks(350u); /* 超过正常直行时长，中止后不能再由路线发出停车或运动。 */
            CHECK(g_route.state == ROUTE_ABORTED && g_mode_fsm.command_id == command);
            if (action == 0u) check_stopped(CAR_STOPPED, CAR_FAULT_NONE);
            else if (action == 3u) check_stopped(CAR_FAULT, CAR_FAULT_IMU);
        }
    }
    begin_case("final in-place turn retains five-second timeout", 1u);
    g_route_plan[8].timeout_ms = 10000u;
    reach_route_state(8u, ROUTE_FINAL_TURN);
    s_sample_yaw = 0.0f;
    ticks(498u);
    CHECK(g_route.state == ROUTE_FINAL_TURN && g_mode_fsm.status == CAR_RUNNING);
    tick();
    check_stopped(CAR_FAULT, CAR_FAULT_TURN_TIMEOUT);
    tick();
    CHECK(g_route.state == ROUTE_ABORTED);
    puts("PASS final turn/forward STOP, takeover, disable, IMU failure and turn timeout");
}

/**
 * @brief 验证各指定十字直接出现、先左侧或先右侧出现时，都只执行一次当前段动作。
 * @param 无。
 * @return 无。
 * @note 单帧十字及中断后的单帧十字均不能触发；转弯段两帧确认后仍先前移 500 ms。
 *       复用真实 Route/FSM 和控制器，模拟反馈只用于验证软件阶段与请求次数。
 */
static void test_route_cross_edges(void)
{
    static const uint8_t edge_masks[] = {0x18u, 0x0Fu, 0xF0u};
    unsigned i, index;
    uint32_t command, turns;

    for (index = 1u; index < TEST_ACTIVE_STEP_COUNT; ++index) {
        const RouteStep *step = &s_original_plan[index];
        if (step->trigger_junction != JUNCTION_CROSS) continue;
        for (i = 0u; i < sizeof(edge_masks) / sizeof(edge_masks[0]); ++i) {
            unsigned beep_before;
            begin_case("confirmed cross after none, left or right edge", 1u);
            reach_route_state(index, ROUTE_FOLLOW);
            command = g_mode_fsm.command_id;
            turns = g_mode_fsm.turn_count;
            beep_before = s_beep_starts;
            set_sensors(edge_masks[i]);
            ticks(2u);
            CHECK(g_route.state == ROUTE_FOLLOW && g_mode_fsm.command_id == command);

            set_sensors(0xFFu);
            tick();
            CHECK(g_route.state == ROUTE_FOLLOW);
            set_sensors(0x18u);
            tick(); /* 只离开一帧，不能解除原单侧图案的事件锁存。 */
            set_sensors(0xFFu);
            tick();
            CHECK(g_route.state == ROUTE_FOLLOW && g_mode_fsm.command_id == command);
            tick();
            CHECK(g_route.junction_event == (uint8_t)(i == 0u));
            CHECK(s_beep_starts == beep_before + 1u);
            if (step->action == ROUTE_ACTION_STRAIGHT) {
                CHECK(g_route.state == ROUTE_ENTRY && g_mode_fsm.command_id == command);
                ticks(2u);
                CHECK(g_route.state == ROUTE_ENTRY && g_mode_fsm.command_id == command);
                CHECK(g_mode_fsm.turn_count == turns && s_beep_starts == beep_before + 1u);
                continue;
            }
            CHECK(g_route.state == ROUTE_APPROACH && g_mode_fsm.command_id == command + 1u);

            ticks(49u); /* 持续八路在线，不能重复提交前移或提前转弯。 */
            CHECK(g_route.state == ROUTE_APPROACH && g_mode_fsm.command_id == command + 1u);
            tick();
            CHECK(g_route.state == ROUTE_TURN && g_mode_fsm.command_id == command + 2u);
            CHECK(g_mode_fsm.imu_state == (step->action == ROUTE_ACTION_LEFT ? IMU_TURN_LEFT : IMU_TURN_RIGHT));
            CHECK(CLOSE(g_mode_fsm.target_yaw, step->action == ROUTE_ACTION_LEFT ? 90.0f : -90.0f));
            s_sample_yaw = step->action == ROUTE_ACTION_LEFT ? 90.0f : -90.0f;
            ticks(2u);
            CHECK(g_mode_fsm.turn_count == turns + 1u && g_mode_fsm.mode == MODE_TRACK);
            tick();
            if (step->action == ROUTE_ACTION_RIGHT_TRACK) {
                CHECK(g_route.state == (index + 1u == TEST_ACTIVE_STEP_COUNT ? ROUTE_CRUISE : ROUTE_RECOVER));
            } else {
                CHECK(g_route.state == ROUTE_ENTRY);
            }
            CHECK(g_mode_fsm.command_id == command + 2u);
        }
    }
    puts("PASS each cross: direct and both side-to-cross transitions, debounce and single action");
}

/**
 * @brief 区分“正常转完但灰度无线”和“接近目标却未满足完成条件”的两种停车线索。
 * @param 无。
 * @return 无。
 * @note 原 3 秒截止时仅转到 -83.2°仍应继续；随后达标立即接入口，不必等满 5 秒。
 *       第 500 帧才到 -86°仍须确认后接循迹；无线时输出 240/240 并继续第二段。
 *       误差持续 5.1° 时保留 5 秒故障保护，不得跳过未完成转弯。
 *       这些场景只验证软件分支，实际轮子在低 PWM 下能否推动小车仍须实车观察。
 */
static void test_route_turn_stop_diagnosis(void)
{
    uint32_t command;
    begin_case("turn past old three-second deadline then continues entry and project", 1u);
    memcpy(g_route_plan, s_original_plan, sizeof(s_original_plan));
    reach_route_state(1u, ROUTE_TURN);
    command = g_mode_fsm.command_id;
    s_sample_yaw = -83.2f;
    ticks(299u); /* 转弯开始帧已执行一次；累计 300 帧时仍允许继续纠偏。 */
    CHECK(g_mode_fsm.state_frames == 300u && g_mode_fsm.status == CAR_RUNNING);
    CHECK(g_route.state == ROUTE_TURN && g_mode_fsm.turn_count == 0u);
    ticks(99u);
    s_sample_yaw = -86.0f;
    tick();
    CHECK(g_mode_fsm.turn_count == 0u && g_mode_fsm.status == CAR_RUNNING);
    tick(); /* 约 4 秒达标后立即切换，不把 5 秒上限当作固定转弯时长。 */
    CHECK(g_mode_fsm.turn_count == 1u && g_mode_fsm.mode == MODE_TRACK);
    CHECK(g_mode_fsm.command_id == command);
    tick();
    CHECK(g_route.state == ROUTE_ENTRY && g_route.state_frames == 1u);
    ticks(198u);
    CHECK(g_route.state == ROUTE_ENTRY && g_route.state_frames == 199u);
    tick();
    CHECK(g_route.state == ROUTE_OBSTACLE && g_mode_fsm.mode == MODE_IMU);
    CHECK(g_mode_fsm.base_pwm == s_original_plan[1].base_pwm && g_mode_fsm.command_id == command + 1u);

    begin_case("completed route turn keeps tracking even with all sensors off line", 1u);
    memcpy(g_route_plan, s_original_plan, sizeof(s_original_plan));
    reach_route_state(1u, ROUTE_TURN);
    ticks(498u); /* 正式转弯已执行一帧，再等待 498 帧，到截止前一帧仍未进入容差。 */
    set_sensors(0u);
    s_sample_yaw = -86.0f;
    ticks(2u);
    CHECK(g_mode_fsm.turn_count == 1u && g_mode_fsm.mode == MODE_TRACK);
    CHECK(g_mode_fsm.status == CAR_RUNNING && g_route.state == ROUTE_TURN);
    CHECK(motorLeft.speed == 240 && motorRight.speed == 240);
    command = g_mode_fsm.command_id;
    s_sample_yaw = 0.0f;
    tick();
    CHECK(g_route.state == ROUTE_ENTRY && g_route.state_frames == 1u);
    ticks(198u);
    CHECK(g_route.state == ROUTE_ENTRY && g_route.state_frames == 199u);
    CHECK(g_mode_fsm.status == CAR_RUNNING && g_mode_fsm.command_id == command);
    CHECK(motorLeft.speed == 240 && motorRight.speed == 240);
    tick();
    CHECK(g_route.state == ROUTE_OBSTACLE && g_mode_fsm.mode == MODE_IMU);
    CHECK(g_mode_fsm.command_id == command + 1u && g_mode_fsm.base_pwm == s_original_plan[1].base_pwm);
    CHECK(motorLeft.speed == s_original_plan[1].base_pwm && motorRight.speed == s_original_plan[1].base_pwm);

    begin_case("near 90 degrees without completion still produces TURN_TO", 1u);
    memcpy(g_route_plan, s_original_plan, sizeof(s_original_plan));
    reach_route_state(1u, ROUTE_TURN);
    s_sample_yaw = -84.9f; /* 与 -90° 目标相差 5.1°，外观看似转完但软件条件尚未成立。 */
    ticks(498u);
    CHECK(g_mode_fsm.status == CAR_RUNNING && g_mode_fsm.turn_count == 0u);
    CHECK(g_route.state == ROUTE_TURN && g_mode_fsm.state_frames == 499u);
    tick();
    check_stopped(CAR_FAULT, CAR_FAULT_TURN_TIMEOUT);
    CHECK(DebugLog_Display() == OLED_OK && log_has("TURN_TO"));
    s_sample_yaw = -90.0f;
    ticks(10u);
    CHECK(g_route.state == ROUTE_ABORTED && g_mode_fsm.turn_count == 0u);
    check_stopped(CAR_FAULT, CAR_FAULT_TURN_TIMEOUT);
    puts("PASS five-second turn window, immediate entry handoff and retained timeout protection");
}

/**
 * @brief 验证早到窄线不缩短 IMU 通行、等待与恢复超时，以及截止帧成功优先。
 * @param 无。
 * @return 无。
 * @note 超时保留真实运动层的故障捕获与零 PWM 输出，不通过推进段号绕过失败。
 */
static void test_project_exit_and_timeout(void)
{
    begin_case("early narrow line cannot end timed project", 1u);
    reach_route_state(0u, ROUTE_OBSTACLE);
    set_sensors(0x18u);
    ticks(4u);
    CHECK(g_route.state == ROUTE_OBSTACLE);
    tick();
    CHECK(g_route.state == ROUTE_RECOVER && g_mode_fsm.mode == MODE_TRACK);
    ticks(3u);
    CHECK(g_route.state == ROUTE_FOLLOW && g_route.step_index == 1u);

    begin_case("waiting for first right junction times out", 1u);
    tick();
    set_sensors(0x18u);
    ticks(99u);
    CHECK(g_mode_fsm.status == CAR_RUNNING);
    tick();
    check_stopped(CAR_FAULT, CAR_FAULT_ROUTE_TIMEOUT);
    CHECK(g_route.step_index == 0u);
    CHECK(DebugLog_Display() == OLED_OK && log_has("ROUTE_TO"));
    CHECK(strstr(s_oled_rows[2], "S:1 P:1 N:100") != NULL);
    CHECK(strstr(s_oled_rows[4], "L:+400 R:+400") != NULL);
    ticks(10u);
    check_stopped(CAR_FAULT, CAR_FAULT_ROUTE_TIMEOUT);
    CHECK(g_route.state == ROUTE_FOLLOW && !g_route.enabled); /* 超时现场保持原阶段。 */

    begin_case("waiting for next cross also times out", 1u);
    reach_route_state(1u, ROUTE_FOLLOW);
    set_sensors(0x18u);
    ticks(100u);
    check_stopped(CAR_FAULT, CAR_FAULT_ROUTE_TIMEOUT);
    CHECK(g_route.step_index == 1u);

    begin_case("wide exit cannot rearm and recovery times out", 1u);
    reach_route_state(0u, ROUTE_RECOVER);
    set_sensors(0xFFu);
    ticks(100u);
    check_stopped(CAR_FAULT, CAR_FAULT_ROUTE_TIMEOUT);
    CHECK(g_route.step_index == 0u);

    begin_case("valid second narrow frame wins at recovery deadline", 1u);
    reach_route_state(0u, ROUTE_RECOVER);
    set_sensors(0xFFu);
    ticks(98u);
    set_sensors(0x18u);
    ticks(2u);
    CHECK(g_route.state == ROUTE_FOLLOW && g_route.step_index == 1u);
    CHECK(g_mode_fsm.status == CAR_RUNNING);

    begin_case("route turn retains the motion layer timeout", 1u);
    g_route_plan[1].timeout_ms = 7000u; /* 留足 5 秒及确认帧，验证运动层而非短场景的路线超时。 */
    reach_route_state(1u, ROUTE_TURN);
    s_sample_yaw = 0.0f;
    ticks(499u);
    check_stopped(CAR_FAULT, CAR_FAULT_TURN_TIMEOUT);
    s_sample_yaw = -90.0f;
    ticks(10u);
    CHECK(g_route.state == ROUTE_ABORTED && g_route.step_index == 1u);
    check_stopped(CAR_FAULT, CAR_FAULT_TURN_TIMEOUT);
    puts("PASS timed passage, narrow rearm, route deadlines and retained turn fault protection");
}

/**
 * @brief 在各阶段停车、接管或禁用，确认路线不会自行恢复或推进段号。
 * @param 无。
 * @return 无。
 * @note 覆盖 READY 与最终 CRUISE；显式重新开始必须先停车并放回起点。
 */
static void test_route_interrupts(void)
{
    static const RouteState phases[] = {ROUTE_READY, ROUTE_FOLLOW, ROUTE_ENTRY,
        ROUTE_OBSTACLE, ROUTE_RECOVER, ROUTE_APPROACH, ROUTE_TURN,
        TEST_ACTIVE_STEP_COUNT == ROUTE_STEP_COUNT ? ROUTE_FINAL_FORWARD : ROUTE_CRUISE,
        ROUTE_FOLLOW, ROUTE_ENTRY, ROUTE_OBSTACLE, ROUTE_RECOVER,
        ROUTE_FOLLOW, ROUTE_APPROACH, ROUTE_TURN, ROUTE_RECOVER,
        ROUTE_FOLLOW, ROUTE_APPROACH, ROUTE_TURN, ROUTE_RECOVER,
        ROUTE_FOLLOW, ROUTE_APPROACH, ROUTE_TURN, ROUTE_ENTRY, ROUTE_OBSTACLE, ROUTE_RECOVER,
        ROUTE_FOLLOW, ROUTE_ENTRY, ROUTE_OBSTACLE, ROUTE_RECOVER,
        ROUTE_FOLLOW, ROUTE_APPROACH, ROUTE_TURN, ROUTE_ENTRY, ROUTE_OBSTACLE, ROUTE_RECOVER,
        ROUTE_FOLLOW, ROUTE_APPROACH, ROUTE_TURN, ROUTE_ENTRY, ROUTE_OBSTACLE};
    static const unsigned indices[] = {0u, 0u, 0u, 0u, 0u, 1u, 1u, TEST_ACTIVE_STEP_COUNT - 1u,
        2u, 2u, 2u, 2u, 3u, 3u, 3u, 3u, 4u, 4u, 4u, 4u,
        5u, 5u, 5u, 5u, 5u, 5u, 6u, 6u, 6u, 6u, 7u, 7u, 7u, 7u, 7u, 7u,
        8u, 8u, 8u, 8u, 8u};
    unsigned i;
    for (i = 0u; i < sizeof(phases) / sizeof(phases[0]); ++i) {
        uint16_t index;
        uint32_t command;
        if (indices[i] >= TEST_ACTIVE_STEP_COUNT ||
            (phases[i] == ROUTE_RECOVER && indices[i] + 1u == TEST_ACTIVE_STEP_COUNT)) continue;
        begin_case("STOP interrupts each navigation phase", 1u);
        reach_route_state(indices[i], phases[i]);
        index = g_route.step_index;
        Car_Stop();
        tick();
        CHECK(g_route.state == ROUTE_ABORTED && g_route.step_index == index);
        CHECK(!s_beep_on);
        check_stopped(CAR_STOPPED, CAR_FAULT_NONE);
        command = g_mode_fsm.command_id;
        Route_Enable(1u);
        set_sensors(0xFFu);
        ticks(100u);
        CHECK(g_route.state == ROUTE_ABORTED && g_route.step_index == index);
        CHECK(g_mode_fsm.command_id == command);
        check_stopped(CAR_STOPPED, CAR_FAULT_NONE);
    }
    for (i = 0u; i < TEST_ACTIVE_STEP_COUNT; ++i) {
        RouteState phase = s_original_plan[i].action == ROUTE_ACTION_RIGHT_TRACK ? ROUTE_TURN : ROUTE_OBSTACLE;
        begin_case("external track takes ownership without advancing", 1u);
        reach_route_state(i, phase);
        Car_Track();
        tick();
        CHECK(g_route.state == ROUTE_ABORTED && g_route.step_index == i);
        CHECK(g_mode_fsm.mode == MODE_TRACK && g_mode_fsm.status == CAR_RUNNING);
        CHECK(!Route_Start());
        Route_Enable(1u);
        ticks(100u);
        CHECK(g_route.state == ROUTE_ABORTED && g_route.step_index == i);

        begin_case("disable retains IMU motion but cannot resume scheduling", 1u);
        reach_route_state(i, phase);
        Route_Enable(0u);
        tick();
        CHECK(g_route.state == ROUTE_ABORTED && g_mode_fsm.mode == MODE_IMU);
        Route_Enable(1u);
        ticks(100u);
        CHECK(g_route.state == ROUTE_ABORTED && g_route.step_index == i);
        Car_Stop();
        tick();
        CHECK(Route_Start() && g_route.step_index == 0u);
        tick();
        CHECK(g_route.state == ROUTE_FOLLOW);
    }
    puts("PASS STOP in original and added phases, external ownership, disable and explicit restart");
}

/**
 * @brief 在所有 IMU 通行、转弯前移和正式转弯中注入采样失败，验证故障取消。
 * @param 无。
 * @return 无。
 * @note 恢复通信并重新允许 Route 后仍须保持故障停车，不能接着执行入口或下一段。
 */
static void test_route_imu_faults(void)
{
    static const RouteState phases[] = {ROUTE_OBSTACLE, ROUTE_APPROACH, ROUTE_TURN, ROUTE_OBSTACLE,
        ROUTE_OBSTACLE, ROUTE_APPROACH, ROUTE_TURN, ROUTE_APPROACH, ROUTE_TURN,
        ROUTE_APPROACH, ROUTE_TURN, ROUTE_OBSTACLE, ROUTE_OBSTACLE,
        ROUTE_APPROACH, ROUTE_TURN, ROUTE_OBSTACLE, ROUTE_APPROACH, ROUTE_TURN, ROUTE_OBSTACLE};
    static const unsigned indices[] = {0u, 1u, 1u, 1u, 2u, 3u, 3u, 4u, 4u, 5u, 5u, 5u,
        6u, 7u, 7u, 7u, 8u, 8u, 8u};
    unsigned i;
    for (i = 0u; i < sizeof(phases) / sizeof(phases[0]); ++i) {
        uint32_t command;
        if (indices[i] >= TEST_ACTIVE_STEP_COUNT) continue;
        begin_case("IMU fault cancels each route IMU action", 1u);
        reach_route_state(indices[i], phases[i]);
        s_sample_ok = 0u;
        tick();
        check_stopped(CAR_FAULT, CAR_FAULT_IMU);
        command = g_mode_fsm.command_id;
        s_sample_ok = g_imu_ready = 1u;
        tick();
        CHECK(g_route.state == ROUTE_ABORTED && g_route.step_index == indices[i]);
        Route_Enable(1u);
        ticks(100u);
        check_stopped(CAR_FAULT, CAR_FAULT_IMU);
        CHECK(g_mode_fsm.command_id == command);
    }
    puts("PASS IMU faults in all projects, approaches and turns cannot resume route");
}

/**
 * @brief 验证各段路口单次短鸣、独立计时，以及停车、禁用、故障取消。
 * @param 无。
 * @return 无。
 * @note 使用短场景跨越 ENTRY、OBSTACLE、RECOVER；阶段重置不得延长或重启提示音。
 */
static void test_project_beep(void)
{
    uint32_t command;
    begin_case("one pulse on first right junction survives phase changes", 1u);
    tick();
    CHECK(g_route.state == ROUTE_FOLLOW && s_beep_starts == 0u);
    reach_route_state(0u, ROUTE_ENTRY);
    CHECK(s_beep_on && s_beep_starts == 1u);
    set_sensors(0x18u);
    ticks(9u);
    CHECK(g_route.state == ROUTE_RECOVER && s_beep_on);
    tick();
    CHECK(!s_beep_on && s_beep_ends == 1u && s_beep_last_frames == 10u);
    reach_route_state(1u, ROUTE_APPROACH);
    CHECK(s_beep_on && s_beep_starts == 2u); /* 第一个项目结束后，接受第二段十字立即短鸣。 */
    command = g_mode_fsm.command_id;
    set_sensors(0xFFu);
    ticks(9u);
    CHECK(s_beep_on && s_beep_starts == 2u);
    tick();
    CHECK(!s_beep_on && s_beep_ends == 2u && s_beep_last_frames == 10u);
    CHECK(g_mode_fsm.command_id == command); /* 保持在同一十字内不能重复提交动作或蜂鸣。 */
    reach_route_state(1u, TEST_ACTIVE_STEP_COUNT == 2u ? ROUTE_CRUISE : ROUTE_RECOVER);
    CHECK(s_beep_starts == 2u && s_beep_ends == 2u);

    begin_case("STOP cancels first post-project cross pulse", 1u);
    reach_route_state(1u, ROUTE_APPROACH);
    CHECK(s_beep_on && s_beep_starts == 2u);
    Car_Stop();
    tick();
    CHECK(!s_beep_on && g_route.state == ROUTE_ABORTED);

    begin_case("STOP cancels right junction pulse", 1u);
    reach_route_state(0u, ROUTE_ENTRY);
    Car_Stop();
    tick();
    CHECK(!s_beep_on && s_beep_starts == 1u && s_beep_ends == 1u);
    ticks(20u);
    CHECK(s_beep_starts == 1u);

    begin_case("disable cancels pulse without stopping existing track", 1u);
    reach_route_state(0u, ROUTE_ENTRY);
    Route_Enable(0u);
    tick();
    CHECK(!s_beep_on && g_route.state == ROUTE_ABORTED);
    CHECK(g_mode_fsm.status == CAR_RUNNING && g_mode_fsm.mode == MODE_TRACK);

    begin_case("IMU failure cancels active pulse by next control frame", 1u);
    reach_route_state(0u, ROUTE_OBSTACLE);
    CHECK(s_beep_on);
    s_sample_ok = 0u;
    tick();
    check_stopped(CAR_FAULT, CAR_FAULT_IMU);
    s_sample_ok = g_imu_ready = 1u;
    tick();
    CHECK(!s_beep_on && g_route.state == ROUTE_ABORTED);
    ticks(100u);
    check_stopped(CAR_FAULT, CAR_FAULT_IMU);
    CHECK(s_beep_starts == 1u && s_beep_ends == 1u);
    if (TEST_ACTIVE_STEP_COUNT >= 3u) {
        begin_case("STOP cancels added straight-cross pulse", 1u);
        reach_route_state(2u, ROUTE_ENTRY);
        CHECK(s_beep_on && s_beep_starts == 3u);
        Car_Stop();
        tick();
        CHECK(!s_beep_on && g_route.state == ROUTE_ABORTED);
    }
    if (TEST_ACTIVE_STEP_COUNT >= 4u) {
        begin_case("disable cancels fourth right-cross pulse", 1u);
        reach_route_state(3u, ROUTE_APPROACH);
        CHECK(s_beep_on && s_beep_starts == 4u);
        Route_Enable(0u);
        tick();
        CHECK(!s_beep_on && g_route.state == ROUTE_ABORTED);

        begin_case("IMU failure cancels fourth right-cross pulse", 1u);
        reach_route_state(3u, ROUTE_APPROACH);
        s_sample_ok = 0u;
        tick();
        check_stopped(CAR_FAULT, CAR_FAULT_IMU);
        tick();
        CHECK(!s_beep_on && g_route.state == ROUTE_ABORTED);
    }
    puts("PASS single 100 ms junction beep, phase changes and cancellation");
}

/**
 * @brief 验证 PWM 随动作原子接受、越障大误差不反转，以及测试限时由控制帧精确结束。
 * @return 无。
 */
static void test_pwm_and_timed_motion(void)
{
    uint32_t command;
    unsigned resets;
    begin_case("PWM mailbox and bounded forward correction", 0u);
    Car_ImuForward(500u);
    Car_ImuForward(600u);
    CHECK(motorLeft.speed == 0 && g_mode_fsm.base_pwm == 500u);
    tick();
    CHECK(g_mode_fsm.base_pwm == 600u && motorLeft.speed == 600 && motorRight.speed == 600);
    command = g_mode_fsm.command_id;
    resets = s_resets;
    s_sample_yaw = 30.0f;
    ticks(5u);
    CHECK(g_mode_fsm.command_id == command && s_resets == resets);
    /* 沿用当前源码的纠偏限幅，验证两轮直行不反转且不越过硬件 PWM 范围。 */
    CHECK(motorLeft.speed >= 0 && motorLeft.speed <= 1000);
    CHECK(motorRight.speed >= 0 && motorRight.speed <= 1000);
    CHECK(motorLeft.speed > motorRight.speed);
    Car_ImuForward(1u);
    tick();
    CHECK(motorLeft.speed >= 0 && motorRight.speed >= 0);
    Car_ImuForward(1000u);
    tick();
    CHECK(motorLeft.speed <= 1000 && motorRight.speed <= 1000);
    Car_ImuForward(0u);
    tick();
    check_stopped(CAR_FAULT, CAR_FAULT_COMMAND);
    Car_ImuForward(1001u);
    tick();
    check_stopped(CAR_FAULT, CAR_FAULT_COMMAND);

    begin_case("timed test starts when accepted and stops without foreground polling", 0u);
    Car_ImuForwardTimed(600u, 1000u);
    s_milliseconds = 900000u; /* 请求在邮箱里等待的时间不能缩短有效测试时长。 */
    tick();
    CHECK(g_mode_fsm.state_frames == 1u && g_mode_fsm.base_pwm == 600u);
    ticks(99u);
    CHECK(g_mode_fsm.status == CAR_RUNNING && g_mode_fsm.state_frames == 100u);
    tick();
    check_stopped(CAR_STOPPED, CAR_FAULT_NONE);
    CHECK(g_mode_fsm.state_frames == 100u);
    ticks(3u);
    check_stopped(CAR_STOPPED, CAR_FAULT_NONE);
    Car_ImuForwardTimed(600u, 11u);
    ticks(2u);
    CHECK(g_mode_fsm.status == CAR_RUNNING && g_mode_fsm.state_frames == 2u);
    tick();
    check_stopped(CAR_STOPPED, CAR_FAULT_NONE);
    Car_ImuForwardTimed(600u, 0u);
    tick();
    check_stopped(CAR_FAULT, CAR_FAULT_COMMAND);

    begin_case("timed test fault cannot be overwritten by its deadline", 0u);
    Car_ImuForwardTimed(600u, 1000u);
    ticks(2u);
    s_sample_ok = 0u;
    tick();
    check_stopped(CAR_FAULT, CAR_FAULT_IMU);
    ticks(110u);
    check_stopped(CAR_FAULT, CAR_FAULT_IMU);
    puts("PASS atomic PWM, non-reversing correction, invalid values and exact timed test");
}

/**
 * @brief 验证只有 TIM4 回调运行控制，其他定时器不会重复执行状态机。
 * @param 无。
 * @return 无。
 * @note 对 TIM4 使用与 tick 相同的互斥判据，非 TIM4 不产生采样或 PWM。
 */
static void test_timer_entry(void)
{
    TIM_HandleTypeDef timer;
    begin_case("TIM4 is the single callback entry", 0u);
    HAL_TIM_PeriodElapsedCallback(NULL);
    timer.Instance = (void *)(uintptr_t)3u;
    HAL_TIM_PeriodElapsedCallback(&timer);
    CHECK(s_sensor_reads == 0u && s_pid_calls == 0u && s_left_writes == 0u);
    timer.Instance = TIM4;
    ++s_frame;
    s_inside_control_frame = 1u;
    HAL_TIM_PeriodElapsedCallback(&timer);
    s_inside_control_frame = 0u;
    CHECK(s_sensor_reads == 1u && s_pid_calls == 1u && s_left_writes == 1u && s_right_writes == 1u);
    puts("PASS TIM4 exclusive callback");
}

/**
 * @brief 查找最近三条实际 OLED 日志中是否存在指定消息。
 * @param message 待查找的短英文事件文本。
 * @return 1：事件存在；0：最近三条中没有该事件。
 * @note 读取真实 DebugLog_Display 输出，不能通过直接检查内部日志数组绕过显示格式。
 */
static int log_has(const char *message)
{
    unsigned row;
    for (row = 5u; row < OLED_PAGE_COUNT; ++row) {
        if (strstr(s_oled_rows[row], message) != NULL) {
            return 1;
        }
    }
    return 0;
}

/**
 * @brief 验证只有实际接受的动作写日志，单帧条件在慢刷新后仍保留且不会重复。
 * @param 无。
 * @return 无。
 * @note 100 ms 只刷新一次，路口事件早已结束；显示仍必须有其发生时记录的内容。
 */
static void test_automatic_logs(void)
{
    char confirmed_rows[sizeof(s_oled_rows)];
    begin_case("accepted mailbox and persistent single-frame event logs", 0u);
    Car_Track();
    Car_TurnLeft90(MODE_TRACK);
    Car_TurnRight90(MODE_IMU);
    Car_ImuForward(500u);
    tick();
    CHECK(DebugLog_Display() == OLED_OK);
    CHECK(log_has("IMU_FWD"));
    CHECK(!log_has("TRACK") && !log_has("L90>TRK") && !log_has("R90>IMU"));
    Car_TurnLeft90(MODE_TRACK);
    tick();
    CHECK(DebugLog_Display() == OLED_OK && log_has("L90>TRK"));
    Car_TurnRight90(MODE_IMU);
    tick();
    CHECK(DebugLog_Display() == OLED_OK && log_has("R90>IMU"));
    s_sample_yaw = -90.0f;
    ticks(3u);
    CHECK(DebugLog_Display() == OLED_OK && log_has("DONE>IMU"));
    Car_TurnLeft90(MODE_TRACK);
    s_sample_yaw = 0.0f;
    tick();
    s_sample_yaw = 90.0f;
    ticks(3u);
    CHECK(DebugLog_Display() == OLED_OK && log_has("DONE>TRK"));
    Car_Stop();
    tick();
    CHECK(DebugLog_Display() == OLED_OK && log_has("STOP"));

    begin_case("10 ms junction event persists until 100 ms OLED refresh", 0u);
    set_sensors(0x0Fu);
    ticks(10u);
    CHECK(s_milliseconds == 100u && !g_route.junction_event);
    CHECK(DebugLog_Display() == OLED_OK && log_has("JUNC_L"));
    memcpy(confirmed_rows, s_oled_rows, sizeof(s_oled_rows));
    ticks(10u);
    CHECK(DebugLog_Display() == OLED_OK);
    CHECK(memcmp(&confirmed_rows[5u * (OLED_LINE_CHARS + 1u)], s_oled_rows[5],
                 3u * (OLED_LINE_CHARS + 1u)) == 0);
    set_sensors(0x18u);
    ticks(3u);
    set_sensors(0xF0u);
    ticks(3u);
    CHECK(DebugLog_Display() == OLED_OK && log_has("JUNC_R"));
    set_sensors(0x18u);
    ticks(3u);
    set_sensors(0xFFu);
    ticks(3u);
    CHECK(DebugLog_Display() == OLED_OK && log_has("CROSS"));
    set_sensors(0u);
    ticks(3u);
    CHECK(DebugLog_Display() == OLED_OK && log_has("LINE_OFF"));
    CHECK(strstr(s_oled_rows[7], " W LINE_OFF") != NULL);
    set_sensors(0x18u);
    ticks(3u);
    CHECK(DebugLog_Display() == OLED_OK && log_has("LINE_ON"));
    Car_ImuForward(500u);
    tick();
    set_sensors(0u);
    ticks(3u);
    CHECK(DebugLog_Display() == OLED_OK && log_has("LINE_OFF"));
    CHECK(strstr(s_oled_rows[7], " I LINE_OFF") != NULL);
    puts("PASS accepted action logs, one-frame conditions, one-shot junctions and line levels");
}

/**
 * @brief 验证真实运动故障入口先捕获 PWM，STOP 和非法请求保留冻结现场，合法动作解除。
 * @param 无。
 * @return 无。
 * @note 停车后实时电机已为零，屏幕仍必须保留 499 帧时的两轮转弯输出。
 */
static void test_fault_log_lifecycle(void)
{
    char fault_rows[sizeof(s_oled_rows)];
    char expected_pwm[OLED_LINE_CHARS + 1u];
    begin_case("real timeout snapshot, stop, invalid command and accepted recovery", 0u);
    Car_TurnLeft90(MODE_TRACK);
    ticks(500u);
    check_stopped(CAR_FAULT, CAR_FAULT_TURN_TIMEOUT);
    CHECK(DebugLog_Display() == OLED_OK);
    CHECK(log_has("TURN_TO"));
    CHECK(strstr(s_oled_rows[0], "FAULT L90") != NULL);
    CHECK(strstr(s_oled_rows[2], "N:500") != NULL);
    CHECK(strstr(s_oled_rows[4], "L:-720") != NULL && strstr(s_oled_rows[4], "R:+720") != NULL);
    memcpy(fault_rows, s_oled_rows, sizeof(s_oled_rows));
    Car_Stop();
    set_sensors(0u);
    ticks(3u);
    check_stopped(CAR_STOPPED, CAR_FAULT_NONE);
    CHECK(DebugLog_Display() == OLED_OK);
    CHECK(memcmp(fault_rows, s_oled_rows, sizeof(s_oled_rows)) == 0);
    Car_TurnRight90((CarMode)99);
    tick();
    check_stopped(CAR_FAULT, CAR_FAULT_COMMAND);
    CHECK(DebugLog_Display() == OLED_OK);
    CHECK(memcmp(fault_rows, s_oled_rows, sizeof(s_oled_rows)) == 0);
    Car_Track();
    tick();
    CHECK(DebugLog_Display() == OLED_OK);
    CHECK(strstr(s_oled_rows[0], "FAULT") == NULL && log_has("TRACK"));
    CHECK(log_has("TURN_TO"));

    begin_case("real IMU failure marks angle unavailable and captures pre-stop PWM", 0u);
    Car_ImuForward(500u);
    s_sample_yaw = 15.0f;
    tick();
    CHECK(g_mode_fsm.imu_valid == 1u && motorLeft.speed > motorRight.speed);
    snprintf(expected_pwm, sizeof(expected_pwm), "L:%+d R:%+d", motorLeft.speed, motorRight.speed);
    s_sample_ok = 0u;
    tick();
    check_stopped(CAR_FAULT, CAR_FAULT_IMU);
    CHECK(g_mode_fsm.imu_valid == 0u);
    CHECK(DebugLog_Display() == OLED_OK && log_has("IMU_ERR"));
    CHECK(strstr(s_oled_rows[1], "Y:--") != NULL && strstr(s_oled_rows[2], "E:--") != NULL);
    CHECK(strstr(s_oled_rows[4], expected_pwm) != NULL);
    puts("PASS pre-stop fault capture, STOP freeze, invalid request freeze and valid recovery");
}

/**
 * @brief 运行运动与路线的行为场景，任何断言失败都会返回非零退出码。
 * @param 无。
 * @return EXIT_SUCCESS 表示全部场景通过；断言失败由 CHECK 立即退出。
 * @note 通过宿主测试只能说明软件状态与控制计算正确，不能替代小车参数校准。
 */
int main(void)
{
    memcpy(s_original_plan, g_route_plan, sizeof(s_original_plan));
#ifdef TEST_ROUTE_FINISH_ONLY
    /* 收尾专项入口：绕开旧调参断言，只验证新增时序、实际停车和中断边界。 */
    test_route_ninth_step();
    test_route_finish_interrupts();
    puts("All final-route scenarios passed.");
    return EXIT_SUCCESS;
#endif
    test_route_debug_limit();
    if (ROUTE_DEBUG_STEP_LIMIT == 1u) {
        puts("All first-segment debug-limit scenarios passed.");
        return EXIT_SUCCESS;
    }
    test_real_pid_forward_deadband();
    test_default_and_mailbox();
    test_turn_handoffs();
    test_turn_minimum_drive();
    test_turn_filter_and_timeout();
    test_turn_deadline_confirmation();
    test_faults_and_recovery();
    test_real_pid_angle_wrap();
    test_route_conditions();
    test_route_steps();
    test_route_extension();
    test_route_later_steps();
    test_route_ninth_step();
    test_route_finish_interrupts();
    test_route_cross_edges();
    test_route_turn_stop_diagnosis();
    test_route_interrupts();
    test_route_imu_faults();
    test_project_exit_and_timeout();
    test_project_beep();
    test_pwm_and_timed_motion();
    test_timer_entry();
    test_automatic_logs();
    test_fault_log_lifecycle();
    puts("All motion/route integration scenarios passed.");
    return EXIT_SUCCESS;
}

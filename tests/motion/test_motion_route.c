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
 * @brief 模拟打开蜂鸣器并记录发生时刻，用于检查每个项目只触发一次。
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
 * @param automatic_route 1 开启默认示例路线；0 只测试运动动作与条件检测。
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
            ticks(2u);
            CHECK(g_mode_fsm.turn_count == 0u);
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
 * @brief 验证连续动作重新建立基准、完成判据防抖以及 300 帧超时边界。
 * @param 无。
 * @return 无。
 * @note 容差要求严格小于 7 度，命中必须连续三帧；断续命中不能累计。
 */
static void test_turn_filter_and_timeout(void)
{
    begin_case("turn debounce and consecutive reset", 0u);
    Car_TurnLeft90(MODE_IMU);
    tick();
    s_sample_yaw = 90.0f;
    ticks(2u);
    s_sample_yaw = 83.0f;
    tick();
    CHECK(g_mode_fsm.turn_count == 0u);
    s_sample_yaw = 84.0f;
    ticks(2u);
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

    begin_case("exact 300-frame timeout", 0u);
    Car_TurnLeft90(MODE_IMU);
    ticks(299u);
    CHECK(g_mode_fsm.status == CAR_RUNNING && g_mode_fsm.state_frames == 299u);
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
    ticks(297u);
    s_sample_yaw = -90.0f;
    ticks(3u);
    CHECK(g_mode_fsm.status == CAR_RUNNING && g_mode_fsm.turn_count == 1u);
    CHECK(g_mode_fsm.imu_state == IMU_FORWARD && g_mode_fsm.state_frames == 1u);
    puts("PASS continuous completion filter, new baseline and timeout");
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
    CHECK(motorLeft.speed == 600 && motorRight.speed == 400);
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
    ticks(4u);
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
    CHECK(motorLeft.speed == 600 && motorRight.speed == 400);
    IMUTask_SetTarget(-170.0f);
    g_euler.yaw = 170.0f;
    IMUTask_ForwardTick(500u);
    CHECK(motorLeft.speed < motorRight.speed);
    CHECK(motorLeft.speed == 400 && motorRight.speed == 600);
    IMUTask_SetTarget(10000.0f);
    g_euler.yaw = 0.0f;
    CHECK(CLOSE(IMUTask_GetError(), -80.0f));
    IMUTask_TurnTick();
    CHECK(abs(motorLeft.speed) <= 800 && abs(motorRight.speed) <= 800);
    puts("PASS real PID shortest-angle feedback and motor signs");
}

/**
 * @brief 验证所有路口三帧确认、类型抖动、单次锁存与有线/无线条件事件。
 * @param 无。
 * @return 无。
 * @note 关闭示例决策只保留检测，防止路线动作影响防抖边界的观察。
 */
static void test_route_conditions(void)
{
    begin_case("cross three-frame confirmation and debounce", 0u);
    set_sensors(0xFFu);
    ticks(2u);
    CHECK(g_route.junction == JUNCTION_NONE && !g_route.junction_event);
    /* 缺一路在线会改变类型，之前的十字候选帧不能累计。 */
    set_sensors(0xFEu);
    tick();
    set_sensors(0xFFu);
    ticks(2u);
    CHECK(g_route.junction == JUNCTION_NONE && !g_route.junction_event);
    tick();
    CHECK(g_route.junction == JUNCTION_CROSS && g_route.junction_event);
    tick();
    CHECK(!g_route.junction_event);
    set_sensors(0x18u);
    ticks(2u);
    CHECK(g_route.junction == JUNCTION_CROSS);
    set_sensors(0xFFu);
    ticks(3u);
    CHECK(g_route.junction == JUNCTION_CROSS && !g_route.junction_event);
    set_sensors(0x18u);
    ticks(3u);
    CHECK(g_route.junction == JUNCTION_NONE);
    set_sensors(0xFFu);
    ticks(2u);
    CHECK(g_route.junction == JUNCTION_NONE && !g_route.junction_event);
    tick();
    CHECK(g_route.junction == JUNCTION_CROSS && g_route.junction_event);

    begin_case("condition debounce and junction latch", 0u);
    set_sensors(0x0Fu);
    ticks(2u);
    CHECK(g_route.junction == JUNCTION_NONE && !g_route.junction_event);
    set_sensors(0xF0u);
    ticks(2u);
    CHECK(g_route.junction == JUNCTION_NONE && !g_route.junction_event);
    set_sensors(0x0Fu);
    ticks(2u);
    CHECK(!g_route.junction_event);
    tick();
    CHECK(g_route.junction == JUNCTION_LEFT && g_route.junction_event);
    tick();
    CHECK(!g_route.junction_event);
    set_sensors(0xFFu);
    tick();
    CHECK(g_route.junction == JUNCTION_LEFT && !g_route.junction_event);
    tick();
    CHECK(g_route.junction == JUNCTION_LEFT && !g_route.junction_event);
    tick();
    CHECK(g_route.junction == JUNCTION_CROSS && !g_route.junction_event);
    set_sensors(0x18u);
    ticks(2u);
    set_sensors(0xF0u);
    ticks(3u);
    CHECK(g_route.junction == JUNCTION_RIGHT && !g_route.junction_event);
    set_sensors(0x18u);
    ticks(3u);
    CHECK(g_route.junction == JUNCTION_NONE);
    set_sensors(0xF0u);
    ticks(3u);
    CHECK(g_route.junction == JUNCTION_RIGHT && g_route.junction_event);

    begin_case("line-found event in IMU and stopped modes", 0u);
    set_sensors(0u);
    Car_ImuForward(500u);
    ticks(3u);
    CHECK(!g_route.line_found);
    set_sensors(0x01u);
    ticks(2u);
    CHECK(!g_route.line_found && !g_route.line_found_event);
    tick();
    CHECK(g_route.line_found && g_route.line_found_event);
    CHECK(s_track_pid_calls == 0u);
    tick();
    CHECK(!g_route.line_found_event);
    Car_Stop();
    set_sensors(0u);
    ticks(2u);
    CHECK(g_route.line_found && !g_route.line_lost_event);
    tick();
    CHECK(!g_route.line_found && g_route.line_lost_event);
    check_stopped(CAR_STOPPED, CAR_FAULT_NONE);
    Route_Tick(NULL, 0u);
    CHECK(!g_route.junction_event && !g_route.line_found_event && !g_route.line_lost_event);
    puts("PASS junction confirmation, latch and line events across modes");
}

/**
 * @brief 给完整路线注入宿主场景的短时间，真实项目编号、PWM 和转向保持生产表原样。
 * @param 无。
 * @return 无。
 * @note 这些时间只存在测试进程的 RAM，不修改生产源码，不作为实车标定值。
 */
static void calibrate_test_plan(void)
{
    unsigned i;
    for (i = 0u; i < ROUTE_STEP_COUNT; ++i) {
        g_route_plan[i].entry_ms = g_route_plan[i].project_id == 0u ? 0u : 30u;
        g_route_plan[i].pass_ms = g_route_plan[i].project_id == 0u ? 30u : 50u;
        g_route_plan[i].timeout_ms = 500u;
    }
}

/**
 * @brief 为真实路线提供下一帧外设读数，推进一个可重复的完整赛道场景。
 * @return 无。
 * @note 项目内先给全黑，超过最短时间后给出口窄线；转弯只模拟真实 IMU 反馈。
 */
static void drive_route_frame(void)
{
    const RouteStep *step = &g_route_plan[g_route.step_index];
    s_sample_yaw = 0.0f;
    set_sensors(0x18u);
    if (g_route.state == ROUTE_FOLLOW || g_route.state == ROUTE_APPROACH) {
        set_sensors(0xFFu);
    } else if (g_route.state == ROUTE_TURN) {
        s_sample_yaw = step->action == ROUTE_ACTION_LEFT ? 90.0f : -90.0f;
    } else if (g_route.state == ROUTE_OBSTACLE && g_route.state_frames < 6u) {
        set_sensors(0xFFu);
    }
    tick();
}

/**
 * @brief 逐帧运行到指定步骤和阶段，测试失败时有界退出而不是死循环。
 * @param index 目标表项下标。
 * @param state 目标阶段。
 * @return 无。
 */
static void reach_route_state(unsigned index, RouteState state)
{
    unsigned budget = 2000u;
    while ((g_route.step_index != index || g_route.state != state) && budget-- != 0u) {
        drive_route_frame();
    }
    CHECK(budget != 0u);
}

/**
 * @brief 检查全表标定闸门、固定项目顺序、全部转向和最终定时停车。
 * @return 无。
 * @note 不把“源码编译通过”或宿主时间当作赛道的实测距离。
 */
static void test_route_steps(void)
{
    static const uint8_t projects[] = {1u, 2u, 3u, 4u, 5u, 6u, 1u, 7u, 8u, 9u};
    static const uint16_t project_pwm[] = {500u, 600u, 400u, 500u, 500u,
                                          500u, 500u, 600u, 500u, 600u};
    static const RouteAction roads[] = {
        ROUTE_ACTION_STRAIGHT, ROUTE_ACTION_STRAIGHT, ROUTE_ACTION_LEFT, ROUTE_ACTION_RIGHT,
        ROUTE_ACTION_LEFT, ROUTE_ACTION_RIGHT, ROUTE_ACTION_LEFT, ROUTE_ACTION_LEFT,
        ROUTE_ACTION_STRAIGHT, ROUTE_ACTION_RIGHT, ROUTE_ACTION_RIGHT, ROUTE_ACTION_RIGHT,
        ROUTE_ACTION_STRAIGHT, ROUTE_ACTION_RIGHT
    };
    unsigned i, p = 0u, r = 0u, budget = 3000u;
    uint16_t previous_step = 0u;
    begin_case("uncalibrated startup must stay stopped", 0u);
    Car_Stop();
    tick();
    CHECK(!Route_IsCalibrated() && !Route_Start());
    CHECK(g_route.state == ROUTE_UNCALIBRATED && !g_route.enabled);
    ticks(30u);
    check_stopped(CAR_STOPPED, CAR_FAULT_NONE);
    CHECK(DebugLog_Display() == OLED_OK && log_has("WAIT_CAL"));
    calibrate_test_plan();
    CHECK(Route_IsCalibrated());
    g_route_plan[ROUTE_STEP_COUNT - 1u].pass_ms = 0u;
    CHECK(!Route_Start() && g_route.step_index == ROUTE_STEP_COUNT - 1u);
    check_stopped(CAR_STOPPED, CAR_FAULT_NONE);

    begin_case("complete fixed course and timed ramp finish", 1u);
    for (i = 0u; i < ROUTE_STEP_COUNT; ++i) {
        if (g_route_plan[i].project_id != 0u) {
            CHECK(p < sizeof(projects) / sizeof(projects[0]));
            CHECK(g_route_plan[i].project_id == projects[p]);
            CHECK(g_route_plan[i].base_pwm == project_pwm[p]);
            ++p;
        } else {
            CHECK(r < sizeof(roads) / sizeof(roads[0]));
            CHECK(g_route_plan[i].action == roads[r++]);
        }
    }
    CHECK(p == 10u && r == 14u);
    while (g_route.state != ROUTE_FINISHED && budget-- != 0u) {
        RouteState previous_state = g_route.state;
        drive_route_frame();
        CHECK(g_mode_fsm.status != CAR_FAULT);
        CHECK(g_route.step_index == previous_step || g_route.step_index == previous_step + 1u);
        if (g_route.state == ROUTE_OBSTACLE) {
            CHECK(!g_route.junction_event);
            if (g_route.project_id == 3u) {
                CHECK(g_mode_fsm.mode == MODE_TRACK);
            } else {
                CHECK(g_mode_fsm.mode == MODE_IMU);
                CHECK(g_mode_fsm.base_pwm == g_route_plan[g_route.step_index].base_pwm);
                CHECK(motorLeft.speed >= 0 && motorRight.speed >= 0);
            }
        }
        if (g_route.step_index != previous_step) {
            CHECK(previous_state == ROUTE_RECOVER);
            previous_step = g_route.step_index;
        }
    }
    CHECK(budget != 0u && g_route.step_index == ROUTE_STEP_COUNT - 1u);
    CHECK(g_mode_fsm.turn_count == 10u && g_route.project_id == 9u);
    CHECK(s_beep_starts == 10u && s_beep_ends == 10u && !s_beep_on);
    check_stopped(CAR_STOPPED, CAR_FAULT_NONE);
    ticks(20u);
    CHECK(g_route.state == ROUTE_FINISHED);
    check_stopped(CAR_STOPPED, CAR_FAULT_NONE);
    puts("PASS uncalibrated gate, complete 24-step course, U tracking, climbing PWM and ramp stop");
}

/**
 * @brief 验证项目内部不能数路口、早到窄线不能提前完成、出口必须稳定且确实为窄线。
 * @return 无。
 */
static void test_project_exit_and_timeout(void)
{
    static const uint8_t invalid_masks[] = {0u, 0xFFu, 0x01u, 0x28u, 0x3Cu};
    unsigned i, resets;
    uint32_t command;
    begin_case("project entry, minimum passage and valid narrow exit", 1u);
    tick();
    CHECK(g_route.state == ROUTE_ENTRY && g_mode_fsm.mode == MODE_TRACK);
    ticks(2u);
    CHECK(g_route.state == ROUTE_ENTRY);
    tick();
    CHECK(g_route.state == ROUTE_OBSTACLE && g_mode_fsm.state_frames == 1u);
    CHECK(g_mode_fsm.base_pwm == 500u);
    resets = s_resets;
    command = g_mode_fsm.command_id;
    ticks(4u); /* 最短 50 ms 尚未完成，入口前的窄线不能算出口。 */
    CHECK(g_route.state == ROUTE_OBSTACLE);
    for (i = 0u; i < sizeof(invalid_masks) / sizeof(invalid_masks[0]); ++i) {
        set_sensors(invalid_masks[i]);
        ticks(3u);
        CHECK(g_route.state == ROUTE_OBSTACLE && g_route.step_index == 0u);
        CHECK(!g_route.junction_event && motorLeft.speed == 500 && motorRight.speed == 500);
        CHECK(s_resets == resets && g_mode_fsm.command_id == command);
    }
    set_sensors(0x18u);
    ticks(2u);
    CHECK(g_route.state == ROUTE_OBSTACLE);
    set_sensors(0u);
    tick();
    set_sensors(0x18u);
    ticks(3u);
    CHECK(g_route.state == ROUTE_RECOVER && g_route.step_index == 0u);
    ticks(3u);
    CHECK(g_route.step_index == 1u && g_route.state == ROUTE_FOLLOW);
    ticks(10u);
    CHECK(g_route.step_index == 1u);
    set_sensors(0xFFu);
    ticks(3u);
    CHECK(g_route.state == ROUTE_CROSSING);
    ticks(6u);
    CHECK(g_route.state == ROUTE_CROSSING && g_route.step_index == 1u);
    set_sensors(0x18u);
    ticks(5u);
    CHECK(g_route.state == ROUTE_RECOVER); /* 仍未完成恢复确认，不会连跳表项。 */
    tick();
    CHECK(g_route.step_index == 2u && g_route.state == ROUTE_ENTRY);
    puts("PASS entry delay, masked surfaces, minimum duration, stable exit and early cross release");

    begin_case("project timeout freezes stage and stops", 1u);
    g_route_plan[0].timeout_ms = 100u;
    reach_route_state(0u, ROUTE_OBSTACLE);
    set_sensors(0u);
    ticks(9u);
    CHECK(g_mode_fsm.status == CAR_RUNNING);
    tick();
    check_stopped(CAR_FAULT, CAR_FAULT_ROUTE_TIMEOUT);
    CHECK(g_route.step_index == 0u);
    CHECK(DebugLog_Display() == OLED_OK && log_has("ROUTE_TO"));
    CHECK(strstr(s_oled_rows[2], "S:1 P:1 N:10") != NULL);
    CHECK(strstr(s_oled_rows[4], "L:+500 R:+500") != NULL);
    ticks(10u);
    check_stopped(CAR_FAULT, CAR_FAULT_ROUTE_TIMEOUT);

    begin_case("waiting for next junction also times out", 1u);
    reach_route_state(1u, ROUTE_FOLLOW);
    set_sensors(0x18u);
    ticks(50u);
    check_stopped(CAR_FAULT, CAR_FAULT_ROUTE_TIMEOUT);
    CHECK(g_route.step_index == 1u);

    begin_case("line recovery timeout does not advance", 1u);
    reach_route_state(0u, ROUTE_RECOVER);
    set_sensors(0xFFu);
    ticks(50u);
    check_stopped(CAR_FAULT, CAR_FAULT_ROUTE_TIMEOUT);
    CHECK(g_route.step_index == 0u);
    puts("PASS project, road and recovery timeout protection");

    begin_case("valid third exit frame wins at timeout boundary", 1u);
    g_route_plan[0].timeout_ms = 70u;
    reach_route_state(0u, ROUTE_OBSTACLE);
    set_sensors(0x18u);
    ticks(6u);
    CHECK(g_route.state == ROUTE_OBSTACLE && g_mode_fsm.status == CAR_RUNNING);
    tick();
    CHECK(g_route.state == ROUTE_RECOVER && g_mode_fsm.status == CAR_RUNNING);
}

/**
 * @brief 在各阶段主动停车，确认不会推进下标，也不能仅靠重新允许路线自动续跑。
 * @return 无。
 */
static void test_route_interrupts(void)
{
    static const RouteState phases[] = {ROUTE_ENTRY, ROUTE_OBSTACLE, ROUTE_RECOVER,
                                       ROUTE_FOLLOW, ROUTE_CROSSING, ROUTE_APPROACH, ROUTE_TURN};
    unsigned i;
    for (i = 0u; i < sizeof(phases) / sizeof(phases[0]); ++i) {
        uint16_t index;
        unsigned budget = 2000u;
        begin_case("STOP interrupts each navigation phase", 1u);
        while (g_route.state != phases[i] && budget-- != 0u) drive_route_frame();
        CHECK(budget != 0u);
        index = g_route.step_index;
        Car_Stop();
        tick();
        CHECK(g_route.state == ROUTE_ABORTED && g_route.step_index == index);
        check_stopped(CAR_STOPPED, CAR_FAULT_NONE);
        Route_Enable(1u);
        set_sensors(0xFFu);
        ticks(10u);
        CHECK(g_route.state == ROUTE_ABORTED && g_route.step_index == index);
        check_stopped(CAR_STOPPED, CAR_FAULT_NONE);
    }
    begin_case("external track takes ownership without advancing", 1u);
    reach_route_state(0u, ROUTE_OBSTACLE);
    Car_Track();
    tick();
    CHECK(g_route.state == ROUTE_ABORTED && g_route.step_index == 0u);
    CHECK(g_mode_fsm.mode == MODE_TRACK && g_mode_fsm.status == CAR_RUNNING);
    CHECK(!Route_Start()); /* 不能在车还运行时从起点重启。 */

    begin_case("disable keeps current IMU action but cannot resume route", 1u);
    reach_route_state(0u, ROUTE_OBSTACLE);
    Route_Enable(0u);
    tick();
    CHECK(g_route.state == ROUTE_ABORTED && g_mode_fsm.mode == MODE_IMU);
    Car_Stop();
    tick();
    CHECK(Route_Start() && g_route.step_index == 0u);
    tick();
    CHECK(g_route.state == ROUTE_ENTRY);
    puts("PASS interruptions, route disable, explicit ownership and restart from origin");
}

/**
 * @brief 验证项目入口单次短鸣、100 ms 时长、U 型隧道提示和中止后的关闭。
 * @param 无。
 * @return 无。
 * @note 沿用真实路线调度，短鸣不能依赖某一阶段的计时器一直不被复位。
 */
static void test_project_beep(void)
{
    unsigned starts;
    begin_case("one 100 ms pulse only when entering a project", 1u);
    tick();
    ticks(2u);
    CHECK(g_route.state == ROUTE_ENTRY && s_beep_starts == 0u);
    tick();
    CHECK(g_route.state == ROUTE_OBSTACLE && s_beep_on && s_beep_starts == 1u);
    set_sensors(0u);
    ticks(9u);
    CHECK(s_beep_on && s_beep_starts == 1u && s_beep_ends == 0u);
    tick();
    CHECK(!s_beep_on && s_beep_ends == 1u && s_beep_last_frames == 10u);
    ticks(10u);
    CHECK(g_route.state == ROUTE_OBSTACLE && s_beep_starts == 1u);

    begin_case("pulse timing survives project recovery and ignores road junctions", 1u);
    reach_route_state(0u, ROUTE_OBSTACLE);
    set_sensors(0x18u);
    ticks(7u);
    CHECK(g_route.state == ROUTE_RECOVER && s_beep_on);
    ticks(2u);
    CHECK(s_beep_on);
    tick();
    CHECK(g_route.step_index == 1u && !s_beep_on && s_beep_last_frames == 10u);
    set_sensors(0xFFu);
    ticks(3u);
    CHECK(g_route.state == ROUTE_CROSSING && s_beep_starts == 1u);

    begin_case("U tunnel tracking also gives one entry pulse", 1u);
    reach_route_state(6u, ROUTE_ENTRY);
    starts = s_beep_starts;
    CHECK(starts == 2u);
    reach_route_state(6u, ROUTE_OBSTACLE);
    CHECK(g_mode_fsm.mode == MODE_TRACK && s_beep_on && s_beep_starts == starts + 1u);
    set_sensors(0xFFu);
    ticks(10u);
    CHECK(!s_beep_on && s_beep_starts == starts + 1u && s_beep_last_frames == 10u);

    begin_case("STOP cancels an active pulse without retriggering", 1u);
    reach_route_state(0u, ROUTE_OBSTACLE);
    Car_Stop();
    tick();
    CHECK(!s_beep_on && s_beep_starts == 1u && s_beep_ends == 1u);
    ticks(20u);
    CHECK(s_beep_starts == 1u);

    begin_case("route disable cancels the pulse while motion is unchanged", 1u);
    reach_route_state(0u, ROUTE_OBSTACLE);
    Route_Enable(0u);
    tick();
    CHECK(!s_beep_on && g_mode_fsm.status == CAR_RUNNING);
    CHECK(g_mode_fsm.mode == MODE_IMU && g_route.state == ROUTE_ABORTED);

    begin_case("IMU fault closes project pulse by the next control frame", 1u);
    reach_route_state(0u, ROUTE_OBSTACLE);
    s_sample_ok = 0u;
    tick();
    check_stopped(CAR_FAULT, CAR_FAULT_IMU);
    tick();
    CHECK(!s_beep_on && s_beep_starts == 1u && s_beep_ends == 1u);

    begin_case("project timeout closes the pulse in the fault frame", 1u);
    g_route_plan[0].timeout_ms = 70u;
    reach_route_state(0u, ROUTE_OBSTACLE);
    set_sensors(0u);
    ticks(7u);
    check_stopped(CAR_FAULT, CAR_FAULT_ROUTE_TIMEOUT);
    CHECK(!s_beep_on && s_beep_ends == 1u);

    begin_case("uncalibrated startup never emits a project pulse", 0u);
    Car_Stop();
    tick();
    CHECK(!Route_Start());
    ticks(20u);
    CHECK(!s_beep_on && s_beep_starts == 0u);
    puts("PASS project entry beep, exact 100 ms, U tracking, recovery, cancellation and faults");
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
    CHECK(motorLeft.speed >= 500 && motorLeft.speed <= 700);
    CHECK(motorRight.speed >= 500 && motorRight.speed <= 700);
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
 * @note 停车后实时电机已为零，屏幕仍必须保留 299 帧时的两轮转弯输出。
 */
static void test_fault_log_lifecycle(void)
{
    char fault_rows[sizeof(s_oled_rows)];
    begin_case("real timeout snapshot, stop, invalid command and accepted recovery", 0u);
    Car_TurnLeft90(MODE_TRACK);
    ticks(300u);
    check_stopped(CAR_FAULT, CAR_FAULT_TURN_TIMEOUT);
    CHECK(DebugLog_Display() == OLED_OK);
    CHECK(log_has("TURN_TO"));
    CHECK(strstr(s_oled_rows[0], "FAULT L90") != NULL);
    CHECK(strstr(s_oled_rows[2], "N:300") != NULL);
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
    CHECK(g_mode_fsm.imu_valid == 1u && motorLeft.speed == 600 && motorRight.speed == 400);
    s_sample_ok = 0u;
    tick();
    check_stopped(CAR_FAULT, CAR_FAULT_IMU);
    CHECK(g_mode_fsm.imu_valid == 0u);
    CHECK(DebugLog_Display() == OLED_OK && log_has("IMU_ERR"));
    CHECK(strstr(s_oled_rows[1], "Y:--") != NULL && strstr(s_oled_rows[2], "E:--") != NULL);
    CHECK(strstr(s_oled_rows[4], "L:+600") != NULL && strstr(s_oled_rows[4], "R:+400") != NULL);
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
    test_real_pid_forward_deadband();
    test_default_and_mailbox();
    test_turn_handoffs();
    test_turn_filter_and_timeout();
    test_faults_and_recovery();
    test_real_pid_angle_wrap();
    test_route_conditions();
    test_route_steps();
    test_route_interrupts();
    test_project_exit_and_timeout();
    test_project_beep();
    test_pwm_and_timed_motion();
    test_timer_entry();
    test_automatic_logs();
    test_fault_log_lifecycle();
    puts("All motion/route integration scenarios passed.");
    return EXIT_SUCCESS;
}

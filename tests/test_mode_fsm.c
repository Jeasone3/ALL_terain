#include "Mode_FSM.h"
#include "Track_Task.h"
#include "Int_Track.h"
#include "Int_MPU6050.h"
#include "Attitude.h"
#include "IMU_Task.h"
#include "motor.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* 这些替身记录真实状态机发出的命令，不实现状态转换和路口判据。 */
uintptr_t test_tim4_instance;
uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];
volatile uint8_t g_imu_ready;
Euler_struct g_euler;
line_following_t g_line_controller;
Motor_Struct motorLeft = {0};
Motor_Struct motorRight = {1};

static uint32_t s_tick_ms;
static uint32_t s_primask;
static uint32_t s_irq_disable_calls;
static uint32_t s_irq_restore_calls;
static uint8_t s_input_mask;
static uint8_t s_fail_imu_once;
static int s_track_calls;
static int s_pid_clear_calls;
static int s_forward_calls;
static int s_turn_calls;
static int s_stop_calls;
static int s_target_calls;
static int16_t s_left_command;
static int16_t s_right_command;
static float s_last_target;
static float s_last_forward_speed;

uint32_t HAL_GetTick(void)
{
    return s_tick_ms;
}

uint32_t __get_PRIMASK(void)
{
    return s_primask;
}

void __disable_irq(void)
{
    s_irq_disable_calls++;
    s_primask = 1u;
}

void __enable_irq(void)
{
    s_primask = 0u;
}

void __set_PRIMASK(uint32_t value)
{
    s_irq_restore_calls++;
    s_primask = value;
}

void Read_All_Track(uint16_t *sensor_values)
{
    unsigned int i;
    for (i = 0u; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        uint16_t is_on_line = (uint16_t)((s_input_mask >> i) & 1u);
        sensor_values[i] = is_on_line ? LINE_RAW_VALUE : (1u - LINE_RAW_VALUE);
    }
}

void Motor_SetSpeed(Motor_Struct *motor, int16_t speed)
{
    if (motor == &motorLeft) s_left_command = speed;
    if (motor == &motorRight) s_right_command = speed;
}

void PID_clear(pid_type_def *pid)
{
    if (pid == &g_line_controller.pid) s_pid_clear_calls++;
    memset(pid, 0, sizeof(*pid));
}

void follow_line(line_following_t *controller, uint16_t *sensor_values,
                 uint16_t line_raw_value)
{
    (void)controller;
    (void)sensor_values;
    (void)line_raw_value;
    s_track_calls++;
    Motor_SetSpeed(&motorLeft, 400);
    Motor_SetSpeed(&motorRight, 400);
}

void Int_MPU6050_Tick(void)
{
    if (s_fail_imu_once) {
        s_fail_imu_once = 0u;
        g_imu_ready = 0u;
    }
}

void Attitude_Reset(void)
{
    g_euler.yaw = 0.0f;
    g_euler.pitch = 0.0f;
    g_euler.roll = 0.0f;
}

void Attitude_Tick(void)
{
    /* 姿态由每个样例输入，避免传感器替身替状态机决定转弯是否完成。 */
}

void IMUTask_SetTarget(float target)
{
    s_target_calls++;
    s_last_target = target;
}

void IMUTask_TurnTick(void)
{
    s_turn_calls++;
    if (s_last_target >= 0.0f) {
        Motor_SetSpeed(&motorLeft, -200);
        Motor_SetSpeed(&motorRight, 200);
    } else {
        Motor_SetSpeed(&motorLeft, 200);
        Motor_SetSpeed(&motorRight, -200);
    }
}

void IMUTask_ForwardTickWithSpeed(float base_speed)
{
    s_forward_calls++;
    s_last_forward_speed = base_speed;
    Motor_SetSpeed(&motorLeft, (int16_t)base_speed);
    Motor_SetSpeed(&motorRight, (int16_t)base_speed);
}

void IMUTask_ForwardTick(void)
{
    IMUTask_ForwardTickWithSpeed(400.0f);
}

void Temp_Turn_Forward(void)
{
    IMUTask_ForwardTickWithSpeed(400.0f);
}

void IMUTask_Stop(void)
{
    s_stop_calls++;
    Motor_SetSpeed(&motorLeft, 0);
    Motor_SetSpeed(&motorRight, 0);
}

#define CHECK(condition) do { \
    if (!(condition)) { \
        printf("FAIL %s:%d: %s\n", __func__, __LINE__, #condition); \
        return 0; \
    } \
} while (0)

static int close_float(float actual, float expected)
{
    return fabsf(actual - expected) < 0.001f;
}

static ModeFSM_t snapshot(void)
{
    ModeFSM_t value;
    memset(&value, 0, sizeof(value));
    ModeFSM_GetSnapshot(&value);
    return value;
}

static void reset_fixture(void)
{
    s_tick_ms = 0u;
    s_primask = 0u;
    s_input_mask = 0x18u;
    s_fail_imu_once = 0u;
    g_imu_ready = 1u;
    memset(g_sensor_data, 0, sizeof(g_sensor_data));
    memset(&g_euler, 0, sizeof(g_euler));
    memset(&g_line_controller, 0, sizeof(g_line_controller));
    g_line_controller.base_speed = 400;
    g_line_controller.max_speed = 1000;
    ModeFSM_Init();
    s_irq_disable_calls = 0u;
    s_irq_restore_calls = 0u;
    s_track_calls = 0;
    s_pid_clear_calls = 0;
    s_forward_calls = 0;
    s_turn_calls = 0;
    s_stop_calls = 0;
    s_target_calls = 0;
    s_left_command = 0;
    s_right_command = 0;
    s_last_target = 0.0f;
    s_last_forward_speed = 0.0f;
    g_line_controller.pid.error[0] = 17.0f;
}

static void step(uint8_t mask, float yaw)
{
    s_input_mask = mask;
    g_euler.yaw = yaw;
    s_tick_ms += MODEFSM_TICK_MS;
    ModeFSM_Tick();
}

static void steps(unsigned int count, uint8_t mask, float yaw)
{
    unsigned int i;
    for (i = 0u; i < count; i++) step(mask, yaw);
}

static int confirm_cross_without_full_mask(void)
{
    /* 两侧先后压到横线，全过程都没有八路全亮。 */
    step(0x0fu, 0.0f);
    CHECK(snapshot().state == STATE_JUNCTION_PENDING);
    CHECK(snapshot().event_count == 0u);
    step(0x0fu, 0.0f);
    CHECK(snapshot().state == STATE_JUNCTION_PENDING);
    step(0xf0u, 0.0f);
    step(0xf0u, 0.0f);
    CHECK(snapshot().event_count == 1u);
    CHECK(snapshot().last_event == JUNCTION_CROSS);
    CHECK(snapshot().state == STATE_CROSS);
    CHECK(snapshot().fault == MODE_FAULT_NONE);
    return 1;
}

static int enter_forward_from_cross(void)
{
    CHECK(confirm_cross_without_full_mask());
    step(0xffu, 0.0f);
    CHECK(snapshot().state == STATE_FORWARD);
    CHECK(close_float(snapshot().target_yaw, 0.0f));
    CHECK(close_float(s_last_target, 0.0f));
    return 1;
}

static int enter_turn(uint8_t mask, Run_State expected_state, float target)
{
    unsigned int i;
    for (i = 0u; i < JUNCTION_CONFIRM_FRAMES + JUNCTION_ADVANCE_FRAMES + 2u; i++) {
        step(mask, 0.0f);
        if (snapshot().state == expected_state) break;
        CHECK(snapshot().state != STATE_FAULT_STOP);
    }
    CHECK(snapshot().state == expected_state);
    CHECK(snapshot().event_count == 1u);
    CHECK(close_float(snapshot().target_yaw, target));
    CHECK(close_float(s_last_target, target));
    CHECK(s_target_calls > 0);
    /* 候选和转前推进合计十二次低速前进，不能额外推进四十帧。 */
    CHECK(s_forward_calls == (int)JUNCTION_ADVANCE_FRAMES);
    CHECK(close_float(s_last_forward_speed, JUNCTION_BASE_PWM));
    CHECK(s_pid_clear_calls == 0);
    return 1;
}

static int finish_exit(float target)
{
    unsigned int i;
    CHECK(snapshot().state == STATE_FORWARD);
    steps(JUNCTION_EXIT_MIN_FRAMES, 0xffu, target);
    for (i = 0u; i < JUNCTION_EXIT_LINE_FRAMES - 1u; i++) {
        step(0x18u, target);
        CHECK(snapshot().state == STATE_FORWARD);
    }
    step(0x18u, target);
    CHECK(snapshot().state == STATE_NORMAL_TRACK);
    CHECK(snapshot().fault == MODE_FAULT_NONE);
    return 1;
}

static int test_noise_and_single_frame_scratch(void)
{
    reset_fixture();
    steps(5u, 0x81u, 0.0f);
    steps(5u, 0x07u, 0.0f);
    CHECK(snapshot().state == STATE_NORMAL_TRACK);
    CHECK(snapshot().event_count == 0u);
    step(0x0fu, 0.0f);
    CHECK(snapshot().state == STATE_JUNCTION_PENDING);
    CHECK(close_float(s_last_forward_speed, JUNCTION_BASE_PWM));
    steps(JUNCTION_CANCEL_LINE_FRAMES, 0x18u, 0.0f);
    CHECK(snapshot().state == STATE_NORMAL_TRACK);
    CHECK(snapshot().event_count == 0u);
    CHECK(snapshot().last_event == JUNCTION_NONE);
    CHECK(snapshot().left_votes == 0u);
    CHECK(snapshot().right_votes == 0u);
    CHECK(s_pid_clear_calls == 1);
    return 1;
}

static int test_three_frame_vote_window(void)
{
    reset_fixture();
    step(0x0fu, 0.0f);
    CHECK(snapshot().left_votes == 1u);
    step(0x99u, 0.0f);
    CHECK(snapshot().left_votes == 1u);
    step(0x0fu, 0.0f);
    CHECK(snapshot().left_votes == 2u);
    CHECK(snapshot().event_count == 0u);
    CHECK(snapshot().state == STATE_JUNCTION_PENDING);
    steps(JUNCTION_CANCEL_LINE_FRAMES, 0x18u, 0.0f);
    CHECK(snapshot().state == STATE_NORMAL_TRACK);
    CHECK(snapshot().left_votes == 0u);
    return 1;
}

static int test_intermittent_insufficient_votes(void)
{
    unsigned int i;
    reset_fixture();
    for (i = 0u; i < JUNCTION_CONFIRM_FRAMES; i++) {
        step((i % 3u == 0u) ? 0x0fu : 0x99u, 0.0f);
        CHECK(snapshot().event_count == 0u);
    }
    CHECK(snapshot().state == STATE_FAULT_STOP);
    CHECK(snapshot().fault == MODE_FAULT_AMBIGUOUS);
    CHECK(s_left_command == 0 && s_right_command == 0);
    return 1;
}

static int test_cross_without_full_mask(void)
{
    reset_fixture();
    CHECK(confirm_cross_without_full_mask());
    CHECK(s_pid_clear_calls == 0);
    CHECK(close_float(s_last_forward_speed, JUNCTION_BASE_PWM));
    return 1;
}

static int test_left_turn_target_and_completion(void)
{
    unsigned int i;
    reset_fixture();
    CHECK(enter_turn(0x0fu, STATE_TURN_LEFT, 90.0f));
    CHECK(snapshot().last_event == JUNCTION_LEFT);
    for (i = 0u; i < TURN_DONE_FRAMES - 1u; i++) {
        step(0x18u, 90.0f);
        CHECK(snapshot().state == STATE_TURN_LEFT);
    }
    /* 一帧超出容差，必须重新累计连续到位帧。 */
    step(0x18u, 70.0f);
    CHECK(snapshot().state == STATE_TURN_LEFT);
    for (i = 0u; i < TURN_DONE_FRAMES - 1u; i++) {
        step(0x18u, 90.0f);
        CHECK(snapshot().state == STATE_TURN_LEFT);
    }
    step(0x18u, 90.0f);
    CHECK(snapshot().state == STATE_FORWARD);
    CHECK(close_float(snapshot().target_yaw, 90.0f));
    CHECK(close_float(s_last_target, 90.0f));
    CHECK(s_pid_clear_calls == 0);
    CHECK(finish_exit(90.0f));
    CHECK(s_pid_clear_calls == 1);
    CHECK(close_float(g_line_controller.pid.error[0], 0.0f));
    return 1;
}

static int test_right_turn_target_and_completion(void)
{
    reset_fixture();
    CHECK(enter_turn(0xf0u, STATE_TURN_RIGHT, -90.0f));
    CHECK(snapshot().last_event == JUNCTION_RIGHT);
    steps(TURN_DONE_FRAMES, 0x18u, -90.0f);
    CHECK(snapshot().state == STATE_FORWARD);
    CHECK(close_float(snapshot().target_yaw, -90.0f));
    CHECK(close_float(s_last_target, -90.0f));
    CHECK(s_pid_clear_calls == 0);
    CHECK(finish_exit(-90.0f));
    CHECK(s_pid_clear_calls == 1);
    return 1;
}

static int test_old_left_evidence_cannot_form_cross(void)
{
    unsigned int i;
    reset_fixture();
    steps(2u, 0x0fu, 0.0f);
    /* 保留中心但两侧不构成连续三点，超过配对时限再出现右侧。 */
    for (i = 0u; i < JUNCTION_PAIR_GAP_FRAMES; i++) {
        step(0x99u, 0.0f);
        CHECK(snapshot().last_event != JUNCTION_CROSS);
    }
    step(0xf0u, 0.0f);
    step(0xf0u, 0.0f);
    CHECK(snapshot().last_event == JUNCTION_RIGHT);
    CHECK(snapshot().event_count == 1u);
    CHECK(close_float(snapshot().target_yaw, -90.0f));
    CHECK(snapshot().state == STATE_TURN_RIGHT || snapshot().state == STATE_APPROACH_TURN);
    return 1;
}

static int test_pair_evidence_age_eight_accepts_cross(void)
{
    reset_fixture();
    steps(2u, 0x0fu, 0.0f);
    /* 左侧最后一组有效三帧票会延续到第三帧；右侧在第十一帧有效，间隔恰好八帧。 */
    steps(JUNCTION_PAIR_GAP_FRAMES - 1u, 0x99u, 0.0f);
    step(0xf0u, 0.0f);
    CHECK(snapshot().event_count == 0u);
    step(0xf0u, 0.0f);
    CHECK(snapshot().state == STATE_CROSS);
    CHECK(snapshot().last_event == JUNCTION_CROSS);
    CHECK(snapshot().event_count == 1u);
    CHECK(snapshot().fault == MODE_FAULT_NONE);
    return 1;
}

static int test_exit_requires_continuous_narrow_line(void)
{
    unsigned int i;
    reset_fixture();
    CHECK(enter_forward_from_cross());
    steps(20u, 0xffu, 0.0f);
    CHECK(snapshot().state == STATE_FORWARD);
    CHECK(snapshot().event_count == 1u);
    for (i = 0u; i < 3u; i++) {
        steps(JUNCTION_EXIT_LINE_FRAMES - 1u, 0x18u, 0.0f);
        CHECK(snapshot().state == STATE_FORWARD);
        step(0xffu, 0.0f);
        CHECK(snapshot().state == STATE_FORWARD);
    }
    steps(JUNCTION_EXIT_LINE_FRAMES - 1u, 0x18u, 0.0f);
    step(0x00u, 0.0f);
    CHECK(snapshot().state == STATE_FORWARD);
    CHECK(snapshot().event_count == 1u);
    CHECK(s_pid_clear_calls == 0);
    steps(JUNCTION_EXIT_LINE_FRAMES - 1u, 0x18u, 0.0f);
    CHECK(snapshot().state == STATE_FORWARD);
    step(0x18u, 0.0f);
    CHECK(snapshot().state == STATE_NORMAL_TRACK);
    CHECK(snapshot().event_count == 1u);
    CHECK(s_pid_clear_calls == 1);
    return 1;
}

static int test_two_independent_junctions(void)
{
    reset_fixture();
    CHECK(enter_forward_from_cross());
    CHECK(finish_exit(0.0f));
    CHECK(snapshot().event_count == 1u);
    step(0xffu, 0.0f);
    CHECK(snapshot().state == STATE_JUNCTION_PENDING);
    CHECK(snapshot().event_count == 1u);
    step(0xffu, 0.0f);
    CHECK(snapshot().event_count == 2u);
    CHECK(snapshot().last_event == JUNCTION_CROSS);
    return 1;
}

static int test_candidate_center_loss_and_heading(void)
{
    reset_fixture();
    step(0x0fu, 0.0f);
    steps(JUNCTION_CENTER_GAP_FRAMES - 1u, 0x07u, 0.0f);
    CHECK(snapshot().state == STATE_JUNCTION_PENDING);
    step(0x07u, 0.0f);
    CHECK(snapshot().state == STATE_FAULT_STOP);
    CHECK(snapshot().fault == MODE_FAULT_CENTER_LOST);
    CHECK(s_left_command == 0 && s_right_command == 0);

    reset_fixture();
    step(0x0fu, 0.0f);
    step(0x0fu, JUNCTION_MAX_HEADING_ERROR);
    CHECK(snapshot().state == STATE_JUNCTION_PENDING);
    step(0x0fu, JUNCTION_MAX_HEADING_ERROR + 1.0f);
    CHECK(snapshot().state == STATE_FAULT_STOP);
    CHECK(snapshot().fault == MODE_FAULT_HEADING);
    CHECK(s_left_command == 0 && s_right_command == 0);
    return 1;
}

static int test_exit_timeout_stops_and_latches(void)
{
    unsigned int i;
    reset_fixture();
    CHECK(enter_forward_from_cross());
    for (i = 0u; i < JUNCTION_EXIT_TIMEOUT_FRAMES; i++) {
        step(0xffu, 0.0f);
        if (snapshot().state == STATE_FAULT_STOP) break;
    }
    CHECK(snapshot().state == STATE_FAULT_STOP);
    CHECK(snapshot().fault == MODE_FAULT_EXIT_TIMEOUT);
    CHECK(s_left_command == 0 && s_right_command == 0);
    CHECK(snapshot().event_count == 1u);
    steps(10u, 0x18u, 0.0f);
    CHECK(snapshot().state == STATE_FAULT_STOP);
    CHECK(s_left_command == 0 && s_right_command == 0);
    CHECK(s_pid_clear_calls == 0);
    return 1;
}

static int test_turn_timeout_stops(void)
{
    unsigned int i;
    reset_fixture();
    CHECK(enter_turn(0x0fu, STATE_TURN_LEFT, 90.0f));
    for (i = 0u; i < TURN_TIMEOUT_FRAMES; i++) {
        step(0x00u, 0.0f);
        if (snapshot().state == STATE_FAULT_STOP) break;
    }
    CHECK(snapshot().state == STATE_FAULT_STOP);
    CHECK(snapshot().fault == MODE_FAULT_TURN_TIMEOUT);
    CHECK(s_left_command == 0 && s_right_command == 0);
    CHECK(snapshot().event_count == 1u);
    return 1;
}

static int test_imu_unavailable_at_candidate_entry(void)
{
    reset_fixture();
    g_imu_ready = 0u;
    step(0x0fu, 0.0f);
    CHECK(snapshot().state == STATE_FAULT_STOP);
    CHECK(snapshot().fault == MODE_FAULT_IMU);
    CHECK(s_left_command == 0 && s_right_command == 0);
    CHECK(snapshot().event_count == 0u);
    g_imu_ready = 1u;
    steps(5u, 0x18u, 0.0f);
    CHECK(snapshot().state == STATE_FAULT_STOP);
    CHECK(s_left_command == 0 && s_right_command == 0);
    return 1;
}

static int test_imu_read_failure_stops_current_frame(void)
{
    int forward_calls;
    int turn_calls;
    reset_fixture();
    step(0x0fu, 0.0f);
    forward_calls = s_forward_calls;
    s_fail_imu_once = 1u;
    step(0x0fu, 0.0f);
    CHECK(snapshot().state == STATE_FAULT_STOP);
    CHECK(snapshot().fault == MODE_FAULT_IMU);
    CHECK(s_left_command == 0 && s_right_command == 0);
    CHECK(s_forward_calls == forward_calls);

    reset_fixture();
    CHECK(enter_turn(0xf0u, STATE_TURN_RIGHT, -90.0f));
    turn_calls = s_turn_calls;
    s_fail_imu_once = 1u;
    step(0xf0u, 0.0f);
    CHECK(snapshot().state == STATE_FAULT_STOP);
    CHECK(s_left_command == 0 && s_right_command == 0);
    CHECK(s_turn_calls == turn_calls);

    reset_fixture();
    CHECK(enter_forward_from_cross());
    forward_calls = s_forward_calls;
    s_fail_imu_once = 1u;
    step(0xffu, 0.0f);
    CHECK(snapshot().state == STATE_FAULT_STOP);
    CHECK(s_left_command == 0 && s_right_command == 0);
    CHECK(s_forward_calls == forward_calls);
    return 1;
}

static int test_nonfinite_candidate_yaw_stops_current_frame(void)
{
    unsigned int i;
    const float invalid_yaw[] = {NAN, INFINITY};
    for (i = 0u; i < (unsigned int)(sizeof(invalid_yaw) / sizeof(invalid_yaw[0])); i++) {
        int forward_calls;
        reset_fixture();
        step(0x0fu, 0.0f);
        CHECK(snapshot().state == STATE_JUNCTION_PENDING);
        forward_calls = s_forward_calls;
        step(0x0fu, invalid_yaw[i]);
        CHECK(snapshot().state == STATE_FAULT_STOP);
        CHECK(snapshot().fault == MODE_FAULT_IMU);
        CHECK(s_forward_calls == forward_calls);
        CHECK(s_left_command == 0 && s_right_command == 0);
        CHECK(snapshot().event_count == 0u);
    }
    return 1;
}

static int test_debug_ring_snapshot_and_primask(void)
{
    ModeFSM_t copy;
    ModeFSM_DebugFrame frame;
    uint32_t last_tick = 0u;
    unsigned int frame_count = 0u;
    unsigned int i;
    reset_fixture();
    step(0x18u, 1.2f);
    s_primask = 1u;
    ModeFSM_GetSnapshot(&copy);
    CHECK(s_primask == 1u);
    CHECK(copy.state == STATE_NORMAL_TRACK);
    CHECK(copy.raw_mask == 0x18u);
    CHECK(ModeFSM_PopDebugFrame(&frame) == 1u);
    CHECK(s_primask == 1u);
    CHECK(frame.tick_ms == MODEFSM_TICK_MS);
    CHECK(frame.raw_mask == 0x18u);
    CHECK(frame.state == STATE_NORMAL_TRACK);
    CHECK(frame.event == JUNCTION_NONE);
    CHECK(frame.imu_ready == 1u);
    CHECK(frame.yaw_ddeg >= 11 && frame.yaw_ddeg <= 12);
    CHECK(ModeFSM_PopDebugFrame(&frame) == 0u);
    CHECK(s_primask == 1u);
    s_primask = 0u;
    ModeFSM_GetSnapshot(&copy);
    CHECK(s_primask == 0u);
    CHECK(ModeFSM_PopDebugFrame(&frame) == 0u);
    CHECK(s_primask == 0u);
    CHECK(s_irq_disable_calls == s_irq_restore_calls);

    reset_fixture();
    for (i = 0u; i < 70u; i++) step(0x18u, 0.0f);
    CHECK(snapshot().debug_dropped == 7u);
    while (ModeFSM_PopDebugFrame(&frame)) {
        CHECK(frame.tick_ms > last_tick);
        CHECK(frame.raw_mask == 0x18u);
        CHECK(frame.event_count == 0u);
        last_tick = frame.tick_ms;
        frame_count++;
    }
    CHECK(frame_count == 63u);
    CHECK(last_tick == 63u * MODEFSM_TICK_MS);
    /* 队列满丢新帧后，前台取空仍能重新记录下一帧。 */
    step(0x18u, 0.0f);
    CHECK(ModeFSM_PopDebugFrame(&frame) == 1u);
    CHECK(frame.tick_ms == 71u * MODEFSM_TICK_MS);
    CHECK(ModeFSM_PopDebugFrame(&frame) == 0u);

    reset_fixture();
    step(0x18u, NAN);
    CHECK(ModeFSM_PopDebugFrame(&frame) == 1u);
    CHECK(frame.yaw_ddeg == 0);
    return 1;
}

static int test_debug_event_survives_transient_cross(void)
{
    ModeFSM_DebugFrame frame;
    int cross_logged = 0;
    reset_fixture();
    CHECK(enter_forward_from_cross());
    while (ModeFSM_PopDebugFrame(&frame)) {
        if (frame.state == STATE_CROSS) {
            CHECK(frame.event_count == 1u);
            CHECK(frame.event == JUNCTION_CROSS);
            cross_logged = 1;
        }
    }
    CHECK(cross_logged);
    CHECK(snapshot().last_event == JUNCTION_CROSS);
    return 1;
}

typedef int (*Test_Function)(void);

typedef struct {
    const char *name;
    Test_Function run;
} Test_Case;

int main(void)
{
    unsigned int i;
    unsigned int passed = 0u;
    const Test_Case cases[] = {
        {"noise_and_single_frame_scratch", test_noise_and_single_frame_scratch},
        {"three_frame_vote_window", test_three_frame_vote_window},
        {"intermittent_insufficient_votes", test_intermittent_insufficient_votes},
        {"cross_without_full_mask", test_cross_without_full_mask},
        {"left_turn_target_and_completion", test_left_turn_target_and_completion},
        {"right_turn_target_and_completion", test_right_turn_target_and_completion},
        {"old_left_evidence_cannot_form_cross", test_old_left_evidence_cannot_form_cross},
        {"pair_evidence_age_eight_accepts_cross", test_pair_evidence_age_eight_accepts_cross},
        {"exit_requires_continuous_narrow_line", test_exit_requires_continuous_narrow_line},
        {"two_independent_junctions", test_two_independent_junctions},
        {"candidate_center_loss_and_heading", test_candidate_center_loss_and_heading},
        {"exit_timeout_stops_and_latches", test_exit_timeout_stops_and_latches},
        {"turn_timeout_stops", test_turn_timeout_stops},
        {"imu_unavailable_at_candidate_entry", test_imu_unavailable_at_candidate_entry},
        {"imu_read_failure_stops_current_frame", test_imu_read_failure_stops_current_frame},
        {"nonfinite_candidate_yaw_stops_current_frame", test_nonfinite_candidate_yaw_stops_current_frame},
        {"debug_ring_snapshot_and_primask", test_debug_ring_snapshot_and_primask},
        {"debug_event_survives_transient_cross", test_debug_event_survives_transient_cross}
    };
    const unsigned int count = (unsigned int)(sizeof(cases) / sizeof(cases[0]));
    printf("LINE_RAW_VALUE=%d\n", LINE_RAW_VALUE);
    for (i = 0u; i < count; i++) {
        if (cases[i].run()) {
            printf("PASS %s\n", cases[i].name);
            passed++;
        }
    }
    printf("RESULT %u/%u passed\n", passed, count);
    return passed == count ? 0 : 1;
}

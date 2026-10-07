/**
 * @file Mode_FSM.c
 * @brief 路口特征投票、有限时间关联及驶离确认，TIM4 每 10ms 调度。
 */
#include "Mode_FSM.h"
#include "Track_Task.h"/* follow_line, g_line_controller, LINE_RAW_VALUE */
#include "Int_Track.h"/* Read_All_Track, GRAYSCALE_SENSOR_CHANNELS */
#include "Int_MPU6050.h"/* Int_MPU6050_Tick, g_imu_ready */
#include "Attitude.h"/* Attitude_Reset, Attitude_Tick, g_euler */
#include "IMU_Task.h"/* IMUTask_SetTarget/TurnTick/ForwardTick/Stop */
#include <math.h>

/* 灰度传感器数组(Int/Int_Track.c 定义) */
extern uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];
//状态机对象，用于存储状态机当前状态和状态转移条件
/**
 * @brief volatile 的作用：确保变量在多线程环境下的可见性和同步性。防止编译器对变量进行优化，确保每次访问都从内存中读取最新的值，而不是使用寄存器中的缓存值。
 * 
 */
volatile ModeFSM_t g_mode_fsm;
// 定义路口特征投票窗口、窄线判定、推进帧数和调试缓冲区容量
#define CENTER_MASK       0x18u
#define NARROW_AREA_MASK  0x3cu
#define VOTE_WINDOW_MASK  0x07u
#define DEBUG_CAPACITY    64u

/* 仅保存本次候选的证据；不能把不同路口的信号拼接起来。 */
static uint8_t s_left_history, s_right_history;
static uint8_t s_left_seen, s_right_seen;
static uint32_t s_left_frame, s_right_frame;
static uint8_t s_narrow_frames, s_center_gap, s_turn_done;
static uint32_t s_advance_frames;
static float s_turn_target;
static uint32_t s_tick_ms;

/* 中断只入队，前台取出打印；满时丢新帧，不阻塞控制。 */
static ModeFSM_DebugFrame s_debug[DEBUG_CAPACITY];
static volatile uint8_t s_debug_head, s_debug_tail;

static uint8_t bit_count(uint8_t bits)
{
    uint8_t count = 0u;
    while (bits != 0u) {
        count += bits & 1u;
        bits >>= 1;
    }
    return count;
}
/**
 * @brief 将目标航向与当前航向的误差限制在 [-180, 180] 范围内
 *         航向跨过 ±180° 时，控制器和状态机使用相同的最短角度误差
 * 
 * @param target 
 * @param yaw 
 * @return float 
 */
static float angle_error(float target, float yaw)
{
    float error = target - yaw;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

static uint8_t read_mask(void)
{
    uint8_t mask = 0u;
    Read_All_Track(g_sensor_data);
    for (uint8_t i = 0u; i < GRAYSCALE_SENSOR_CHANNELS; ++i) {
        if (g_sensor_data[i] == LINE_RAW_VALUE)
            mask |= (uint8_t)(1u << i);
    }
    return mask;
}

/* 半边至少三个相邻探头在线，且仍有中间线；孤立外侧黑点不算分支。 */
static uint8_t side_feature(uint8_t half, uint8_t mask)
{
    return (mask & CENTER_MASK) != 0u &&
           (((half & 0x07u) == 0x07u) || ((half & 0x0eu) == 0x0eu));
}

/* 普通线限于中间四路，并要求有效中心点和一段连续窄线。 */
static uint8_t narrow_line(uint8_t mask)
{
    if (mask == 0u || (mask & (uint8_t)~NARROW_AREA_MASK) != 0u ||
        (mask & CENTER_MASK) == 0u || bit_count(mask) > 3u)
        return 0u;
    while ((mask & 1u) == 0u) mask >>= 1;
    return (mask & (uint8_t)(mask + 1u)) == 0u;
}

static void clear_evidence(void)
{
    s_left_history = s_right_history = 0u;
    s_left_seen = s_right_seen = 0u;
    s_left_frame = s_right_frame = 0u;
    s_narrow_frames = s_center_gap = s_turn_done = 0u;
    s_advance_frames = 0u;
    g_mode_fsm.left_votes = g_mode_fsm.right_votes = 0u;
}

static void enter_state(Run_State state, float target)
{
    g_mode_fsm.state = state;
    g_mode_fsm.target_yaw = target;
    g_mode_fsm.state_frames = 0u;
    s_narrow_frames = s_turn_done = 0u;
    if (state == STATE_NORMAL_TRACK) {
        /* 停顿过的误差不是上一控制帧，恢复循迹前重新建立 PID 历史。 */
        PID_clear(&g_line_controller.pid);
        clear_evidence();
        IMUTask_Stop();
    }
}

static void fault_stop(ModeFSM_Fault fault)
{
    g_mode_fsm.fault = fault;
    enter_state(STATE_FAULT_STOP, g_mode_fsm.target_yaw);
    IMUTask_Stop();
}

/* 必须在任何角度控制电机输出之前检查本帧采样有效性。 */
static uint8_t update_imu(void)
{
    Int_MPU6050_Tick();
    if (!g_imu_ready) {
        fault_stop(MODE_FAULT_IMU);
        return 0u;
    }
    Attitude_Tick();
    if (!isfinite(g_euler.yaw) || fabsf(g_euler.yaw) > 360.0f) {
        fault_stop(MODE_FAULT_IMU);
        return 0u;
    }
    return 1u;
}

/* 候选与追加推进使用同一航向和中心线保护，调长推进也不能盲走。 */
static uint8_t check_approach(uint8_t mask)
{
    if (fabsf(angle_error(0.0f, g_euler.yaw)) > JUNCTION_MAX_HEADING_ERROR) {
        fault_stop(MODE_FAULT_HEADING);
        return 0u;
    }
    if ((mask & CENTER_MASK) == 0u) ++s_center_gap;
    else s_center_gap = 0u;
    if (s_center_gap >= JUNCTION_CENTER_GAP_FRAMES) {
        fault_stop(MODE_FAULT_CENTER_LOST);
        return 0u;
    }
    return 1u;
}

static void vote_features(uint8_t mask)
{
    uint8_t left = side_feature(mask & 0x0fu, mask);
    uint8_t right = side_feature(mask >> 4, mask);
    s_left_history = (uint8_t)(((s_left_history << 1) | left) & VOTE_WINDOW_MASK);
    s_right_history = (uint8_t)(((s_right_history << 1) | right) & VOTE_WINDOW_MASK);
    g_mode_fsm.left_votes = bit_count(s_left_history);
    g_mode_fsm.right_votes = bit_count(s_right_history);
    if (g_mode_fsm.left_votes >= 2u) {
        s_left_seen = 1u;
        s_left_frame = g_mode_fsm.state_frames;
    }
    if (g_mode_fsm.right_votes >= 2u) {
        s_right_seen = 1u;
        s_right_frame = g_mode_fsm.state_frames;
    }
}

static uint8_t fresh_evidence(uint8_t seen, uint32_t frame)
{
    return seen && (g_mode_fsm.state_frames - frame) <= JUNCTION_PAIR_GAP_FRAMES;
}

static void begin_turn(void)
{
    /* 基准只在进入候选时建立，选择转向时必须更新 IMU 内部目标。 */
    IMUTask_SetTarget(s_turn_target);
    enter_state(s_turn_target > 0.0f ? STATE_TURN_LEFT : STATE_TURN_RIGHT,
                s_turn_target);
}

static void confirm_junction(Junction_Event event)
{
    g_mode_fsm.last_event = event;
    ++g_mode_fsm.event_count;
    if (event == JUNCTION_CROSS) {
        enter_state(STATE_CROSS, 0.0f);
    } else {
        s_turn_target = event == JUNCTION_LEFT ? 90.0f : -90.0f;
        /* 候选和转前推进同为低速，累计到总帧数，不再追加旧的 400ms。 */
        if (s_advance_frames >= JUNCTION_ADVANCE_FRAMES)
            begin_turn();
        else
            enter_state(STATE_APPROACH_TURN, 0.0f);
    }
}

static void pending_tick(uint8_t mask)
{
    uint8_t left, right;
    if (!update_imu() || !check_approach(mask)) return;

    vote_features(mask);
    if (narrow_line(mask)) ++s_narrow_frames;
    else s_narrow_frames = 0u;
    if (s_narrow_frames >= JUNCTION_CANCEL_LINE_FRAMES) {
        /* 短划痕已经消失，取消本次候选；旧左右证据同时清除。 */
        enter_state(STATE_NORMAL_TRACK, 0.0f);
        return;
    }

    left = fresh_evidence(s_left_seen, s_left_frame);
    right = fresh_evidence(s_right_seen, s_right_frame);
    if (g_mode_fsm.state_frames >= JUNCTION_CONFIRM_FRAMES && !left && !right) {
        fault_stop(MODE_FAULT_AMBIGUOUS);
        return;
    }
    /* 每帧只由一个控制器输出电机命令，路口内不运行普通循迹 PID。 */
    IMUTask_ForwardTickWithSpeed(JUNCTION_BASE_PWM);
    ++s_advance_frames;
    if (left && right) {
        confirm_junction(JUNCTION_CROSS);
    } else if (g_mode_fsm.state_frames >= JUNCTION_CONFIRM_FRAMES) {
        if (left && !right) confirm_junction(JUNCTION_LEFT);
        else if (right && !left) confirm_junction(JUNCTION_RIGHT);
    }
}

static void start_candidate(uint8_t mask)
{
    clear_evidence();
    Attitude_Reset();
    IMUTask_SetTarget(0.0f);
    enter_state(STATE_JUNCTION_PENDING, 0.0f);
    g_mode_fsm.state_frames = 1u;
    pending_tick(mask);
}

static void push_debug(void)
{
    uint8_t next = (uint8_t)((s_debug_head + 1u) % DEBUG_CAPACITY);
    ModeFSM_DebugFrame *frame;
    if (next == s_debug_tail) {
        ++g_mode_fsm.debug_dropped;
        return;
    }
    frame = &s_debug[s_debug_head];
    frame->tick_ms = s_tick_ms;
    frame->event_count = g_mode_fsm.event_count;
    frame->yaw_ddeg = isfinite(g_euler.yaw) && fabsf(g_euler.yaw) < 3276.0f ?
                      (int16_t)(g_euler.yaw * 10.0f) : 0;
    frame->raw_mask = g_mode_fsm.raw_mask;
    frame->state = (uint8_t)g_mode_fsm.state;
    frame->left_votes = g_mode_fsm.left_votes;
    frame->right_votes = g_mode_fsm.right_votes;
    frame->event = (uint8_t)g_mode_fsm.last_event;
    frame->fault = (uint8_t)g_mode_fsm.fault;
    frame->imu_ready = g_imu_ready;
    s_debug_head = next;
}

void ModeFSM_Tick(void)
{
    uint8_t mask = read_mask();
    g_mode_fsm.raw_mask = mask;
    s_tick_ms += MODEFSM_TICK_MS;
    switch (g_mode_fsm.state) {
    case STATE_NORMAL_TRACK:
        if (side_feature(mask & 0x0fu, mask) || side_feature(mask >> 4, mask)) {
            /* 先识别候选，避免宽线图案先被循迹 PID 拉向一侧。 */
            start_candidate(mask);
        } else {
            follow_line(&g_line_controller, g_sensor_data, LINE_RAW_VALUE);
        }
        break;

    case STATE_JUNCTION_PENDING:
        ++g_mode_fsm.state_frames;
        pending_tick(mask);
        break;

    case STATE_CROSS:
        /* 十字默认直行；状态锁定期间不再生成新的路口事件。 */
        if (update_imu()) {
            enter_state(STATE_FORWARD, 0.0f);
            IMUTask_ForwardTickWithSpeed(JUNCTION_BASE_PWM);
        }
        break;

    case STATE_APPROACH_TURN:
        if (!update_imu() || !check_approach(mask)) break;
        ++g_mode_fsm.state_frames;
        IMUTask_ForwardTickWithSpeed(JUNCTION_BASE_PWM);
        ++s_advance_frames;
        if (s_advance_frames >= JUNCTION_ADVANCE_FRAMES) begin_turn();
        break;

    case STATE_TURN_LEFT:
    case STATE_TURN_RIGHT:
        if (!update_imu()) break;
        ++g_mode_fsm.state_frames;
        if (fabsf(angle_error(g_mode_fsm.target_yaw, g_euler.yaw)) < TURN_DONE_TOLERANCE)
            ++s_turn_done;
        else s_turn_done = 0u;
        if (s_turn_done >= TURN_DONE_FRAMES) {
            /* 沿转后的目标航向找线，不能把目标重新设为零度。 */
            enter_state(STATE_FORWARD, g_mode_fsm.target_yaw);
            IMUTask_Stop();
        } else if (g_mode_fsm.state_frames >= TURN_TIMEOUT_FRAMES) {
            fault_stop(MODE_FAULT_TURN_TIMEOUT);
        } else {
            IMUTask_TurnTick();
        }
        break;

    case STATE_FORWARD:
        if (!update_imu()) break;
        ++g_mode_fsm.state_frames;
        if (g_mode_fsm.state_frames >= JUNCTION_EXIT_MIN_FRAMES && narrow_line(mask))
            ++s_narrow_frames;
        else s_narrow_frames = 0u;
        if (s_narrow_frames >= JUNCTION_EXIT_LINE_FRAMES) {
            enter_state(STATE_NORMAL_TRACK, 0.0f);
        } else if (g_mode_fsm.state_frames >= JUNCTION_EXIT_TIMEOUT_FRAMES) {
            fault_stop(MODE_FAULT_EXIT_TIMEOUT);
        } else {
            IMUTask_ForwardTickWithSpeed(JUNCTION_BASE_PWM);
        }
        break;

    case STATE_FAULT_STOP:
        IMUTask_Stop();
        break;

    default:
        fault_stop(MODE_FAULT_INVALID_STATE);
        break;
    }
    push_debug();
}

void ModeFSM_Init(void)
{
    ModeFSM_t initial = {0};
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    g_mode_fsm = initial;
    clear_evidence();
    s_turn_target = 0.0f;
    s_tick_ms = 0u;
    s_debug_head = s_debug_tail = 0u;
    __set_PRIMASK(primask);
}

void ModeFSM_GetSnapshot(ModeFSM_t *snapshot)
{
    uint32_t primask;
    if (snapshot == 0) return;
    primask = __get_PRIMASK();
    __disable_irq();
    *snapshot = g_mode_fsm;
    __set_PRIMASK(primask);
}

uint8_t ModeFSM_PopDebugFrame(ModeFSM_DebugFrame *frame)
{
    uint32_t primask;
    if (frame == 0) return 0u;
    primask = __get_PRIMASK();
    __disable_irq();
    if (s_debug_head == s_debug_tail) {
        __set_PRIMASK(primask);
        return 0u;
    }
    *frame = s_debug[s_debug_tail];
    s_debug_tail = (uint8_t)((s_debug_tail + 1u) % DEBUG_CAPACITY);
    __set_PRIMASK(primask);
    return 1u;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
        ModeFSM_Tick();
    
}

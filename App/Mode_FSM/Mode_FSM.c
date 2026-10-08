/**
 * @file Mode_FSM.c
 * @brief 路口特征投票、有限时间关联及驶离确认，TIM4 每 10ms 调度。
 *
 * 阅读顺序：ModeFSM_Tick -> start_candidate -> pending_tick -> confirm_junction。
 * NORMAL_TRACK：普通循迹，发现单侧路口特征后进入候选。
 * JUNCTION_PENDING：低速保持原航向，积累左右证据，不立即决定转弯。
 * CROSS / TURN_*：执行已确认的动作；十字默认直行，单侧分支转 ±90°。
 * FORWARD：沿选定航向驶离并重新找到稳定窄线，随后恢复普通循迹。
 * FAULT_STOP：锁存停车；通信恢复不会自动重新启动旧动作。
 *
 * “一帧”就是一次 10ms 控制调用。这里按帧推进动作，没有等待动作完成的阻塞循环。
 * 路口滤波针对“左右区域是否有分支”的布尔特征，原始灰度数组仍保留本帧数据。
 * 日志只在中断内入队，前台取出后打印；日志与控制判断相互独立。
 */
#include "Mode_FSM.h"
#include "Track_Task.h"/* 普通循迹控制器，以及传感器在线时的原始电平约定 */
#include "Int_Track.h"/* 八路灰度 GPIO 采样接口 */
#include "Int_MPU6050.h"/* 本帧 IMU 数据读取与通信状态 */
#include "Attitude.h"/* 姿态基准重置、姿态解算和当前欧拉角 */
#include "IMU_Task.h"/* 保持航向、定角转弯及停车接口 */
#include "motor.h"/* 只读取最终 PWM 指令，诊断不写电机输出 */
#include <math.h>

/* 数组由 Int_Track.c 定义；这里只声明并使用，不重复分配存储空间。
 * 下标 0..7 对应 IN1..IN8，每次 read_mask 都先刷新这份原始 GPIO 数据。 */
extern uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];
/* 中断维护的公开状态记录，前台通过 ModeFSM_GetSnapshot 读取。
 * volatile 要求编译器保留必要的读写，不把共享状态当作不会变化的数据。
 * 它不提供互斥，也不保证整个结构体被一次性、一致地读取；一致性由短临界区保证。 */
volatile ModeFSM_t g_mode_fsm;
/* 在线位图最低位对应 IN1。常规二进制打印从高位到低位，显示顺序是 IN8..IN1。
 * CENTER_MASK = 00011000：选择 IN4、IN5，任意一路在线即认为中间仍有线。 */
#define CENTER_MASK       0x18u
/* NARROW_AREA_MASK = 00111100：普通窄线只允许落在 IN3..IN6 范围内。 */
#define NARROW_AREA_MASK  0x3cu
/* VOTE_WINDOW_MASK = 00000111：保留最近三帧的特征，不是选择三个灰度探头。 */
#define VOTE_WINDOW_MASK  0x07u
/* 环形日志队列留一个空位区分“满”和“空”，64 个位置实际可保存 63 帧。 */
#define DEBUG_CAPACITY    64u

/* 下列 static 变量仅本文件可访问，其值跨控制调用保留。
 * 新候选开始或恢复循迹时清除证据，防止把不同路口的左右信号拼接起来。 */
/* 最近三帧的左右特征历史：最低位是最新帧，1 表示该帧存在对应侧分支特征。 */
static uint8_t s_left_history, s_right_history;
/* 有效标志：本次候选中是否曾获得至少两票；不是“当前这一路是否在线”。 */
static uint8_t s_left_seen, s_right_seen;
/* 最近一次票数达到两票时的候选帧号；与 state_frames 相减可计算证据年龄。 */
static uint32_t s_left_frame, s_right_frame;
/* 分别统计连续窄线、连续中心丢线、连续角度到位；条件中断后相应计数清零。 */
static uint8_t s_narrow_frames, s_center_gap, s_turn_done;
/* 候选与转前推进累计的前进帧数；切换到 APPROACH_TURN 时继续累计。 */
static uint32_t s_advance_frames;
/* 已选择的转弯目标：左转 +90°、右转 -90°，相对于进入候选时的航向基准。 */
static float s_turn_target;
/* 每次 ModeFSM_Tick 增加 MODEFSM_TICK_MS；这是合成节拍时间，不是独立墙钟。 */
static uint32_t s_tick_ms;
/* 观测数据不参与控制决策。电机字段记录的是 PWM 指令，不是编码器速度。 */
extern Motor_Struct motorLeft, motorRight;
static volatile ModeFSM_Diagnostics s_diagnostics;
static uint8_t s_yaw_valid, s_yaw_fresh;
static uint32_t s_yaw_tick_ms;
static uint32_t s_previous_cycle;
static uint8_t s_have_previous_cycle;

/* 中断生产日志、前台消费日志。队列满时丢弃新帧，不能等待前台来腾空间。 */
static ModeFSM_DebugFrame s_debug[DEBUG_CAPACITY];
/* head 指向下一个写入位置，tail 指向最旧的待读记录；下标到末尾后循环回零。 */
static volatile uint8_t s_debug_head, s_debug_tail;

/**
 * @brief 统计一个字节中有几个二进制 1，用于统计在线探头或有效票数。
 * @param bits 待统计的位图，例如 00011000 有两个 1。
 * @return 1 的个数，范围 0..8。
 */
static uint8_t bit_count(uint8_t bits)
{
    uint8_t count = 0u;
    while (bits != 0u) {
        count += bits & 1u; /* 取最低位：结果只有 0 或 1，加入累计数量。 */
        bits >>= 1;        /* 已统计的最低位移出，让下一位成为最低位。 */
    }
    return count;
}
/**
 * @brief 将角度误差折算到 [-180°, 180°]，避免跨越角度边界时判断成转一大圈。
 * @param target 目标航向，单位为度。
 * @param yaw 当前测得的航向，单位为度；调用前已经检查其数值有效性。
 * @return 最短的带符号角度误差，完成判断时用 fabsf 取绝对值。
 * 例如 target=90、yaw=-179：直接相减为 269°，减去 360° 后为 -91°。
 * IMU_Task 中的实际角度控制使用相同的误差约定。
 */
static float angle_error(float target, float yaw)
{
    float error = target - yaw;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

/**
 * @brief 刷新八路原始数据，再生成“在线为 1”的归一化位图。
 * @return bit0..bit7 对应 IN1..IN8。例如 0x18 是 IN4/IN5 在线，0x0F 是左四路在线。
 * LINE_RAW_VALUE 定义黑线上实际读到的 GPIO 电平；该函数不假定黑线必然是高电平。
 */
static uint8_t read_mask(void)
{
    uint8_t mask = 0u;
    Read_All_Track(g_sensor_data);
    for (uint8_t i = 0u; i < GRAYSCALE_SENSOR_CHANNELS; ++i) {
        if (g_sensor_data[i] == LINE_RAW_VALUE)
            mask |= (uint8_t)(1u << i); /* 将第 i 位设为 1，其余已经设置的位保持不变。 */
    }
    return mask;
}

/**
 * @brief 判断一帧是否有单侧分支的空间特征；返回 1 仅表示候选特征，不直接转弯。
 * @param half 对应半边的四位数据；左侧取 mask & 0x0F，右侧取 mask >> 4。
 * @param mask 完整八路位图，用于额外检查 IN4/IN5 是否至少一路在线。
 * 三个相邻点成立的组合为 0111、1110、1111；零散的 0101 不符合。
 * 这能过滤孤立黑点，但持续出现、形状接近真实分支的划痕仍可能符合条件。
 */
static uint8_t side_feature(uint8_t half, uint8_t mask)
{
    return (mask & CENTER_MASK) != 0u &&
           (((half & 0x07u) == 0x07u) || ((half & 0x0eu) == 0x0eu));
}

/**
 * @brief 判断是否恢复普通窄线，供“取消候选”和“确认驶离”使用。
 * 必须同时满足：非全零、在线点仅在 IN3..IN6、IN4/IN5 有线、最多三个点且连续。
 * 0x18、0x1C 符合；0x00 是丢线、0x3C 过宽、0x28 点不连续，均不符合。
 * 普通循迹本身仍使用 follow_line，这个判据不是替代其全部线位置计算。
 */
static uint8_t narrow_line(uint8_t mask)
{
    /* ~NARROW_AREA_MASK 选择中间四路之外的位，命中这些位就不能算恢复窄线。 */
    if (mask == 0u || (mask & (uint8_t)~NARROW_AREA_MASK) != 0u ||
        (mask & CENTER_MASK) == 0u || bit_count(mask) > 3u)
        return 0u;
    while ((mask & 1u) == 0u) mask >>= 1; /* 去掉右侧的零，使最低位从第一个在线点开始。 */
    /* 连续 1：0011 + 1 = 0100，两者按位与为零。
     * 间断 1：0101 + 1 = 0110，两者按位与不为零，因此排除分散的亮点。 */
    return (mask & (uint8_t)(mask + 1u)) == 0u;
}

/* 清除本次识别过程的历史和计数。
 * event_count、last_event、fault 不在这里清除，保留累计事件与故障诊断记录。 */
static void clear_evidence(void)
{
    s_left_history = s_right_history = 0u;
    s_left_seen = s_right_seen = 0u;
    s_left_frame = s_right_frame = 0u;
    s_narrow_frames = s_center_gap = s_turn_done = 0u;
    s_advance_frames = 0u;
    g_mode_fsm.left_votes = g_mode_fsm.right_votes = 0u;
}

/**
 * @brief 更新状态记录并重新开始该状态的局部计时。
 * @param state 接下来要运行的状态；switch 通常要到下一帧才执行这个新分支。
 * @param target 状态机记录的目标航向，不会在本函数中自动写入 IMU 控制器。
 * IMU 控制器的内部目标要显式调用 IMUTask_SetTarget 设置。
 * s_advance_frames 不随普通状态切换清零，以便候选和追加推进合并计算。
 */
static void enter_state(Run_State state, float target)
{
    g_mode_fsm.state = state;
    g_mode_fsm.target_yaw = target;
    g_mode_fsm.state_frames = 0u;
    s_narrow_frames = s_turn_done = 0u;
    if (state == STATE_NORMAL_TRACK) {
        /* 恢复循迹：清除暂停前的积分/微分历史和路口证据。
         * 当前帧先停车，下一帧 NORMAL_TRACK 才重新调用循迹控制器。 */
        PID_clear(&g_line_controller.pid);
        clear_evidence();
        IMUTask_Stop();
    }
}

/* 记录具体故障并立即停车。FAULT_STOP 每帧继续发停车命令，不自动恢复旧动作。 */
static void fault_stop(ModeFSM_Fault fault)
{
    g_mode_fsm.fault = fault;
    enter_state(STATE_FAULT_STOP, g_mode_fsm.target_yaw);
    IMUTask_Stop();
}

/**
 * @brief 读取本帧 IMU 并解算姿态，在角度控制输出之前确认数据可用。
 * @return 成功为 1；失败为 0，并且已经进入故障停车。
 * 通信失败时不能继续使用冻结的旧 yaw 驱动电机。
 * isfinite 排除非数值和无穷大；额外角度范围检查排除明显异常的姿态输出。
 */
static uint8_t update_imu(void)
{
    Int_MPU6050_Tick();
    if (!g_imu_ready) {
        s_yaw_valid = 0u;
        fault_stop(MODE_FAULT_IMU);
        return 0u;
    }
    Attitude_Tick();
    if (!isfinite(g_euler.yaw) || fabsf(g_euler.yaw) > 360.0f) {
        s_yaw_valid = 0u;
        fault_stop(MODE_FAULT_IMU);
        return 0u;
    }
    s_yaw_valid = s_yaw_fresh = 1u;
    s_yaw_tick_ms = s_tick_ms;
    return 1u;
}

/**
 * @brief 对候选和转前推进检查航向、中心线；调用前先通过 update_imu。
 * 两个阶段都以进入候选时的 0° 相对航向前进，偏差超过上限就停车。
 * IN4/IN5 同时离线时累计 s_center_gap；任意一路恢复就归零。
 * 该检查用于候选和追加推进，不代表普通循迹阶段也采用同样的丢线保护。
 * @return 正常为 1；故障为 0，并且已经停车。
 */
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

/**
 * @brief 对左右区域的布尔特征进行三帧投票，仅在候选阶段更新。
 * 每一票代表“一帧符合单侧分支条件”，不是“一只探头在线”。
 * 历史左移一位后把本帧放进最低位，再用 0x07 丢弃三帧之前的内容。
 * 例如左特征依次为 1、1、0、0：历史为 001、011、110、100，票数为 1、2、2、1。
 * 至少两票时记录有效标志和帧号；以后票数下降，仍可在有限时间内使用这份证据。
 */
static void vote_features(uint8_t mask)
{
    uint8_t left = side_feature(mask & 0x0fu, mask);
    uint8_t right = side_feature(mask >> 4, mask);
    s_left_history = (uint8_t)(((s_left_history << 1) | left) & VOTE_WINDOW_MASK);
    s_right_history = (uint8_t)(((s_right_history << 1) | right) & VOTE_WINDOW_MASK);
    g_mode_fsm.left_votes = bit_count(s_left_history);
    g_mode_fsm.right_votes = bit_count(s_right_history);
    /* 两票未必相邻，历史 101 也有两票。当前帧无左特征但历史为 110 时，
     * 仍满足两票并刷新有效帧号，因此关联年龄从“最近一次两票成立”开始计算。 */
    if (g_mode_fsm.left_votes >= 2u) {
        s_left_seen = 1u;
        s_left_frame = g_mode_fsm.state_frames;
    }
    if (g_mode_fsm.right_votes >= 2u) {
        s_right_seen = 1u;
        s_right_frame = g_mode_fsm.state_frames;
    }
}

/**
 * @brief 检查本次候选中曾成立的单侧证据是否尚未过期。
 * @param seen 有效标志；初值 0 不能把初始帧号 0 误当成已获得证据。
 * @param frame 最近一次获得至少两票时的候选帧号。
 * @return 有效且年龄不超过关联窗口时返回 1；默认年龄 8 帧可用、9 帧过期。
 * 该函数只在候选阶段使用；新候选会清除旧标志和帧号，避免跨路口关联。
 */
static uint8_t fresh_evidence(uint8_t seen, uint32_t frame)
{
    return seen && (g_mode_fsm.state_frames - frame) <= JUNCTION_PAIR_GAP_FRAMES;
}

/* 设置 IMU 控制器内部的实际转弯目标，再选择左转或右转状态。
 * 本次路口的姿态基准已经在候选入口建立，不在这里再次归零。 */
static void begin_turn(void)
{
    /* 仅修改 g_mode_fsm.target_yaw 不会改变 IMU_Task 的内部目标，必须调用此接口。 */
    IMUTask_SetTarget(s_turn_target);
    enter_state(s_turn_target > 0.0f ? STATE_TURN_LEFT : STATE_TURN_RIGHT,
                s_turn_target);
}

/**
 * @brief 确认一次路口事件，记录结果并进入动作阶段。
 * @param event 已确认的十字、左侧分支或右侧分支。
 * event_count 是确认次数，不是成功通过次数；动作后发生故障时计数也会保留。
 * 进入动作后不再调用 pending_tick，因而同一路口不会在动作阶段重复确认。
 */
static void confirm_junction(Junction_Event event)
{
    g_mode_fsm.last_event = event;
    ++g_mode_fsm.event_count;
    if (event == JUNCTION_CROSS) {
        enter_state(STATE_CROSS, 0.0f);
    } else {
        s_turn_target = event == JUNCTION_LEFT ? 90.0f : -90.0f;
        /* 三目运算选择转弯目标：左侧取 +90°，右侧取 -90°。
         * 候选和转前推进同为低速，累计到总帧数，不再追加旧的 400ms。
         * 默认确认和总推进均为 12 帧，所以单侧确认后通常直接开始转弯；
         * 以后调大总推进帧数时，才进入 APPROACH_TURN 补足剩余推进。 */
        if (s_advance_frames >= JUNCTION_ADVANCE_FRAMES)
            begin_turn();
        else
            enter_state(STATE_APPROACH_TURN, 0.0f);
    }
}

/**
 * @brief 候选阶段每帧的完整处理：保护检查、投票、取消、低速前进、分类。
 * 左右都有效可提前确认十字；单侧必须等确认窗口结束，给另一侧补齐证据的机会。
 * 连续恢复窄线则取消候选，清除短划痕留下的证据；短真实分支也可能被取消，
 * 这是当前抗短干扰的识别取舍，帧数需要依据实车记录调整。
 * 本函数会改变全局状态，但不会在内部持续循环等待确认完成。
 */
static void pending_tick(uint8_t mask)
{
    uint8_t left, right;
    /* || 有短路特性：IMU 检查失败时不再检查航向，直接返回；检查函数已经停车。 */
    if (!update_imu() || !check_approach(mask)) return;

    vote_features(mask);
    /* 必须连续看到窄线；夹入一帧宽线或丢线，就要重新计数。 */
    if (narrow_line(mask)) ++s_narrow_frames;
    else s_narrow_frames = 0u;
    if (s_narrow_frames >= JUNCTION_CANCEL_LINE_FRAMES) {
        /* 取消优先于后面的分类：视为短特征已经过去，不再用其旧证据作转向决定。 */
        enter_state(STATE_NORMAL_TRACK, 0.0f);
        return;
    }

    left = fresh_evidence(s_left_seen, s_left_frame);
    right = fresh_evidence(s_right_seen, s_right_frame);
    /* 确认期限已到且没有新鲜证据：不猜方向，也不先输出一次前进命令。 */
    if (g_mode_fsm.state_frames >= JUNCTION_CONFIRM_FRAMES && !left && !right) {
        fault_stop(MODE_FAULT_AMBIGUOUS);
        return;
    }
    /* 持续低速前进才能让另一侧探头经过横线；此阶段保持原航向，不运行循迹 PID。
     * s_advance_frames 统计已经输出的前进帧数，后面的转前推进接着用它。 */
    IMUTask_ForwardTickWithSpeed(JUNCTION_BASE_PWM);
    ++s_advance_frames;
    /* 十字优先：只要左右证据都还新鲜，就不要求同一帧出现八路全亮。
     * 单侧证据则等完整窗口后分类；窗口内持续存在的分支会不断刷新证据时间。 */
    if (left && right) {
        confirm_junction(JUNCTION_CROSS);
    } else if (g_mode_fsm.state_frames >= JUNCTION_CONFIRM_FRAMES) {
        if (left && !right) confirm_junction(JUNCTION_LEFT);
        else if (right && !left) confirm_junction(JUNCTION_RIGHT);
    }
}

/**
 * @brief 首次发现单侧空间特征时建立候选，而不是立即决定转弯。
 * 姿态只在这里重置一次，建立进入路口时的相对 0° 基准；不等于校准陀螺零偏。
 * 首帧也参与候选处理，state_frames 从 1 开始；保护检查正常时同帧投票并低速前进。
 * 即使只有一帧划痕，也可能短暂进入候选；只有后续票数足够才确认动作。
 */
static void start_candidate(uint8_t mask)
{
    clear_evidence();
    Attitude_Reset();
    s_yaw_valid = 0u;
    IMUTask_SetTarget(0.0f);
    enter_state(STATE_JUNCTION_PENDING, 0.0f);
    g_mode_fsm.state_frames = 1u;
    pending_tick(mask);
}

/**
 * @brief 中断生产一条诊断记录，不执行串口打印或等待队列腾空。
 * head == tail 表示空，head 的下一个位置等于 tail 表示满；留一个位置区分两者。
 * 满时保留原有日志并丢弃本条新记录，debug_dropped 加一，控制照常继续。
 * 正常循迹和故障停车不更新姿态，记录中的 yaw 可能是之前留下的值。
 */
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
    /* 角度乘 10 后保存为整数，避免前台需要输出浮点数。
     * 非数值或超出 int16_t 的安全转换范围时记为 0，不能据此断言实际航向为零。 */
    frame->yaw_ddeg = isfinite(g_euler.yaw) && fabsf(g_euler.yaw) < 3276.0f ?
                      (int16_t)(g_euler.yaw * 10.0f) : 0;
    frame->raw_mask = g_mode_fsm.raw_mask;
    frame->state = (uint8_t)g_mode_fsm.state;
    frame->left_votes = g_mode_fsm.left_votes;
    frame->right_votes = g_mode_fsm.right_votes;
    frame->event = (uint8_t)g_mode_fsm.last_event;
    frame->fault = (uint8_t)g_mode_fsm.fault;
    frame->imu_ready = g_imu_ready;
    s_debug_head = next; /* 记录填写完后再发布新的写入位置，前台即可取出本条。 */
}

/**
 * @brief TIM4 每 10ms 调用一次，按当前状态执行一步控制并记录本帧数据。
 * 所有状态都采样灰度，便于驶离确认和故障诊断；普通循迹不额外解算 IMU。
 * switch 选中一个分支后，enter_state 改变 state 不会让 switch 自动重选分支。
 * NORMAL 的候选入口会显式调用 pending_tick，使首次候选也能立即开始处理。
 */
void ModeFSM_Tick(void)
{
    s_yaw_fresh = 0u;
    uint8_t mask = read_mask(); /* 当前帧原始数据生成的在线位图，并非历史滤波位图。 */
    g_mode_fsm.raw_mask = mask;
    s_tick_ms += MODEFSM_TICK_MS; /* 按调用次数累计，不能用于测量中断实际耗时或漏拍。 */
    switch (g_mode_fsm.state) {
    case STATE_NORMAL_TRACK:
        /* 仅此状态允许发现新路口；动作和驶离阶段都不重新触发，这是事件锁定机制。 */
        if (side_feature(mask & 0x0fu, mask) || side_feature(mask >> 4, mask)) {
            /* 先识别候选，避免宽线图案先被循迹 PID 拉向一侧。 */
            start_candidate(mask);
        } else {
            follow_line(&g_line_controller, g_sensor_data, LINE_RAW_VALUE);
        }
        break;

    case STATE_JUNCTION_PENDING:
        /* 后续候选帧先加一；首次候选帧已由 start_candidate 设置成 1。 */
        ++g_mode_fsm.state_frames;
        pending_tick(mask);
        break;

    case STATE_CROSS:
        /* 已确认十字的短暂过渡状态，当前固定选直行。
         * 检查 IMU 后进入驶离阶段，本帧就保持 0° 前进；该分支没有推进帧数加一。 */
        if (update_imu()) {
            enter_state(STATE_FORWARD, 0.0f);
            IMUTask_ForwardTickWithSpeed(JUNCTION_BASE_PWM);
        }
        break;

    case STATE_APPROACH_TURN:
        /* 只补足候选阶段尚未完成的推进，同样检查中心线和航向。
         * 当前帧完成前进输出后，begin_turn 设置目标，下一帧才执行转弯分支。 */
        if (!update_imu() || !check_approach(mask)) break;
        ++g_mode_fsm.state_frames;
        IMUTask_ForwardTickWithSpeed(JUNCTION_BASE_PWM);
        ++s_advance_frames;
        if (s_advance_frames >= JUNCTION_ADVANCE_FRAMES) begin_turn();
        break;

    case STATE_TURN_LEFT:
    case STATE_TURN_RIGHT:
        /* 左右转共用结束判断，实际方向由已经设置的 ±90° 目标决定。
         * 误差必须连续小于容差；一帧超出就清零，避免刚掠过目标角便结束。 */
        if (!update_imu()) break;
        ++g_mode_fsm.state_frames;
        if (fabsf(angle_error(g_mode_fsm.target_yaw, g_euler.yaw)) < TURN_DONE_TOLERANCE)
            ++s_turn_done;
        else s_turn_done = 0u;
        if (s_turn_done >= TURN_DONE_FRAMES) {
            /* 保留转后的航向目标去找线，当前帧先停车，下一帧再低速驶离。
             * 此处若把目标改为 0°，会使小车试图返回进入路口时的原方向。 */
            enter_state(STATE_FORWARD, g_mode_fsm.target_yaw);
            IMUTask_Stop();
        } else if (g_mode_fsm.state_frames >= TURN_TIMEOUT_FRAMES) {
            fault_stop(MODE_FAULT_TURN_TIMEOUT);
        } else {
            IMUTask_TurnTick();
        }
        break;

    case STATE_FORWARD:
        /* 驶离和重新找线：仍用角度环，不直接把宽线交给普通循迹 PID。
         * 默认第 5 帧开始累计窄线，连满 4 帧，最早在本分支第 8 次执行时恢复循迹。
         * 宽线或全零都会打断计数；达到 1 秒上限仍未恢复则停车，而非强行解锁。 */
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
        /* 锁存停车：即使主循环已经恢复 IMU 通信，也不继续执行原动作。 */
        IMUTask_Stop();
        break;

    default:
        /* 非法枚举值也必须转入明确停车状态，不能沿用上一帧的电机命令。 */
        fault_stop(MODE_FAULT_INVALID_STATE);
        break;
    }
    push_debug(); /* 包括辅助函数切入故障的帧，最终状态仍会进入日志。 */
    /* 在控制完成后发布诊断，前台不会拼接不同帧的计数与电机输出。 */
    s_diagnostics.fsm = g_mode_fsm;
    s_diagnostics.tick_ms = s_tick_ms;
    s_diagnostics.yaw_tick_ms = s_yaw_tick_ms;
    s_diagnostics.yaw_valid = s_yaw_valid;
    s_diagnostics.yaw_fresh = s_yaw_fresh;
    s_diagnostics.yaw_ddeg = s_yaw_valid ? (int16_t)(g_euler.yaw * 10.0f) : 0;
    s_diagnostics.imu_ready = g_imu_ready;
    s_diagnostics.pwm_left = motorLeft.speed;
    s_diagnostics.pwm_right = motorRight.speed;
    s_diagnostics.advance_frames = s_advance_frames;
    s_diagnostics.center_gap = s_center_gap;
    s_diagnostics.narrow_frames = s_narrow_frames;
    s_diagnostics.turn_done = s_turn_done;
    s_diagnostics.left_age = s_diagnostics.right_age = 255u;
    s_diagnostics.left_fresh = s_diagnostics.right_fresh = 0u;
    if (g_mode_fsm.state == STATE_JUNCTION_PENDING) {
        uint32_t left_age = g_mode_fsm.state_frames - s_left_frame;
        uint32_t right_age = g_mode_fsm.state_frames - s_right_frame;
        if (s_left_seen) {
            s_diagnostics.left_age = (uint8_t)(left_age > 254u ? 254u : left_age);
            s_diagnostics.left_fresh = left_age <= JUNCTION_PAIR_GAP_FRAMES;
        }
        if (s_right_seen) {
            s_diagnostics.right_age = (uint8_t)(right_age > 254u ? 254u : right_age);
            s_diagnostics.right_fresh = right_age <= JUNCTION_PAIR_GAP_FRAMES;
        }
    }
}

/**
 * @brief 上电初始化，主程序在启动 TIM4 控制中断之前调用。
 * 结构体清零后 state=0，恰好对应 NORMAL_TRACK，事件和故障也回到 NONE。
 * 日志 head/tail 归零即可视为队列为空，无须逐条清除旧数组内容。
 * 该函数会丢弃识别过程的记忆，不应在主循环中每轮重复调用。
 */
void ModeFSM_Init(void)
{
    ModeFSM_t initial = {.state = STATE_NORMAL_TRACK};
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    g_mode_fsm = initial;
    clear_evidence();
    s_turn_target = 0.0f;
    s_tick_ms = 0u;
    s_debug_head = s_debug_tail = 0u;
    {
        ModeFSM_Diagnostics empty = {.fsm = {.state = STATE_NORMAL_TRACK}};
        s_diagnostics = empty;
    }
    s_yaw_valid = s_yaw_fresh = s_have_previous_cycle = 0u;
    s_yaw_tick_ms = s_previous_cycle = 0u;
    __set_PRIMASK(primask);
}

/**
 * @brief 前台取得一致的状态快照，供 OLED 等较慢操作使用。
 * @param snapshot 调用者提供的结构体地址；空指针时不操作。
 * 只在复制期间短暂屏蔽中断；显示/打印应在返回后使用副本，不在临界区内完成。
 * 保存并恢复原 PRIMASK，避免误把调用前已经屏蔽的中断重新打开。
 */
void ModeFSM_GetSnapshot(ModeFSM_t *snapshot)
{
    uint32_t primask;
    if (snapshot == 0) return;
    primask = __get_PRIMASK();
    __disable_irq();
    *snapshot = g_mode_fsm;
    __set_PRIMASK(primask);
}

/* 与控制状态一样，仅复制期间屏蔽中断；DWT 换算和字符串处理均在前台。 */
void ModeFSM_GetDiagnostics(ModeFSM_Diagnostics *snapshot)
{
    uint32_t primask;
    if (snapshot == 0) return;
    primask = __get_PRIMASK();
    __disable_irq();
    *snapshot = s_diagnostics;
    snapshot->imu_ready = g_imu_ready;
    __set_PRIMASK(primask);
}

/**
 * @brief 前台从日志队列取出最旧的一帧。
 * @param frame 用于接收记录的有效地址。
 * @return 取到记录返回 1；队列为空或地址为空返回 0。
 * 复制和推进 tail 在短临界区完成；取出后才交给串口打印，不阻塞中断生产者。
 */
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

/* HAL 定时器中断回调：仅 TIM4 更新事件驱动本状态机，其他定时器不触发控制。
 * 10ms 周期由 tim.c 的实际定时器配置决定，不是 MODEFSM_TICK_MS 宏单独决定的。 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) {
        uint32_t begin = DWT->CYCCNT;
        uint32_t period = (SystemCoreClock / 1000u) * MODEFSM_TICK_MS;
        if (s_have_previous_cycle) {
            uint32_t interval = begin - s_previous_cycle;
            if (interval > s_diagnostics.interval_max_cycles)
                s_diagnostics.interval_max_cycles = interval;
            /* 允许 10% 的到达抖动；这是到达间隔异常数，不等于精确漏拍数。 */
            if (interval > period + period / 10u) ++s_diagnostics.late_intervals;
        }
        s_previous_cycle = begin;
        s_have_previous_cycle = 1u;
        ModeFSM_Tick();
        s_diagnostics.control_last_cycles = DWT->CYCCNT - begin;
        if (s_diagnostics.control_last_cycles > s_diagnostics.control_max_cycles)
            s_diagnostics.control_max_cycles = s_diagnostics.control_last_cycles;
        if (s_diagnostics.control_last_cycles >= period) ++s_diagnostics.control_overruns;
    }
    
}

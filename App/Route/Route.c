/**
 * @file Route.c
 * @brief 固定路线：道路路口与项目分别执行，完成后才推进，不在项目内数路口。
 * @note 无编码器，所有入口和通行时间必须实车标定；默认参数禁止自动启动。
 */
#include "Route.h"
#include "Mode_FSM.h"
#include "Debug_Log.h"
#include "Track_Task.h"
#include "Int_Beep.h"

#define ROUTE_CONFIRM_FRAMES 3u
#define ROUTE_FRAME_MS       10u
#define ROUTE_PROJECT_BEEP_FRAMES 10u /* 每次进入项目短鸣 100 ms，不阻塞控制节拍 */

/* 顺序与箭头图一致；背后连接弯角也列出，避免把它们与岔路口混计。
 * 所有时间先填 0，逐段记录后分别填写，不把统一 5 秒当作真实距离。
 * 列顺序：动作、项目编号、PWM、入口延时、最短通过/前移、阶段超时。
 */
RouteStep g_route_plan[ROUTE_STEP_COUNT] = {
    {ROUTE_ACTION_IMU_PASS,   1u, 600u, 4000u, 5000u, 10000u}, /* 01 起点 -> 石块 */
    {ROUTE_ACTION_STRAIGHT,   0u, 500u, 0u, 3000u, 5000u}, /* 02 石块后十字直行 */
    {ROUTE_ACTION_IMU_PASS,   2u, 700u, 3000u, 10000u, 13000u}, /* 03 大台阶 */
    {ROUTE_ACTION_STRAIGHT,   0u, 500u, 0u, 3000u, 5000u}, /* 04 高台后方十字直行，避开方形隧道 */
    {ROUTE_ACTION_LEFT,       0u, 500u, 0u, 50u, 5000u}, /* 05 后方连接直角左转 */
    {ROUTE_ACTION_RIGHT,      0u, 500u, 0u, 50u, 5000u}, /* 06 转入 U 型隧道入口支路 */
    {ROUTE_ACTION_TRACK_PASS, 3u, 400u, 2000u, 10000u, 15000u}, /* 07 U 型隧道全程循迹 */
    {ROUTE_ACTION_LEFT,       0u, 500u, 0u, 50u, 5000u}, /* 08 U 出口连接直角左转 */
    {ROUTE_ACTION_RIGHT,      0u, 500u, 0u, 50u, 5000u}, /* 09 转向栅格 */
    {ROUTE_ACTION_IMU_PASS,   4u, 600u, 2000u, 6000u, 10000u}, /* 10 栅格 */
    {ROUTE_ACTION_LEFT,       0u, 500u, 0u, 50u, 5000u}, /* 11 栅格后进入左侧外沿 */
    {ROUTE_ACTION_LEFT,       0u, 500u, 0u, 50u, 5000u}, /* 12 左外沿转入减速带支路 */
    {ROUTE_ACTION_IMU_PASS,   5u, 600u, 3000u, 5000u, 10000u}, /* 13 减速带 */
    {ROUTE_ACTION_STRAIGHT,   0u, 500u, 0u, 3000u, 5000u}, /* 14 中间十字直行 */
    {ROUTE_ACTION_IMU_PASS,   6u, 600u, 3000u, 6000u, 10000u}, /* 15 草地 */
    {ROUTE_ACTION_RIGHT,      0u, 500u, 0u, 50u, 5000u}, /* 16 草地后右转去石块，禁止进入方形隧道 */
    {ROUTE_ACTION_IMU_PASS,   1u, 600u, 3000u, 5000u, 10000u}, /* 17 第二次石块 */
    {ROUTE_ACTION_RIGHT,      0u, 500u, 0u, 50u, 5000u}, /* 18 石块后右转进入外沿 */
    {ROUTE_ACTION_RIGHT,      0u, 500u, 0u, 50u, 5000u}, /* 19 外沿右转去小台阶 */
    {ROUTE_ACTION_IMU_PASS,   7u, 700u, 3000u, 5000u, 10000u}, /* 20 小台阶 */
    {ROUTE_ACTION_STRAIGHT,   0u, 500u, 0u, 50u, 5000u}, /* 21 中间十字直行 */
    {ROUTE_ACTION_IMU_PASS,   8u, 700u, 3000u, 6000u, 10000u}, /* 22 防滑带 */
    {ROUTE_ACTION_RIGHT,      0u, 1000u, 2000u, 50u, 10000u}, /* 23 转向项目 9 斜坡 */
    {ROUTE_ACTION_TIMED_STOP, 9u, 600u, 1000u, 5000u, 10000u}  /* 24 整车上蓝色高台后定时停车 */
};

volatile Route_t g_route;

static RouteJunction s_junction_candidate; /* 三帧候选路口类型 */
static uint8_t s_junction_frames;          /* 候选类型连续帧数 */
static uint8_t s_junction_latched;         /* 当前路口已经报告，离开前不重复 */
static uint8_t s_line_candidate;           /* 任意通道在线的候选值 */
static uint8_t s_line_frames;              /* 有线/无线连续帧数 */
static uint8_t s_narrow_frames;            /* 当前阶段出口窄线连续帧数 */
static uint8_t s_active;                   /* 真正启动的路线，不等同于 enabled */
static uint32_t s_turn_count_before;       /* 本次转弯开始前的完成总数 */
static uint8_t s_project_beep_frames;      /* 项目提示音剩余帧数，独立于路线阶段计时 */

/**
 * @brief 每个控制帧维护项目提示音，到时或取消时关闭蜂鸣器。
 * @param cancel 非零立即结束当前项目提示；零消耗一个已完成的 10 ms 帧。
 * @return 无。
 * @note 只管理 Route 自己启动的短鸣，不影响控制定时器启动前的上电校准蜂鸣。
 *       必须在路线提前返回前调用，保证离开项目或中止路线后不会留下持续鸣叫。
 */
static void update_project_beep(uint8_t cancel)
{
    if (s_project_beep_frames == 0u) return;
    if (cancel) {
        s_project_beep_frames = 0u;
    } else {
        --s_project_beep_frames;
    }
    if (s_project_beep_frames == 0u) Int_Beep_Off();
}

/**
 * @brief 清除路口候选与锁存，项目读数不能成为下一道路步骤的待处理事件。
 * @param 无。
 * @return 无。
 * @note 不清除有线/无线监测，也不改路线下标或电机。
 */
static void reset_junction(void)
{
    s_junction_candidate = JUNCTION_NONE;
    s_junction_frames = 0u;
    s_junction_latched = 0u;
    g_route.junction = JUNCTION_NONE;
    g_route.junction_event = 0u;
}

/**
 * @brief 从有效八路数组分类传感器图案，同时返回是否有任意通道在线。
 * @param sensor_data 本帧采样，IN1 至 IN8。
 * @param any_line 返回 0/1，不代表这就是有效出口窄黑线。
 * @return 未防抖的路口图案，十字优先。
 */
static RouteJunction classify_sensor_frame(const uint16_t *sensor_data, uint8_t *any_line)
{
    uint8_t left_on = 1u, right_on = 1u, i;
    *any_line = 0u;
    for (i = 0u; i < 8u; ++i) {
        uint8_t on_line = (uint8_t)(sensor_data[i] == LINE_RAW_VALUE);
        *any_line |= on_line;
        if (i < 4u) {
            left_on &= on_line;
        } else {
            right_on &= on_line;
        }
    }
    if (left_on && right_on) return JUNCTION_CROSS;
    if (left_on) return JUNCTION_LEFT;
    if (right_on) return JUNCTION_RIGHT;
    return JUNCTION_NONE;
}

/**
 * @brief 路口三帧确认，离开三帧后才解除锁存。
 * @param raw_junction 本帧图案。
 * @return 无。
 * @note 左右与十字之间变化不会解锁；只能在道路识别阶段消费事件。
 */
static void update_junction(RouteJunction raw_junction)
{
    if (raw_junction != s_junction_candidate) { //进入候选
        s_junction_candidate = raw_junction;
        s_junction_frames = 1u;
    } else if (s_junction_frames < ROUTE_CONFIRM_FRAMES) { //候选连续帧数
        ++s_junction_frames;
    }

    if (s_junction_frames < ROUTE_CONFIRM_FRAMES) //未连续三帧确认
        return;
    
    g_route.junction = s_junction_candidate; //候选成功
   
    if (s_junction_candidate == JUNCTION_NONE) {
        s_junction_latched = 0u;
    } else if (!s_junction_latched) {
        s_junction_latched = 1u;
        g_route.junction_event = 1u;
    }
}

/**
 * @brief 三帧确认任意有线/无线变化，供日志观察，不直接决定项目完成。
 * @param raw_line 任意通道是否在线。
 * @return 无。
 */
static void update_line(uint8_t raw_line)
{
    if (raw_line != s_line_candidate) {
        s_line_candidate = raw_line;
        s_line_frames = 1u;
    } else if (s_line_frames < ROUTE_CONFIRM_FRAMES) {
        ++s_line_frames;
    }
    if (s_line_frames >= ROUTE_CONFIRM_FRAMES && g_route.line_found != s_line_candidate) {
        g_route.line_found = s_line_candidate;
        if (s_line_candidate) g_route.line_found_event = 1u;
        else g_route.line_lost_event = 1u;
    }
}

/**
 * @brief 识别居中的单段窄黑线，拒绝全零、全黑、外侧线和不连续反光点。
 * @param sensor_data 本帧八路采样。
 * @return 1：中间 IN3 至 IN6 内连续 1 至 3 路在线，且包含 IN4 或 IN5；否则 0。
 * @note 此出口判据依赖当前八路排布，线宽必须在标定时验证。
 */
static uint8_t narrow_line(const uint16_t *sensor_data)
{
    uint8_t i, count = 0u, center = 0u, seen = 0u, gap = 0u;

    for (i = 0u; i < 8u; ++i) {
        if (sensor_data[i] == LINE_RAW_VALUE) {
            if (i < 2u || i > 5u || gap) 
                return 0u;
            ++count;
            seen = 1u;
            if (i == 3u || i == 4u) 
            {
                center = 1u;
            }
        } else if (seen) {
            gap = 1u;
        }
    }
    return (uint8_t)(center && count <= 3u);
}

/**
 * @brief 为当前出口阶段累计窄黑线连续帧数。
 * @param sensor_data 本帧采样。
 * @return 连续三帧有效时为 1，任一无效帧立即清零。
 */
static uint8_t confirm_exit(const uint16_t *sensor_data)
{
    if (narrow_line(sensor_data)) {
        if (s_narrow_frames < ROUTE_CONFIRM_FRAMES) ++s_narrow_frames;
    } else {
        s_narrow_frames = 0u;
    }
    return (uint8_t)(s_narrow_frames >= ROUTE_CONFIRM_FRAMES);
}

/**
 * @brief 更新监测；项目入口、通行和恢复阶段屏蔽路口图案。
 * @param sensor_data 有效八路数组或空指针。
 * @return 无。
 * @note 事件只持续一帧，日志在控制入口即时保存，前台不轮询单帧事件。
 */
static void update_conditions(const uint16_t *sensor_data)
{
    uint8_t raw_line;
    RouteJunction raw_junction;

    g_route.junction_event = 0u; // 新路口事件
    g_route.line_found_event = 0u; // 新有线事件
    g_route.line_lost_event = 0u;  // 新无线事件

    if (sensor_data == 0) 
        return;

    raw_junction = classify_sensor_frame(sensor_data, &raw_line); //识别路口
    if (g_route.state == ROUTE_ENTRY || g_route.state == ROUTE_OBSTACLE ||
        g_route.state == ROUTE_RECOVER || g_route.state == ROUTE_READY) {
        reset_junction();
    } else {
        update_junction(raw_junction);
    }
    update_line(raw_line);
    if (g_route.junction_event) {
        DebugLog_Write(LOG_INFO, g_route.junction == JUNCTION_LEFT ? "JUNC_L" :
                                 g_route.junction == JUNCTION_RIGHT ? "JUNC_R" : "CROSS");
    }
    if (g_route.line_found_event) DebugLog_Write(LOG_INFO, "LINE_ON");
    if (g_route.line_lost_event) {
        DebugLog_Write(g_mode_fsm.mode == MODE_TRACK && g_mode_fsm.status == CAR_RUNNING ?
                       LOG_WARN : LOG_INFO, "LINE_OFF");
    }
}

/**
 * @brief 返回第一条无效参数的位置，全部有效时返回表长。
 * @return 0 至 ROUTE_STEP_COUNT，0 时间始终表示未标定。
 * @note 道路允许入口等待为 0；所有前移/通行和阶段超时都必须明确填写。
 */
static uint16_t invalid_step(void)
{
    uint16_t i;
    /* 最后一项必须负责停车，否则成功推进会越过固定表尾。 */
    if (g_route_plan[ROUTE_STEP_COUNT - 1u].action != ROUTE_ACTION_TIMED_STOP ||
        g_route_plan[ROUTE_STEP_COUNT - 1u].project_id != 9u) return ROUTE_STEP_COUNT - 1u;
    for (i = 0u; i < ROUTE_STEP_COUNT; ++i) {
        const RouteStep *step = &g_route_plan[i];
        if (step->action > ROUTE_ACTION_TIMED_STOP ||
            (step->action == ROUTE_ACTION_TIMED_STOP && i != ROUTE_STEP_COUNT - 1u) ||
            step->pass_ms == 0u || step->timeout_ms <= step->pass_ms ||
            step->timeout_ms <= step->entry_ms ||
            step->base_pwm == 0u || step->base_pwm > 1000u ||
            (step->action >= ROUTE_ACTION_IMU_PASS && step->entry_ms == 0u) ||
            (step->action == ROUTE_ACTION_TRACK_PASS && step->base_pwm != 400u)) {
            return i;
        }
    }
    return ROUTE_STEP_COUNT;
}

/**
 * @brief 改变执行阶段，计时和出口防抖从新阶段重新开始。
 * @param state 新阶段。
 * @return 无；不发运动请求，不改变路线表下标。
 */
static void enter_state(RouteState state)
{
    g_route.state = state;
    g_route.state_frames = 0u;
    s_narrow_frames = 0u;
}

/**
 * @brief 进入当前表项，从本次状态切换开始统计已完成控制帧。
 * @param 无。
 * @return 无。
 * @note 每次表项只调用一次；不会在 while(1) 重复重置 PID 或航向。
 */
static void begin_step(void)
{
    const RouteStep *step = &g_route_plan[g_route.step_index];//从0开始
    g_route.project_id = step->project_id;  //项目编号
    reset_junction(); //清空上一个状态
    enter_state(step->action >= ROUTE_ACTION_IMU_PASS ? ROUTE_ENTRY : ROUTE_FOLLOW); //如果是IMU直行或U型隧道循迹，进入入口阶段，否则进入循迹等待阶段
    if (g_mode_fsm.status != CAR_RUNNING || g_mode_fsm.mode != MODE_TRACK) //如果当前不是循迹模式，则切换到循迹模式
    {
        Car_Track();
    }
    DebugLog_Write(LOG_INFO, "STEP_BEGIN");
}

/**
 * @brief 成功恢复并离开当前区域后只推进一次，随即进入下一表项。
 * @param 无。
 * @return 无；失败和外部取消都不能调用本函数。
 */
static void finish_step(void)
{
    ++g_route.step_index;
    begin_step(); /* 最后一项只走定时停车分支，不会从这里越过表尾。 */
}

/**
 * @brief 项目或道路动作完成后恢复循迹，再确认三帧窄黑线。
 * @param 无。
 * @return 无。
 * @note 恢复期间仍屏蔽路口，禁止同一交叉区域重复推进下标。
 */
static void begin_recovery(void)
{
    enter_state(ROUTE_RECOVER);
    reset_junction();
    /* 转弯完成已自动接循迹时保留 PID 历史，不再提交重复的 TRACK 请求。 */
    if (g_mode_fsm.mode != MODE_TRACK) Car_Track();
}

/** @brief 初始化监测和路线进度，不启动电机，也不修改已填写的标定表。 */
void Route_Init(void)
{
    Route_t initial = {ROUTE_FOLLOW, JUNCTION_NONE, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    update_project_beep(1u); /* 重新从起点初始化时，清除上一轮尚未结束的项目短鸣。 */
    g_route = initial;
    s_line_candidate = s_line_frames = s_narrow_frames = s_active = 0u;
    s_turn_count_before = 0u;
    reset_junction();
}

/** @brief 只检查整张表；返回 1 表示参数齐全，不代表已经通过实车验证。 */
uint8_t Route_IsCalibrated(void)
{
    return (uint8_t)(invalid_step() == ROUTE_STEP_COUNT);
}

/**
 * @brief 仅允许停车时从起点启动，参数缺失则保留停车并指出第一条待标定步骤。
 * @return 1 已准备；0 未标定或尚在运行。
 * @note 不提交启动运动，下一控制帧由 READY 状态统一发送动作。
 */
uint8_t Route_Start(void)
{
    uint16_t invalid;
    uint32_t saved_primask = __get_PRIMASK();
    __disable_irq();

    if (g_mode_fsm.status != CAR_STOPPED) {
        __set_PRIMASK(saved_primask);
        return 0u;
    }
    invalid = invalid_step();
    Route_Init();
    if (invalid != ROUTE_STEP_COUNT) {
        g_route.step_index = invalid;
        g_route.project_id = g_route_plan[invalid].project_id;
        g_route.state = ROUTE_UNCALIBRATED;
        DebugLog_Write(LOG_WARN, "WAIT_CAL");
        __set_PRIMASK(saved_primask);
        return 0u;
    }
    g_route.enabled = s_active = 1u;
    g_route.state = ROUTE_READY;
    DebugLog_Write(LOG_INFO, "ROUTE_START");

    __set_PRIMASK(saved_primask);
    return 1u;
}

/** @brief 设置路线许可，不直接改运动模式；中止后只能通过 Route_Start 从起点重启。 */
void Route_Enable(uint8_t enabled)
{
    g_route.enabled = (uint8_t)(enabled != 0u);
}

/**
 * @brief 执行一帧路线，使用已完成控制帧计时，成功优先于同帧超时。
 * @param sensor_data 本帧八路采样。
 * @param external_command 本帧已有外部接管时为 1。
 * @note 外部接管保留其动作；超时通过请求邮箱交给运动层故障停车。
 */
void Route_Tick(const uint16_t *sensor_data, uint8_t external_command)
{
    const RouteStep *step;
    uint32_t elapsed_ms;

    /* 先服务提示音；路线未启动、被关闭或本帧外部接管时也能及时关蜂鸣器。 */
    update_project_beep((uint8_t)(external_command || !g_route.enabled ||
                                 g_mode_fsm.status != CAR_RUNNING));
    update_conditions(sensor_data);
    if (!s_active) 
        return;

    if (external_command || !g_route.enabled ||
        (g_mode_fsm.status != CAR_RUNNING && g_route.state != ROUTE_READY)) {
        s_active = g_route.enabled = 0u;
        g_route.state = ROUTE_ABORTED;
        return;
    }
    if (sensor_data == 0) 
    {
        return;
    }
    if (g_route.state == ROUTE_READY) {
        begin_step();
        return;
    }

    if (g_route.state_frames < UINT32_MAX / ROUTE_FRAME_MS) 
    {
        ++g_route.state_frames;
    }

    elapsed_ms = g_route.state_frames * ROUTE_FRAME_MS;
    step = &g_route_plan[g_route.step_index];

    switch (g_route.state) {
    case ROUTE_FOLLOW:
        if (elapsed_ms >= step->entry_ms && g_route.junction_event) {
            enter_state(step->action == ROUTE_ACTION_STRAIGHT ? ROUTE_CROSSING : ROUTE_APPROACH);
            Car_ImuForward(step->base_pwm);
            return;
        }
        break;
    case ROUTE_APPROACH:
        if (elapsed_ms >= step->pass_ms) {
            s_turn_count_before = g_mode_fsm.turn_count;
            enter_state(ROUTE_TURN);
            if (step->action == ROUTE_ACTION_LEFT) Car_TurnLeft90(MODE_TRACK);
            else Car_TurnRight90(MODE_TRACK);
            return;
        }
        break;
    case ROUTE_TURN:
        if (g_mode_fsm.turn_count != s_turn_count_before) {
            begin_recovery();
            return;
        }
        break;
    case ROUTE_CROSSING:
        if (elapsed_ms >= step->pass_ms && confirm_exit(sensor_data)) {
            begin_recovery();
            return;
        }
        break;
    case ROUTE_ENTRY:
        if (elapsed_ms >= step->entry_ms) {
            enter_state(ROUTE_OBSTACLE);
            /* 每个项目只在进入通行阶段时触发一次，后续帧不能重复续鸣。 */
            s_project_beep_frames = ROUTE_PROJECT_BEEP_FRAMES;
            Int_Beep_On();
            if (step->action != ROUTE_ACTION_TRACK_PASS) {
                Car_ImuForward(step->base_pwm);
            }
            DebugLog_Write(LOG_INFO, "PROJECT_IN");
            return;
        }
        break;
    case ROUTE_OBSTACLE:
        if (step->action == ROUTE_ACTION_TIMED_STOP) {
            if (elapsed_ms >= step->pass_ms) {
                g_route.state = ROUTE_FINISHED;
                update_project_beep(1u);
                s_active = g_route.enabled = 0u;
                DebugLog_Write(LOG_INFO, "ROUTE_DONE");
                Car_Stop();
                return;
            }
        } else if (elapsed_ms >= step->pass_ms && confirm_exit(sensor_data)) {
            begin_recovery();
            return;
        }
        break;
    case ROUTE_RECOVER:
        if (confirm_exit(sensor_data)) {
            finish_step();
            return;
        }
        break;
    default:
        break;
    }

    if (elapsed_ms >= step->timeout_ms) {
        /* 不先清阶段或计时，运动层捕获的故障现场仍指向发生超时的位置。 */
        s_active = g_route.enabled = 0u;
        update_project_beep(1u);
        Car_RouteTimeout();
    }
}

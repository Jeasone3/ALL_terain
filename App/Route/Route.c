/**
 * @file Route.c
 * @brief 按指定路口逐段进入项目，到调试截止段后保持灰度循迹。
 * @note Route 只安排阶段；TIM4 中的 ModeFSM 执行动作。PWM 和前移距离仍需实车标定。
 */
#include "Route.h"
#include "Mode_FSM.h"
#include "Debug_Log.h"
#include "Track_Task.h"
#include "Int_Beep.h"

#define ROUTE_CONFIRM_FRAMES 2u
#define ROUTE_FRAME_MS       10u
#define ROUTE_PROJECT_BEEP_FRAMES 10u /* 首个右路口确认后短鸣 100 ms，独立于阶段计时 */
#define ROUTE_TURN_APPROACH_MS   500u /* 转弯前保持航向前移 50 帧；试车起点，按车身对齐位置标定 */
#define ROUTE_TURN_APPROACH_PWM  500u /* 前移使用原普通 IMU 直行基础 PWM */

#if ROUTE_DEBUG_STEP_LIMIT == 0u
#define ROUTE_ACTIVE_STEP_COUNT ROUTE_STEP_COUNT
#else
#define ROUTE_ACTIVE_STEP_COUNT ROUTE_DEBUG_STEP_LIMIT
#endif

/* 仅填写已经确认的新顺序，后续项目继续追加到同一张表，并同步增加 ROUTE_STEP_COUNT。
 * 第 1 段保留用户已经设置的 2000 ms 入口；两段 IMU 时长均按本次要求设为 4000 ms。
 * 列顺序：路口动作、路线段号、IMU PWM、循迹入口 ms、IMU 通行 ms、阶段超时 ms、触发图案。
 * 超时沿用原前两个项目的 10000/13000 ms；等待路口距离也需要实车确认。
 */
RouteStep g_route_plan[ROUTE_STEP_COUNT] = {
    {ROUTE_ACTION_STRAIGHT, 1u, 600u, 2000u, 4000u, 10000u, JUNCTION_RIGHT}, //栅格
    {ROUTE_ACTION_RIGHT,    2u, 800u, 2000u, 4000u, 10000u, JUNCTION_CROSS} //减速带
};

volatile Route_t g_route;

static RouteJunction s_junction_candidate; /* 两帧候选路口类型 */
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
 * @brief 路口两帧确认，离开两帧后才解除新事件锁存。
 * @param raw_junction 本帧图案。
 * @return 无。
 * @note 类型变化不会产生第二个新事件，但仍更新确认类型；FOLLOW 可据此接受随后确认的十字。
 */
static void update_junction(RouteJunction raw_junction)
{
    if (raw_junction != s_junction_candidate) { //进入候选
        s_junction_candidate = raw_junction;
        s_junction_frames = 1u;
    } else if (s_junction_frames < ROUTE_CONFIRM_FRAMES) { //候选连续帧数
        ++s_junction_frames;
    }

    if (s_junction_frames < ROUTE_CONFIRM_FRAMES) //未连续两帧确认
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
 * @brief 两帧确认任意有线/无线变化，供日志观察，不直接决定项目完成。
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
 * @return 连续两帧有效时为 1，任一无效帧立即清零。
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
 * @brief 更新监测；入口、通行、恢复、转弯和调试截止阶段屏蔽路口图案。
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
        g_route.state == ROUTE_RECOVER || g_route.state == ROUTE_READY ||
        g_route.state == ROUTE_APPROACH || g_route.state == ROUTE_TURN ||
        g_route.state == ROUTE_CRUISE) {
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
 * @brief 检查本次启用段，返回第一条无效参数的位置。
 * @param 无。
 * @return 全部有效时返回 ROUTE_ACTIVE_STEP_COUNT，否则返回无效段的下标。
 * @note 不检查调试截止点之后的参数，避免尚未填写的后续项目阻止当前短段试车。
 *       入口与通行时间必须非零，阶段超时必须大于入口、通行和转弯前移时间。
 */
static uint16_t invalid_step(void)
{
    uint16_t i;
    for (i = 0u; i < ROUTE_ACTIVE_STEP_COUNT; ++i) {
        const RouteStep *step = &g_route_plan[i];
        if ((step->action != ROUTE_ACTION_LEFT && step->action != ROUTE_ACTION_RIGHT &&
             step->action != ROUTE_ACTION_STRAIGHT) ||
            (step->trigger_junction != JUNCTION_LEFT && step->trigger_junction != JUNCTION_RIGHT &&
             step->trigger_junction != JUNCTION_CROSS) ||
            step->entry_ms == 0u || step->pass_ms == 0u || step->timeout_ms <= step->pass_ms ||
            step->timeout_ms <= step->entry_ms || step->timeout_ms <= ROUTE_TURN_APPROACH_MS ||
            step->base_pwm == 0u || step->base_pwm > 1000u ||
            step->project_id == 0u) {
            return i;
        }
    }
    return ROUTE_ACTIVE_STEP_COUNT;
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
 * @brief 进入当前路线段，先循迹等待该段指定类型的路口。
 * @param 无。
 * @return 无。
 * @note 每次表项只调用一次；不会在 while(1) 重复重置 PID 或航向。
 */
static void begin_step(void)
{
    const RouteStep *step = &g_route_plan[g_route.step_index];
    g_route.project_id = step->project_id;
    reset_junction();
    enter_state(ROUTE_FOLLOW);
    if (g_mode_fsm.status != CAR_RUNNING || g_mode_fsm.mode != MODE_TRACK)
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
    begin_step(); /* 调试截止段在通行结束时进入 CRUISE，不会调用此处越过末尾。 */
}

/**
 * @brief 项目或道路动作完成后恢复循迹，再确认两帧窄黑线。
 * @param 无。
 * @return 无。
 * @note IMU 计时一到立即切回循迹；恢复期间仍屏蔽路口，两帧窄线后才启用下一段。
 */
static void begin_recovery(void)
{
    enter_state(ROUTE_RECOVER);
    reset_junction();
    /* 转弯完成已自动接循迹时保留 PID 历史，不再提交重复的 TRACK 请求。 */
    if (g_mode_fsm.mode != MODE_TRACK) Car_Track();
}

/**
 * @brief 结束入口对正，只提交一次本段 IMU 直行并开始通行计时。
 * @param step 当前有效路线段，提供随请求一起提交的基础 PWM。
 * @return 无。
 * @note OBSTACLE 从零统计已完成控制帧；请求由 ModeFSM 本帧第二次取邮箱接受，
 *       随后才建立 IMU 航向并输出电机。持续运行期间不得重复调用本函数。
 */
static void begin_project(const RouteStep *step)
{
    enter_state(ROUTE_OBSTACLE);
    Car_ImuForward(step->base_pwm);
    DebugLog_Write(LOG_INFO, "PROJECT_IN");
}

/**
 * @brief 初始化监测与路线进度，不启动电机，也不修改已填写的路线表。
 * @param 无。
 * @return 无。
 * @note 清除本轮路口、出口与提示音计数；调用后须由 Route_Start 显式启动。
 */
void Route_Init(void)
{
    Route_t initial = {ROUTE_FOLLOW, JUNCTION_NONE, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    update_project_beep(1u); /* 重新从起点初始化时，清除上一轮尚未结束的项目短鸣。 */
    g_route = initial;
    s_line_candidate = s_line_frames = s_narrow_frames = s_active = 0u;
    s_turn_count_before = 0u;
    reset_junction();
}

/**
 * @brief 检查本次启用段的参数是否齐全，不操作电机。
 * @param 无。
 * @return 1 表示启用段均有效；0 表示至少一段参数缺失或非法。
 * @note 函数名保持原接口；通过检查不代表 PWM、图案和前移距离已经实车验证。
 */
uint8_t Route_IsCalibrated(void)
{
    return (uint8_t)(invalid_step() == ROUTE_ACTIVE_STEP_COUNT);
}

/**
 * @brief 仅允许停车时从起点启动，参数缺失则保留停车并指出第一条待标定步骤。
 * @param 无。
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
    if (invalid != ROUTE_ACTIVE_STEP_COUNT) {
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

/**
 * @brief 设置路线许可，不直接改变当前运动模式或电机输出。
 * @param enabled 0 禁用；非零允许已经显式启动的路线继续运行。
 * @return 无。
 * @note 禁用由下一控制帧消费并中止路线；中止后单纯启用不能恢复，须停车后 Route_Start。
 */
void Route_Enable(uint8_t enabled)
{
    g_route.enabled = (uint8_t)(enabled != 0u);
}

/**
 * @brief 执行一帧路线，使用已完成控制帧计时，成功优先于同帧超时。
 * @param sensor_data 本帧八路采样。
 * @param external_command 本帧已有外部接管时为 1。
 * @return 无。
 * @note 每帧代表 10 ms，阶段切换只提交一次动作，不直接写电机或等待。
 *       外部接管保留其动作；超时通过请求邮箱交给运动层故障停车。
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

    /* 截止后仅更新显示计时和保留接管检查，不再计路线超时或派发后续动作。 */
    if (g_route.state == ROUTE_CRUISE) return;

    elapsed_ms = g_route.state_frames * ROUTE_FRAME_MS;
    step = &g_route_plan[g_route.step_index];

    switch (g_route.state) {
    case ROUTE_FOLLOW:
        /* 斜着进入十字时可能先确认单侧路口，新事件会先被单侧图案锁存。
         * 十字仍须连续两帧确认，但不再要求第二个新事件；接受后立即离开 FOLLOW，
         * 同一十字不会重复提交动作。左右单侧路口仍保留原有的新事件要求。 */
        if (g_route.junction == step->trigger_junction &&
            (g_route.junction_event || g_route.junction == JUNCTION_CROSS)) {
            /* 提示绑定首个右路口的确认，不在 IMU 开始或第二段重复触发。 */
            if (g_route.step_index == 0u) {
                s_project_beep_frames = ROUTE_PROJECT_BEEP_FRAMES;
                Int_Beep_On();
            }
            if (step->action == ROUTE_ACTION_STRAIGHT) {
                enter_state(ROUTE_ENTRY); /* 当前已经循迹，保留 PID 历史与电机控制。 */
            } else {
                enter_state(ROUTE_APPROACH);
                Car_ImuForward(ROUTE_TURN_APPROACH_PWM);
            }
            return;
        }
        break;
    case ROUTE_APPROACH:
        if (elapsed_ms >= ROUTE_TURN_APPROACH_MS) {
            s_turn_count_before = g_mode_fsm.turn_count;
            enter_state(ROUTE_TURN);
            if (step->action == ROUTE_ACTION_LEFT) Car_TurnLeft90(MODE_TRACK);
            else Car_TurnRight90(MODE_TRACK);
            return;
        }
        break;
    case ROUTE_TURN:
        if (g_mode_fsm.turn_count != s_turn_count_before) {
            enter_state(ROUTE_ENTRY);
            /* Route 先于本帧 IMU 更新；此处看到的是上一帧已经完成的转弯。
             * FSM 完成帧已经运行一次循迹，将其计入入口的已完成帧，避免多算 10 ms。
             * 不再次提交 TRACK，以免重置刚建立的循迹 PID 历史。 */
            g_route.state_frames = g_mode_fsm.state_frames;
            if (g_route.state_frames * ROUTE_FRAME_MS >= step->entry_ms) {
                begin_project(step); /* 入口配置不足一帧时，也从实际转完的帧边界计算。 */
            }
            return;
        }
        break;
    case ROUTE_ENTRY:
        if (elapsed_ms >= step->entry_ms) {
            begin_project(step);
            return;
        }
        break;
    case ROUTE_OBSTACLE:
        if (elapsed_ms >= step->pass_ms) {
            if (g_route.step_index + 1u >= ROUTE_ACTIVE_STEP_COUNT) {
                enter_state(ROUTE_CRUISE);
                reset_junction();
                Car_Track();
                DebugLog_Write(LOG_INFO, "DEBUG_END");
            } else {
                begin_recovery();
            }
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

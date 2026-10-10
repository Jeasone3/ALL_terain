/**
 * @file Route.c
 * @brief 条件监测和示例路线：循迹发现路口后，独立安排前移、转弯或穿越。
 * @note 后期路径规划只需替换路线选择策略，运动层仍通过 Car_* 接口执行动作。
 */
#include "Route.h"
#include "Mode_FSM.h"
#include "Debug_Log.h"
#include "Track_Task.h"            /* LINE_RAW_VALUE：传感器在线时的原始电平 */

/* 10 ms 一帧；下面的距离由时间近似，上车后按实际车速校准。 */
#define ROUTE_CONFIRM_FRAMES       3u
#define ROUTE_CROSS_CONFIRM_FRAMES 2u  /* 十字路口连续两帧在线即确认 */
#define ROUTE_APPROACH_FRAMES      80u  /* 转弯前移 800 ms，保留当前测试起点 */
#define ROUTE_CROSSING_FRAMES      10u  /* 十字路口直行 100 ms */

volatile Route_t g_route;

/* 候选连续帧只负责确认；锁存保证未离开路口时不重复触发。 */
static RouteJunction s_junction_candidate;
static uint8_t s_junction_frames; /* 连续帧计数 */
static uint8_t s_junction_latched; 
static uint8_t s_line_candidate; 
static uint8_t s_line_frames;
static RouteJunction s_turn_direction;
static uint32_t s_turn_count_before;

/**
 * @brief 从一帧八路灰度中识别路口类型，并判断是否存在引导线。
 * @param sensor_data 有效的八路灰度数组，顺序为 IN1 至 IN8。
 * @param any_line 接收本帧是否有任意一路在线；1：有线，0：无线。
 * @return 本帧原始路口类型，尚未经过连续帧确认。
 * @note 按 LINE_RAW_VALUE 比较，支持改变传感器电平极性；十字优先于左右路口。
 *       此函数只读取传感器数据，不修改模式、PID 或电机输出。
 */
static RouteJunction classify_sensor_frame(const uint16_t *sensor_data,
                                         uint8_t *any_line)
{
    uint8_t left_on = 1u;
    uint8_t right_on = 1u;
    uint8_t i;

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

    if (left_on && right_on) {
        return JUNCTION_CROSS;
    }
    if (left_on) {
        return JUNCTION_LEFT;
    }
    if (right_on) {
        return JUNCTION_RIGHT;
    }
    return JUNCTION_NONE;
}

/**
 * @brief 十字连续两帧、左右连续三帧确认，并产生离开后才能再次触发的单帧事件。
 * @param raw_junction 当前帧原始路口类型。
 * @return 无。
 * @note 类型改变时重新计数；左右与十字之间变化不会解除路口锁存。
 *       只有连续三帧 JUNCTION_NONE 才允许上报下一个路口。
 */
static void update_junction(RouteJunction raw_junction)
{
    uint8_t confirm_frames = raw_junction == JUNCTION_CROSS ?
                             ROUTE_CROSS_CONFIRM_FRAMES : ROUTE_CONFIRM_FRAMES;

    if (raw_junction != s_junction_candidate) {
        s_junction_candidate = raw_junction;
        s_junction_frames = 1u;
    } else if (s_junction_frames < confirm_frames) {
        ++s_junction_frames;
    }

    if (s_junction_frames < confirm_frames) {
        return;
    }

    g_route.junction = s_junction_candidate;
    if (s_junction_candidate == JUNCTION_NONE) {
        s_junction_latched = 0u;
    } else if (!s_junction_latched) {
        s_junction_latched = 1u;
        g_route.junction_event = 1u;
    }
}

/**
 * @brief 连续三帧确认有线或无线状态，并上报对应的状态变化事件。
 * @param raw_line 当前帧是否有任意通道在线；0：无线，1：有线。
 * @return 无。
 * @note 一帧或两帧抖动不会改变 line_found；初始化时默认无线，首次确认有线会上报事件。
 *       在 IMU、停车和故障期间仍调用，可供未来路线表判断目标黑线。
 */
static void update_line(uint8_t raw_line)
{
    if (raw_line != s_line_candidate) {
        s_line_candidate = raw_line;
        s_line_frames = 1u;
    } else if (s_line_frames < ROUTE_CONFIRM_FRAMES) {
        ++s_line_frames;
    }

    if (s_line_frames >= ROUTE_CONFIRM_FRAMES &&
        g_route.line_found != s_line_candidate) {
        g_route.line_found = s_line_candidate;
        if (s_line_candidate) {
            g_route.line_found_event = 1u;
        } else {
            g_route.line_lost_event = 1u;
        }
    }
}

/**
 * @brief 取消未完成的示例路线步骤，恢复为等待条件的路线状态。
 * @param 无。
 * @return 无。
 * @note 不发送 Car_* 请求，因此不会覆盖外部指令，也不会自动改变当前运动模式。
 *       路口与黑线检测值、锁存和事件保持原样，避免取消步骤造成同一路口重复触发。
 */
static void cancel_route_step(void)
{
    g_route.state = ROUTE_FOLLOW;
    g_route.state_frames = 0u;
    s_turn_direction = JUNCTION_NONE;
    s_turn_count_before = g_mode_fsm.turn_count;
}

/**
 * @brief 根据新路口选择示例动作，集中保留未来路径规划的替换入口。
 * @param junction 已经连续帧确认、且本帧首次上报的路口类型。
 * @return 无。
 * @note 当前测试策略：左右路口先 IMU 前移再转弯，十字路口 IMU 直行穿越。
 *       以后可在这里查询规划路线决定左、右或直行；不直接操作 PID 和电机。
 */
static void select_route_action(RouteJunction junction)
{
    //循迹态识别到转弯，先进入前移状态，等待 800 ms 后再发起转弯动作；十字路口直接进入穿越状态。
    if (junction == JUNCTION_LEFT || junction == JUNCTION_RIGHT) {
        s_turn_direction = junction;
        g_route.state = ROUTE_APPROACH;
        g_route.state_frames = 0u;
        Car_ImuForward();
    } else if (junction == JUNCTION_CROSS) {  //十字路口直接进入穿越状态，持续 100 ms 后恢复循迹。
        g_route.state = ROUTE_CROSSING;
        g_route.state_frames = 0u;
        Car_ImuForward();
    }
}

/**
 * @brief 初始化条件确认计数、事件和路线步骤，默认开启示例自动决策。
 * @param 无。
 * @return 无。
 * @note 在控制定时器启动前调用；不发出运动指令，首次稳定十字在第二帧、左右在第三帧产生事件。
 */
void Route_Init(void)
{
    g_route.state = ROUTE_FOLLOW;
    g_route.junction = JUNCTION_NONE;
    g_route.junction_event = 0u;
    g_route.line_found = 0u;
    g_route.line_found_event = 0u;
    g_route.line_lost_event = 0u;
    g_route.enabled = 1u;
    g_route.state_frames = 0u;
    s_junction_candidate = JUNCTION_NONE;
    s_junction_frames = 0u;
    s_junction_latched = 0u;
    s_line_candidate = 0u;
    s_line_frames = 0u;
    s_turn_direction = JUNCTION_NONE;
    s_turn_count_before = 0u;
}

/**
 * @brief 设置示例自动决策开关，不影响当前已经执行的运动动作。
 * @param enabled 0：关闭；非零：开启，内部保存为 1。
 * @return 无。
 * @note 可由主循环调用，关闭后在下一次 Route_Tick 取消路线步骤。
 *       关闭期间仍监测灰度；需要中止实际运动时应另外调用 Car_Stop。
 */
void Route_Enable(uint8_t enabled)
{
    g_route.enabled = (uint8_t)(enabled != 0u);
}

/**
 * @brief 执行一帧条件监测与路线调度，只向运动层提交动作请求。
 * @param sensor_data 已采样的八路灰度数组；空指针跳过检测和自动决策。
 * @param external_command 本帧已接受外部请求时传 1，否则传 0。
 * @return 无。
 * @note ModeFSM_Tick 每 10 ms 调用；先更新检测，再判断是否允许路线提交请求。
 *       单帧条件事件在本入口立即写日志，避免 100 ms 显示周期遗漏；稳定状态不重复记录。
 *       前移和穿越按运动层已执行帧数计时，转弯按完成总数变化判定，避免重置角度基准。
 */
void Route_Tick(const uint16_t *sensor_data, uint8_t external_command)
{
    uint8_t raw_line;
    RouteJunction raw_junction;

    /* 事件只保持一个控制帧，稳定检测值与路口锁存保留。 */
    g_route.junction_event = 0u;
    g_route.line_found_event = 0u;
    g_route.line_lost_event = 0u;
    if (sensor_data != 0) {
        raw_junction = classify_sensor_frame(sensor_data, &raw_line);
        update_junction(raw_junction);
        update_line(raw_line);
    }

    /* 条件监测在所有模式中保留；只在确认后的状态变化帧记录一次。 */
    if (g_route.junction_event != 0u) {
        const char *junction_text = g_route.junction == JUNCTION_LEFT ? "JUNC_L" :
                                    g_route.junction == JUNCTION_RIGHT ? "JUNC_R" : "CROSS";
        DebugLog_Write(LOG_INFO, junction_text);
    }
    if (g_route.line_found_event != 0u) {
        DebugLog_Write(LOG_INFO, "LINE_ON");
    }
    if (g_route.line_lost_event != 0u) {
        LogLevel level = (g_mode_fsm.mode == MODE_TRACK && g_mode_fsm.status == CAR_RUNNING) ?
                         LOG_WARN : LOG_INFO;
        DebugLog_Write(level, "LINE_OFF");
    }

    /* 手动请求、停车、故障和关闭自动路线都取消未完成步骤，但不自动恢复运动。 */
    if (external_command || !g_route.enabled ||
        g_mode_fsm.status != CAR_RUNNING) {
        cancel_route_step();
        return;
    }
    if (sensor_data == 0) {
        return;
    }

    switch (g_route.state) {
    //循迹态：自动识别转弯路口，并按顺序执行示例动作。
    case ROUTE_FOLLOW:
        if (g_route.state_frames != UINT32_MAX) {
            ++g_route.state_frames;
        }
        if (g_mode_fsm.mode == MODE_TRACK && g_route.junction_event) {
            select_route_action(g_route.junction);
        }
        break;

    case ROUTE_APPROACH:
        g_route.state_frames = g_mode_fsm.state_frames;//同步前移状态帧数，确保转弯动作在 800 ms 后发起。
        if (g_route.state_frames >= ROUTE_APPROACH_FRAMES) {
            /* 转弯另发动作，运动层在此刻建立新的角度起点。 */
            s_turn_count_before = g_mode_fsm.turn_count;
            g_route.state = ROUTE_TURN;
            g_route.state_frames = 0u;
            if (s_turn_direction == JUNCTION_LEFT) {
                Car_TurnLeft90(MODE_TRACK);
            } else {
                Car_TurnRight90(MODE_TRACK);
            }
        }
        break;

    case ROUTE_TURN:
        g_route.state_frames = g_mode_fsm.state_frames;
        if (g_mode_fsm.turn_count != s_turn_count_before) {
            /* 转后模式已经由运动层衔接，只恢复路线等待，不重复发送循迹请求。 */
            cancel_route_step();
        }
        break;

    case ROUTE_CROSSING:
        g_route.state_frames = g_mode_fsm.state_frames;
        if (g_route.state_frames >= ROUTE_CROSSING_FRAMES) {
            Car_Track();
            cancel_route_step();
        }
        break;

    default:
        cancel_route_step();
        break;
    }
}

/**
 * @file Route.c
 * @brief 条件监测、默认自动路线和用户自定义规划入口。
 * @note 项目动作与自动开关由用户在 PathPlan_Tick 中自行安排，不预设动作顺序。
 */
#include "Route.h"
#include "Mode_FSM.h"
#include "Debug_Log.h"
#include "Track_Task.h"            /* LINE_RAW_VALUE：传感器在线时的原始电平 */

/* 10 ms 一帧；下面的距离由时间近似，上车后按实际车速校准。 */
#define ROUTE_CONFIRM_FRAMES       3u      /* 路口连续三帧在线即确认 */
#define ROUTE_CROSS_CONFIRM_FRAMES 2u  /* 十字路口连续两帧在线即确认 */
#define ROUTE_APPROACH_FRAMES      80u     /* 默认转弯前移约 900 ms，需上车标定 */
#define ROUTE_CROSSING_FRAMES      500u    /* 备用穿越步骤约 5 秒；默认十字已不使用 */

volatile Route_t g_route;
/* 一个地图阶段变量：默认新十字加一，项目内是否推进由你自己决定。 */
volatile uint32_t g_project_stage;

/* 候选连续帧只负责确认；锁存保证未离开路口时不重复触发。 */
static RouteJunction s_junction_candidate; /* 连续帧确认的路口类型 */
static uint8_t s_junction_frames; /* 连续帧计数 */
static uint8_t s_junction_latched; // 路口锁存，防止未离开时重复触发
static uint8_t s_line_candidate; // 连续帧确认的引导线状态
static uint8_t s_line_frames; // 连续帧计数
static RouteJunction s_turn_direction; // 转弯方向
static uint32_t s_turn_count_before; // 转弯前计数

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

    g_route.junction = s_junction_candidate;//路口类型

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
 * @note 默认策略：左右路口先 IMU 前移再转弯，新十字只推进项目阶段并继续巡线。
 *       以后可在这里查询规划路线决定左、右或直行；不直接操作 PID 和电机。
 */
static void select_route_action(RouteJunction junction)
{
    /* 默认左右路口先前移约 900 ms 再转弯；新十字只把项目阶段加一。 */
    if (junction == JUNCTION_LEFT || junction == JUNCTION_RIGHT) {
        s_turn_direction = junction;
        g_route.state = ROUTE_APPROACH;
        g_route.state_frames = 0u;
        Car_ImuForward();
    } else if (junction == JUNCTION_CROSS) {
        /* 入口仅在新路口事件上调用，锁存期间不重复加一，不切换运动模式。 */
        if (g_project_stage != UINT32_MAX) {
            ++g_project_stage;
            DebugLog_Write(LOG_INFO, "STAGE_NEXT");
        }
    }
}

//----------------------------------------路径规划---------------------------------
/**
 * @brief 用户自己编写项目动作的入口，不内置直行、转弯或自动交接流程。
 * @param 无。
 * @return 无。
 * @note TIM4 每 10 ms 调用一次，关闭默认自动决策后仍调用；主动停车后也可继续检查条件。
 *       外部请求优先帧、空传感器输入或运动故障时跳过，不自动覆盖人工动作或恢复故障。
 *       每次切换动作只发一个 Car_* 请求，不使用 HAL_Delay 等待动作完成。
 */
static void PathPlan_Tick(void)
{
    /*
     * 你的项目代码写在这里：
     * 1. 用 g_project_stage、灰度事件、动作帧数或转弯完成计数判断条件。
     * 2. 自己开始安排动作前调用 Route_Enable(0u)，再提交一个 Car_* 请求。
     * 3. 后续动作、转向、停车和阶段推进都由你自己的条件决定。
     * 4. 项目结束想恢复自动巡线时，调用 Car_Track() 和 Route_Enable(1u)。
     * 此函数当前留空，默认不会自动执行任何项目动作，也不会修改你的开关。
     */

    //  if (g_project_stage == 1u && g_route.junction_event &&
    //      g_route.junction == JUNCTION_CROSS) {
    //     Route_Enable(0u);
    //     Car_TurnLeft90(MODE_TRACK);
    //     DebugLog_Write(LOG_INFO, "PROJECT_LEFT");
    //     Route_Enable(1u);
    // }

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
    g_project_stage = 0u;
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
    uint8_t auto_was_enabled;

    /* 事件只保持一个控制帧，稳定检测值与路口锁存保留。 */
    g_route.junction_event = 0u; //本帧确认了一个新路口事件
    g_route.line_found_event = 0u;
    g_route.line_lost_event = 0u;
    //
    if (sensor_data != 0) {
        raw_junction = classify_sensor_frame(sensor_data, &raw_line);
        update_junction(raw_junction); //滤波，防止细线划痕抖动误判，更新路口类型和事件
        update_line(raw_line); //判断时候丢线
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

    /* 先调你的规划函数。默认开关只控制后面的自动决策，不控制这个入口。 */
    auto_was_enabled = g_route.enabled;
    if (sensor_data != 0 && external_command == 0u &&
        g_mode_fsm.status != CAR_FAULT) {
        PathPlan_Tick();
    }

    /* 从关闭态恢复自动的这一帧，先接受你的动作；下一帧才执行默认决策。 */
    if (external_command || !auto_was_enabled || !g_route.enabled ||
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
        g_route.state_frames = g_mode_fsm.state_frames; /* 默认前移计时约 900 ms。 */
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

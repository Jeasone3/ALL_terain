/**
 * @file Route.c
 * @brief 按表通行，两段超声波后，第 20 项转弯、循迹、对齐并定时 IMU 直行后停车。
 * @note Route 只安排阶段；TIM4 中的 ModeFSM 执行动作。PWM 和前移距离仍需实车标定。
 */
#include "Route.h"
#include "Mode_FSM.h"
#include "Attitude.h"
#include "Debug_Log.h"
#include "Track_Task.h"
#include "Int_Beep.h"
#include "Int_SR04.h"

#define ROUTE_CONFIRM_FRAMES 2u
#define ROUTE_FRAME_MS       10u
#define ROUTE_PROJECT_BEEP_FRAMES 10u /* 接受指定路口或进入超声波开环时短鸣 100 ms，独立计时 */
#define ROUTE_TURN_APPROACH_MS   640u /* 表内路口转弯前保持航向前移 64 帧，按车身对齐位置标定 */
#define ROUTE_TURN_APPROACH_PWM  500u /* 前移使用原普通 IMU 直行基础 PWM */
#define ROUTE_FINAL_FORWARD_MS 3000u /* 完整路线收尾：从左转实际完成时起直行 3 秒 */
#define ROUTE_REVERSE_MS       2000u /* 大台阶入口：从左转完成的倒退首帧起计时 2 秒 */
#define ROUTE_REVERSE_DRIVE_MS 3200u /* 高台倒车：停车校准后保持 ideal_yaw 倒退 3.2 秒，PWM 与最后直行同 */
#define ROUTE_S1_IMU_MS        3000u /* 高台右转后以 PWM500 保持航向直行 3.5 秒，再切循迹找十字 */
#define ROUTE_S1_PWM           500u  /* S1 直行基础 PWM（慢速清出高台区域） */

#if ROUTE_DEBUG_STEP_LIMIT == 0u
#define ROUTE_ACTIVE_STEP_COUNT ROUTE_STEP_COUNT
#else
#define ROUTE_ACTIVE_STEP_COUNT ROUTE_DEBUG_STEP_LIMIT
#endif

/* 路线按表项顺序执行，project_id 为屏幕显示段号；追加表项时同步增加 ROUTE_STEP_COUNT。
 * 使用字段名填写参数，未列出的字段自动为 0，不再靠末尾的 0、1、2 判断各列含义。
 *
 * 路口动作：LEFT / RIGHT / LEFT_TRACK / RIGHT_TRACK 先对齐，以 500 PWM 前移 640 ms，再转 90°；第 12 项前移 450 ms，第 17 项前移 400 ms。
 *           LEFT_TRACK / RIGHT_TRACK 转完恢复循迹，省略项目 PWM、入口和通行时间。
 *           FOLLOW_LEFT 全程由灰度循迹 PID 沿左弯行驶，确认居中窄线后接下一项。
 *           STRAIGHT_TRACK 可先按 cal_before 校准，再循迹直行，通过路口并确认窄线后接下一项。
 * 项目时间：entry_ms 为转后或直行路口后的循迹时间；pass_ms 为随后保持 IMU 航向的通行时间。
 *           pass_ms 为 0 时只执行 entry_ms 循迹；base_pwm 保留非零供参数检查，不执行 IMU 通行。
 * 校准选项：cal_before = 1 表示路口动作前对齐保持并重建 yaw 基准。
 *           STRAIGHT 校准后进入定时入口循迹，STRAIGHT_TRACK 校准后直接恢复循迹并确认出口。
 *           cal_after = 1 表示转后再做一次校准。
 * 后续流程：final_seq = 1 接高台及 S1；final_seq = 2 在本项定时循迹结束后停车；省略时普通衔接。
 *           高台内部的原地转弯由专用状态执行，不使用表内路口转弯的前移时间。
 *
 * 未列出的项目参数与校准选项保持为 0；全程循迹左弯使用循迹模块已有的速度与 PID 参数。
 */
RouteStep g_route_plan[ROUTE_STEP_COUNT] = {
    {   /* 01 栅格：右侧路口直行，循迹 3 秒，再 IMU 通行 4.5 秒。 */
        .action = ROUTE_ACTION_STRAIGHT, .project_id = 1u, .trigger_junction = JUNCTION_RIGHT,
        .base_pwm = 500u, .entry_ms = 2400u, .pass_ms = 4500u, .timeout_ms = 10000u
    },
    {   /* 02 防滑带：十字右转，循迹 2 秒，再 IMU 通行 4.3 秒。 */
        .action = ROUTE_ACTION_RIGHT, .project_id = 2u, .trigger_junction = JUNCTION_CROSS,
        .base_pwm = 530u, .entry_ms = 2000u, .pass_ms = 4000u, .timeout_ms = 10000u
    },
    {   /* 03 小台阶：十字直行，循迹 2 秒，再 IMU 通行 6.5 秒。 */
        .action = ROUTE_ACTION_STRAIGHT, .project_id = 3u, .trigger_junction = JUNCTION_CROSS,
        .base_pwm = 500u, .entry_ms = 1500u, .pass_ms = 5000u, .timeout_ms = 10000u
    },
    {   /* 04 十字右转后循迹。 */
        .action = ROUTE_ACTION_RIGHT_TRACK, .project_id = 4u, .trigger_junction = JUNCTION_CROSS,
        .timeout_ms = 15000u
    },
    {   /* 05 右侧路口右转后循迹。 */
        .action = ROUTE_ACTION_RIGHT_TRACK, .project_id = 5u, .trigger_junction = JUNCTION_RIGHT,
        .timeout_ms = 15000u
    },





    {   /* 06 减速带：十字右转，循迹 1 秒，再 IMU 通行 5 秒。 */
        .action = ROUTE_ACTION_RIGHT, .project_id = 6u, .trigger_junction = JUNCTION_CROSS,
        .base_pwm = 500u, .entry_ms = 800u, .pass_ms = 5000u, .timeout_ms = 10000u
    },



    {   /* 07 草坪：十字直行，循迹 2 秒，再 IMU 通行 4 秒。 */
        .action = ROUTE_ACTION_STRAIGHT, .project_id = 7u, .trigger_junction = JUNCTION_CROSS,
        .base_pwm = 500u, .entry_ms = 2000u, .pass_ms = 4000u, .timeout_ms = 10000u
    },


    
    {   /* 08 大台阶：十字左转，循迹 200 ms，再以 400 PWM 通行 11 秒；倒退仍停用。 */
        .action = ROUTE_ACTION_LEFT, .project_id = 8u, .trigger_junction = JUNCTION_CROSS,
        .base_pwm = 400u, .entry_ms = 200u, .pass_ms = 9000u, .timeout_ms = 15000u
    },
    {   /* 09 窄桥：十字左转，循迹 400 ms，再 IMU 通行 4 秒，随后接高台及 S1。 */
        .action = ROUTE_ACTION_LEFT, .project_id = 9u, .trigger_junction = JUNCTION_CROSS,
        .base_pwm = 600u, .entry_ms = 400u, .pass_ms = 3500u, .timeout_ms = 10000u,
        .final_seq = 1u
    },

    /* 高台及 S1 后复用第 4、5 项流程：快速对齐、前移、右转后循迹，不重建 yaw 基准。 */
    {   /* 10 十字右转：与第 4 项一致，快速对齐、前移 640 ms、右转 90°后循迹确认窄线。 */
        .action = ROUTE_ACTION_RIGHT_TRACK, .project_id = 10u, .trigger_junction = JUNCTION_CROSS,
        .timeout_ms = 15000u
    },
    {   /* 11 右侧路口右转：与第 5 项一致，快速对齐、前移并闭环右转，确认窄线后接第 12 项。 */
        .action = ROUTE_ACTION_RIGHT_TRACK, .project_id = 11u, .trigger_junction = JUNCTION_RIGHT,
        .timeout_ms = 15000u
    },
    {   /* 12 左侧路口左转：前移 450 ms 后以 90°为目标左转，本次容差 45°；转完接超声波通行。 */
        .action = ROUTE_ACTION_LEFT_TRACK, .project_id = 12u, .trigger_junction = JUNCTION_LEFT,
        .timeout_ms = 15000u
    },
    {   /* 13 超声波通行后循迹找左路口：快速对齐、前移 640 ms、角度闭环左转 90°再循迹。 */
        .action = ROUTE_ACTION_LEFT_TRACK, .project_id = 13u, .trigger_junction = JUNCTION_LEFT,
        .timeout_ms = 15000u
    },
    {   /* 14 循迹找右路口：按相同前移流程闭环右转 90°，确认窄线后接下一项。 */
        .action = ROUTE_ACTION_RIGHT_TRACK, .project_id = 14u, .trigger_junction = JUNCTION_RIGHT,
        .timeout_ms = 15000u
    },
    {   /* 15 与第 10 项相同：慢速循迹找十字、快速对齐、前移 640 ms，仅转向改为闭环左转 90°。 */
        .action = ROUTE_ACTION_LEFT_TRACK, .project_id = 15u, .trigger_junction = JUNCTION_CROSS,
        .timeout_ms = 15000u
    },
    {   /* 16 慢速循迹识别右侧路口后按原流程闭环右转 90°，转后恢复循迹。 */
        .action = ROUTE_ACTION_RIGHT_TRACK, .project_id = 16u, .trigger_junction = JUNCTION_RIGHT,
        .timeout_ms = 15000u
    },
    {   /* 17 左侧路口左转：前移 400 ms、左转目标 90°、容差 45°，转完接第二段超声波。 */
        .action = ROUTE_ACTION_LEFT_TRACK, .project_id = 17u, .trigger_junction = JUNCTION_LEFT,
        .timeout_ms = 15000u
    },
    {   /* 18 第二段超声波通行结束后，慢速循迹找左路口并闭环左转 90°。 */
        .action = ROUTE_ACTION_LEFT_TRACK, .project_id = 18u, .trigger_junction = JUNCTION_LEFT,
        .timeout_ms = 15000u
    },
    {   /* 19 慢速循迹找右路口，按原前移和闭环流程右转 90°。 */
        .action = ROUTE_ACTION_RIGHT_TRACK, .project_id = 19u, .trigger_junction = JUNCTION_RIGHT,
        .timeout_ms = 15000u
    },
    {   /* 20 慢速循迹找十字，左转后循迹 800 ms，再对齐保持、重建航向基准，以 PWM500 直行 5 秒后停车。 */
        .action = ROUTE_ACTION_LEFT, .project_id = 20u, .trigger_junction = JUNCTION_CROSS,
        .base_pwm = 500u, .entry_ms = 800u, .pass_ms = 9000u, .timeout_ms = 15000u,
        .final_seq = 3u
    }
};

volatile Route_t g_route;

static RouteJunction s_junction_candidate; /* 两帧候选路口类型 */
static uint8_t s_junction_frames;          /* 候选类型连续帧数 */
static uint8_t s_junction_latched;         /* 当前路口已经报告，离开前不重复 */
static uint8_t s_line_candidate;           /* 任意通道在线的候选值 */
static uint8_t s_line_frames;              /* 有线/无线连续帧数 */
static uint8_t s_narrow_frames;            /* 当前阶段出口窄线连续帧数 */
static uint8_t s_active;                   /* 真正启动的路线，不等同于 enabled */
static uint8_t s_sonar_pending;            /* 前台已接受一次测距，尚未取出终态。 */
static volatile uint8_t s_sonar_valid;     /* 新的成功结果，10 ms 路线帧消费后清零。 */
static volatile float s_sonar_distance;   /* 与有效标记一起在短临界区内发布，单位 cm。 */
static uint32_t s_turn_count_before;       /* 本次转弯开始前的完成总数 */
static uint32_t s_align_count_before;      /* 本次对齐开始前的完成总数，仿 turn_count 轮询 */
static RouteState s_align_next_state;      /* 对齐完成后要进入的阶段（OBSTACLE 或 APPROACH） */
static uint8_t s_align_cal_flag;           /* 本次对齐是否跟 Attitude_SetYawOffset 校准：cal_before/cal_after 时置 1 */
static uint8_t s_project_beep_frames;      /* 项目提示音剩余帧数，独立于路线阶段计时 */

/**
 * @brief 每个控制帧维护路口、超声波开环短鸣和高台倒车持续鸣叫，到时或取消时关闭蜂鸣器。
 * @param cancel 非零立即关闭路线提示音；零消耗短鸣的一个 10 ms 帧，倒车期间保持鸣叫。
 * @return 无。
 * @note 只管理 Route 自己启动的提示音，不影响控制定时器启动前的上电校准蜂鸣。
 *       必须在路线提前返回前调用，保证离开项目或中止路线后不会留下持续鸣叫。
 */
static void update_route_beep(uint8_t cancel)
{
    /* 倒车提示由倒车阶段结束时关闭；中途停车、禁用、接管或故障时在此提前关闭。 */
    if (g_route.state == ROUTE_REVERSE_DRIVE) {
        if (cancel) Int_Beep_Off();
        return;
    }
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
 * @brief 更新监测；入口、通行、恢复、转弯、倒退和调试截止阶段屏蔽路口图案。
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
        g_route.state == ROUTE_ALIGN || g_route.state == ROUTE_APPROACH ||
        g_route.state == ROUTE_TURN ||
        g_route.state == ROUTE_CRUISE || g_route.state == ROUTE_FINAL_TURN ||
        g_route.state == ROUTE_FINAL_FORWARD || g_route.state == ROUTE_FINISHED ||
        g_route.state == ROUTE_REVERSE ||
        g_route.state == ROUTE_REVERSE_DRIVE || g_route.state == ROUTE_FINAL_RIGHT_TURN ||
        g_route.state == ROUTE_S1_IMU || g_route.state == ROUTE_SONAR_FOLLOW ||
        g_route.state == ROUTE_SONAR_OPEN_LOOP) {
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
 * @brief 返回指定表项转弯前的前移时长，供参数检查与阶段计时共同使用。
 * @param step_index 路线表下标，从 0 开始。
 * @return 前移时长（毫秒）；第 12 项使用 450 ms，第 17 项使用 400 ms，其余项使用原统一参数。
 * 
 * 
 * 指定路段，执行转向前移时长
 */
static uint32_t turn_approach_ms(uint16_t step_index)
{
    /* 第 17 项前移 40 个控制帧；表项下标从 0 开始，第 12 项仍使用原来的 450 ms。 */
    if (step_index == 16u) 
        return 400u;
    return step_index == 11u ? 420u : ROUTE_TURN_APPROACH_MS;
}

/**
 * @brief 检查本次启用段，返回第一条无效参数的位置。
 * @param 无。
 * @return 全部有效时返回 ROUTE_ACTIVE_STEP_COUNT，否则返回无效段的下标。
 * @note 只检查 ROUTE_START_STEP 至调试截止项，跳过起始项之前和截止项之后的参数。
 *       项目入口与通行时间必须非零，阶段超时必须大于入口、通行和转弯前移时间。
 *       第 8 项左转后的阶段超时还须大于固定的 2 秒倒退时间。
 *       RIGHT_TRACK 可接后续路段，入口、通行和对应 PWM 为零，不虚构项目时长。
 */
static uint16_t invalid_step(void)
{
    uint16_t i;
    for (i = ROUTE_START_STEP - 1u; i < ROUTE_ACTIVE_STEP_COUNT; ++i) {
        const RouteStep *step = &g_route_plan[i];
        if ((step->action != ROUTE_ACTION_LEFT && step->action != ROUTE_ACTION_RIGHT &&
             step->action != ROUTE_ACTION_STRAIGHT && step->action != ROUTE_ACTION_RIGHT_TRACK &&
             step->action != ROUTE_ACTION_LEFT_TRACK && step->action != ROUTE_ACTION_FOLLOW_LEFT &&
             step->action != ROUTE_ACTION_STRAIGHT_TRACK) ||
            (step->trigger_junction != JUNCTION_LEFT && step->trigger_junction != JUNCTION_RIGHT &&
             step->trigger_junction != JUNCTION_CROSS) ||
            step->timeout_ms <= turn_approach_ms(i) ||
            step->project_id == 0u) {
            return i;
        }
        if (i == 7u && step->action == ROUTE_ACTION_LEFT && step->timeout_ms <= ROUTE_REVERSE_MS) {
            return i;
        }
        if (step->action == ROUTE_ACTION_RIGHT_TRACK || step->action == ROUTE_ACTION_LEFT_TRACK ||
            step->action == ROUTE_ACTION_FOLLOW_LEFT || step->action == ROUTE_ACTION_STRAIGHT_TRACK) {
            if (step->base_pwm != 0u ||
                step->entry_ms != 0u || step->pass_ms != 0u) return i;
        } else if (step->entry_ms == 0u || step->base_pwm == 0u || step->base_pwm > 1000u ||
                   step->timeout_ms <= step->entry_ms) {
            return i;
        } else if (step->pass_ms != 0u && step->timeout_ms <= step->pass_ms) {
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
/**
 * @brief 路口动作对应的理想朝向增量，均为 90° 的整数倍。
 * @param action 当前段的路口动作。
 * @return LEFT=+90、RIGHT/RIGHT_TRACK=-90、STRAIGHT=0，单位：度。
 * @note 此处只提供路口增量；两段超声波出口各补 180°。对齐本身 Δθ=0，目标为当前 ideal_yaw。
 */
static float route_action_yaw_delta(RouteAction action)
{
    switch (action)
    {
    case ROUTE_ACTION_LEFT:  return  90.0f;
    case ROUTE_ACTION_RIGHT: return -90.0f;
    case ROUTE_ACTION_RIGHT_TRACK: return -90.0f;
    case ROUTE_ACTION_LEFT_TRACK:  return  90.0f;
    case ROUTE_ACTION_FOLLOW_LEFT: return  90.0f;
    default: return 0.0f;  /* ROUTE_ACTION_STRAIGHT */
    }
}

static void enter_state(RouteState state)
{
    g_route.state = state;
    g_route.state_frames = 0u;
    s_narrow_frames = 0u;
}

/**
 * @brief 切换当前路段的循迹速度及丢线降速许可，不重置 PID 或修改 IMU 动作参数。
 * @param slow_track 非零表示第 10、11、15、16、18、19、20 项的慢速循迹；零表示恢复普通循迹配置。
 * @return 无。
 * @note 第 10 项恢复循迹及第 11 项转后持续循迹均保留本配置，中止、结束或重新初始化时恢复默认。
 */
static void set_track_parameters(uint8_t slow_track)
{
    g_line_controller.base_speed = slow_track ? 250 : LINE_DEFAULT_BASE_PWM;
    g_line_controller.lost_slowdown_enabled = !slow_track;
}

/**
 * @brief 进入当前路线段，设置本段循迹参数并等待指定类型的路口。
 * @param 无。
 * @return 无。
 * @note 每次表项只调用一次；不会重复重置 PID 或航向。
 */
static void begin_step(void)
{
    const RouteStep *step = &g_route_plan[g_route.step_index];
    set_track_parameters((uint8_t)(g_route.step_index == 9u || g_route.step_index == 10u ||
                                   g_route.step_index == 14u || g_route.step_index == 15u ||
                                   g_route.step_index == 17u || g_route.step_index == 18u ||
                                   g_route.step_index == 19u)); /* 第 15、16、18、19、20 项复用第 10 项的 PWM250 循迹配置。 */
    g_route.project_id = step->project_id;
    reset_junction();
    enter_state(ROUTE_FOLLOW);
    if (g_mode_fsm.status != CAR_RUNNING || g_mode_fsm.mode != MODE_TRACK) //上电后首次进入循迹
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
    begin_step(); /* 截止段通行结束或 RIGHT_TRACK 转完时进入 CRUISE，不会从此处越过末尾。 */
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
    if (g_mode_fsm.mode != MODE_TRACK)
     Car_Track();
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
    /* 项目入口 TRACK→IMU：先对齐到 ideal_yaw 清除循迹带来的偏航，再进 IMU 直行。
     * 对齐本身 Δθ=0，目标就是当前 ideal_yaw；FSM 到位后 align_count++，本层轮询衔接。
     * base_pwm 由 ROUTE_ALIGN 完成后从 step 重新读取，此处不用 step。 */
    (void)step;
    s_align_count_before = g_mode_fsm.align_count;
    s_align_next_state = ROUTE_OBSTACLE;
    enter_state(ROUTE_ALIGN);
    Car_AlignToHold(); /* 进项目对齐到位后保持 800ms，等待角度稳定对齐项目再切 IMU 直行 */
    DebugLog_Write(LOG_INFO, "ALIGN_IN");
}

/**
 * @brief 初始化监测与路线进度，不启动电机，也不修改已填写的路线表。
 * @param 无。
 * @return 无。
 * @note 清除本轮路口、出口与提示音计数；调用后须由 Route_Start 显式启动。
 */
void Route_Init(void)
{
    Route_t initial = {ROUTE_FOLLOW, JUNCTION_NONE, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0.0f};
    update_route_beep(1u); /* 重新初始化前，关闭上一轮尚未结束的短鸣或倒车提示音。 */
    g_route = initial;
    set_track_parameters(0u);
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
 * @brief 仅允许停车时从配置的起始项启动，参数缺失则保留停车并指出第一条待标定步骤。
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
    /* 保留原路线表下标，后续仍逐项递增，第 8 项等依赖下标的特殊处理不受起点改变影响。
     * 先同步段号，使 READY 阶段的日志与显示立即指向本次实际起始项目。 */
    g_route.step_index = ROUTE_START_STEP - 1u;
    g_route.project_id = g_route_plan[g_route.step_index].project_id;
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
 *       每段指定路口被接受时短鸣一次，蜂鸣计时独立于前移、转弯和项目阶段。
 *       RIGHT_TRACK 转完后保留 FSM 已接入的循迹；有后续段则先恢复窄线，截止段直接 CRUISE。
 *       第 10、11 项右转后确认窄线接下一项；第 12 项左转使用局部 45°容差，转完接超声波通行。
 *       外部接管保留其动作；超时通过请求邮箱交给运动层故障停车。
 */
void Route_Tick(const uint16_t *sensor_data, uint8_t external_command)
{
    const RouteStep *step;
    uint32_t elapsed_ms;

    /* 先服务提示音；路线未启动、被关闭或本帧外部接管时也能及时关蜂鸣器。 */
    update_route_beep((uint8_t)(external_command || !g_route.enabled ||
                                 g_mode_fsm.status != CAR_RUNNING));
   //更新路口事件
    update_conditions(sensor_data);
    if (!s_active) 
        return;

    /* 对齐完成会先停车并递增计数；仅这一明确的完成事件允许下方 ROUTE_ALIGN 衔接。
     * 外部停车、禁用和故障仍按原流程中止，不能凭 ALIGN 阶段绕过停车检查。 */
    if (external_command || !g_route.enabled ||
        (g_mode_fsm.status != CAR_RUNNING && g_route.state != ROUTE_READY &&
         !(g_mode_fsm.status == CAR_STOPPED && g_route.state == ROUTE_ALIGN &&
           g_mode_fsm.align_count != s_align_count_before))) {
        s_active = g_route.enabled = 0u;
        g_route.state = ROUTE_ABORTED;
        set_track_parameters(0u);
        return;
    }
    //灰度线检测校验
    if (sensor_data == 0) 
    {
        return;
    }

    //开始进行路线表
    if (g_route.state == ROUTE_READY) {
        begin_step();
        return;
    }
    if (g_route.state_frames < UINT32_MAX / ROUTE_FRAME_MS) //小于最大计数值
    {
        ++g_route.state_frames;
    }

    /* 截止后仅更新显示计时和保留接管检查，不再计路线超时或派发后续动作。 */
    if (g_route.state == ROUTE_CRUISE) 
        return;

    /* 只消费本阶段新完成的有效距离；40 cm 和超时均保持当前动作。
     * 完成一次开环通行后回到普通循迹，前台随即停止发送超声波触发脉冲。 */
    if (g_route.state == ROUTE_SONAR_FOLLOW || g_route.state == ROUTE_SONAR_OPEN_LOOP)
    {
        if (s_sonar_valid != 0u)
        {
            s_sonar_valid = 0u;
            if (g_route.state == ROUTE_SONAR_FOLLOW && s_sonar_distance < 40.0f)
            {
                enter_state(ROUTE_SONAR_OPEN_LOOP);
                /* 两段超声波均在进入开环时短鸣一次，后续测距帧不重复触发。 */
                s_project_beep_frames = ROUTE_PROJECT_BEEP_FRAMES;
                Int_Beep_On();
                if (g_route.step_index == 16u)
                    //方PWM
                    Car_OpenLoopWithPWM(460u, 200u); /* 第二段单独给定 PWM，不覆盖第一段的调参。 */
                else
                    Car_OpenLoop();
            }
            else if (g_route.state == ROUTE_SONAR_OPEN_LOOP && s_sonar_distance > 40.0f)
            {
                /* 两段超声波通行的出口均与入口反向，各补上路线目标的 180°净变化。
                 * 实测 yaw 持续积分保留，不在出口重设基准；第 13、18 项路口再按新目标对齐。
                 * 随后立即离开超声波状态，保证每段只更新一次。 */
                if (g_route.step_index == 11u || g_route.step_index == 16u)
                    g_route.ideal_yaw += 180.0f;

                /* 两段超声波分别推进第 13、18 项；begin_step 清除旧路口并恢复循迹。
                 * 截止在超声波所属项时只恢复循迹，不能越过启用范围。 */
                if (g_route.step_index + 1u < ROUTE_ACTIVE_STEP_COUNT)
                    finish_step();
                else
                {
                    enter_state(ROUTE_CRUISE);
                    Car_Track();
                }
                DebugLog_Write(LOG_INFO, "SONAR_DONE");
            }
        }
        return;
    }
    
    //计算已运行时间
    elapsed_ms = g_route.state_frames * ROUTE_FRAME_MS;
    //根据当前状态执行相应操作
    step = &g_route_plan[g_route.step_index];

    switch (g_route.state) {
    case ROUTE_FOLLOW: //一般巡线
        /* 斜着进入十字时可能先确认单侧路口，新事件会先被单侧图案锁存。
         * 十字仍须连续两帧确认，但不再要求第二个新事件；接受后立即离开 FOLLOW，
         * 同一十字不会重复提交动作。左右单侧路口仍保留原有的新事件要求。 */
        if (g_route.junction == step->trigger_junction &&
            (g_route.junction_event || g_route.junction == JUNCTION_CROSS)) {
            /* 包含第一个项目结束后的第二段十字；接受后立即离开 FOLLOW，同一路口不重复鸣叫。 */
            s_project_beep_frames = ROUTE_PROJECT_BEEP_FRAMES;
            Int_Beep_On();
            if (step->action == ROUTE_ACTION_STRAIGHT) {
                if (step->cal_before) {
                    /* 直行项路口校准：复用保持对齐和 ENTRY 衔接，完成后才开始入口循迹计时。
                     * 不执行转弯前移、不改变 ideal_yaw；入口结束后的项目校准仍照常执行。 */
                    s_align_count_before = g_mode_fsm.align_count;
                    s_align_next_state = ROUTE_ENTRY;
                    enter_state(ROUTE_ALIGN);
                    Car_AlignToHold();
                } else {
                    enter_state(ROUTE_ENTRY); /* 普通直行项继续循迹，保留 PID 历史与电机控制。 */
                }
            } else if (step->action == ROUTE_ACTION_STRAIGHT_TRACK) {
                if (step->cal_before) {
                    /* 纯循迹直行项也可在路口先对齐保持校准，完成后复用 RECOVER 切回循迹。
                     * 不进入定时入口或项目通行；确认离开当前十字后才允许下一项接收路口。 */
                    s_align_count_before = g_mode_fsm.align_count;
                    s_align_next_state = ROUTE_RECOVER;
                    enter_state(ROUTE_ALIGN);
                    Car_AlignToHold();
                } else {
                    begin_recovery(); /* 未配置校准时继续循迹直行，不改变理想朝向。 */
                }
            } else if (step->action == ROUTE_ACTION_FOLLOW_LEFT) {
                /* 左弯由当前灰度偏差驱动循迹 PID 完成，不提交对齐、前移或 IMU 转弯请求。
                 * RECOVER 屏蔽本路口，连续两帧居中窄线后才允许下一项接收路口。
                 * 理想朝向只更新一次，供后续项目的 IMU 对齐使用，不参与本次循迹控制。 */
                g_route.ideal_yaw += route_action_yaw_delta(step->action);
                begin_recovery();
                DebugLog_Write(LOG_INFO, "FOLLOW_LEFT");
            } else {
                /* 路口前移前 TRACK→IMU：先对齐到 ideal_yaw，防循迹偏航被 IMU 直行 PID
                 * 放大成侧摆；对齐完成后再前移、更新 ideal_yaw、绝对转弯。
                 * cal_before 时用 align-hold 代替快速对齐，并在对齐完成时重建 yaw 基准。 */
                s_align_count_before = g_mode_fsm.align_count;
                s_align_next_state = ROUTE_APPROACH;
                s_align_cal_flag = step->cal_before;
                enter_state(ROUTE_ALIGN);
                if (step->cal_before) Car_AlignToHold(); else Car_AlignTo();
            }
            return;
        }
        break;
    case ROUTE_ALIGN: //功能切换前原地慢转到 ideal_yaw
        if (g_mode_fsm.align_count != s_align_count_before)
        {
            if (s_align_next_state == ROUTE_OBSTACLE)
            {
                /* 进项目切 IMU 直行前重建 yaw 基准到 ideal_yaw：ALIGN_HOLD 800ms 静止期间
                 * 陀螺零偏残值让 yaw 单向虚漂(车没转 yaw 变)，切直行时 error=ideal_yaw-yaw≠0，
                 * PID 把 yaw 拉回 ideal_yaw 的同时把车体真实朝向反向拉偏，车斜着冲台阶。
                 * 重置基准让 yaw=ideal_yaw、error=0，复刻纯 IMU 测试(main.c:178)的零误差起步。 */
                Attitude_SetYawOffset(g_route.ideal_yaw);
                enter_state(ROUTE_OBSTACLE);
                Car_ImuForward(step->base_pwm);
                DebugLog_Write(LOG_INFO, "PROJECT_IN");
            }
            else if (s_align_next_state == ROUTE_REVERSE_DRIVE)
            {
                /* 平台停车校准后倒车：重建 yaw 基准到 ideal_yaw，倒车起步 error=0 真正直着倒。 */
                Attitude_SetYawOffset(g_route.ideal_yaw);
                enter_state(ROUTE_REVERSE_DRIVE);
                Car_Reverse(500u);
                Int_Beep_On(); /* 与倒车请求同帧开启，整个高台倒车阶段持续鸣叫。 */
                DebugLog_Write(LOG_INFO, "PLATFORM_REV");
            }
            else if (s_align_next_state == ROUTE_FINAL_RIGHT_TURN)
            {
                /* 倒车后停车校准、右转 90°：重建基准清除倒车段虚漂，转后目标 ideal_yaw-90 准确。
                 * Attitude_SetYawOffset 须在 ideal_yaw += -90 之前，用更新前的 ideal_yaw 重建基准。 */
                Attitude_SetYawOffset(g_route.ideal_yaw);
                s_turn_count_before = g_mode_fsm.turn_count;
                enter_state(ROUTE_FINAL_RIGHT_TURN);
                g_route.ideal_yaw += route_action_yaw_delta(ROUTE_ACTION_RIGHT); /* -90° */
                Car_TurnRight90(MODE_IMU);
                DebugLog_Write(LOG_INFO, "FINAL_R90");
            }
            else if (s_align_next_state == ROUTE_ENTRY)
            {
                /* 直行项路口校准或 cal_after 转后校准完成，切回循迹并从零计入口时间。 */
                Attitude_SetYawOffset(g_route.ideal_yaw);
                enter_state(ROUTE_ENTRY);
                Car_Track(); /* 对齐期间 FSM 在 IMU_ALIGN，需切回循迹 */
                DebugLog_Write(LOG_INFO, step->action == ROUTE_ACTION_STRAIGHT ? "PRECAL>ENTRY" : "AFTCAL>ENTRY");
            }
            else if (s_align_next_state == ROUTE_RECOVER)
            {
                /* 纯循迹直行项路口校准或 cal_after 转后校准完成，恢复循迹并重新确认出口窄线。 */
                Attitude_SetYawOffset(g_route.ideal_yaw);
                enter_state(ROUTE_RECOVER);
                Car_Track();
                reset_junction();
                DebugLog_Write(LOG_INFO, step->action == ROUTE_ACTION_STRAIGHT_TRACK ? "PRECAL>REC" : "AFTCAL>REC");
            }
            else /* ROUTE_APPROACH */
            {
                if (s_align_cal_flag)
                    Attitude_SetYawOffset(g_route.ideal_yaw); /* cal_before：转前 align-hold 校准 */
                enter_state(ROUTE_APPROACH);
                Car_ImuForward(ROUTE_TURN_APPROACH_PWM);
            }
        }
        break;
    case ROUTE_APPROACH: //转向前移
        if (elapsed_ms >= turn_approach_ms(g_route.step_index)) { // 到达当前表项的前移时间
            s_turn_count_before = g_mode_fsm.turn_count;
            enter_state(ROUTE_TURN);
            /* 提交转弯前更新理想朝向，FSM 取此绝对值作为转弯目标，转完即对齐。 */
            g_route.ideal_yaw += route_action_yaw_delta(step->action);
            if (step->action == ROUTE_ACTION_LEFT || step->action == ROUTE_ACTION_LEFT_TRACK) {
                if (g_route.step_index == 11u || g_route.step_index == 16u)
                    Car_TurnLeft90WithTolerance(MODE_TRACK, 45.0f); /* 仅第 12、17 项使用 45°到位容差 */
                else
                    Car_TurnLeft90(MODE_TRACK);
            } else
                Car_TurnRight90(MODE_TRACK);
            return;
        }
        break;
    case ROUTE_TURN:     //陀螺仪转向
        if (g_mode_fsm.turn_count != s_turn_count_before) {
            /* 大台阶后退暂时停用；转完后直接执行下方的入口循迹，不等待倒退计时。 */
            // if (g_route.step_index == 7u && step->action == ROUTE_ACTION_LEFT) {
            //     enter_state(ROUTE_REVERSE);
            //     /* FSM 在转完当帧已开始倒退；继承这一帧，保证 2 秒从实际倒退开始计算。
            //      * 不重发动作或重置姿态，继续保持左转的 +90°目标航向。 */
            //     g_route.state_frames = g_mode_fsm.state_frames;
            //     return;
            // }
            if (step->action == ROUTE_ACTION_RIGHT_TRACK || step->action == ROUTE_ACTION_LEFT_TRACK) {
                /* RIGHT_TRACK/LEFT_TRACK 转完接循迹；cal_after 时先 align-hold 校准再恢复循迹。 */
                if (step->cal_after) {
                    s_align_count_before = g_mode_fsm.align_count;
                    s_align_next_state = ROUTE_RECOVER;
                    s_align_cal_flag = 1u;
                    enter_state(ROUTE_ALIGN);
                    Car_AlignToHold();
                } else if (g_route.step_index == 11u || g_route.step_index == 16u) {
                    /* 第 12、17 项转后先执行对应超声波通行，再衔接第 13、18 项。 */
                    s_sonar_pending = s_sonar_valid = 0u;
                    enter_state(ROUTE_SONAR_FOLLOW);
                    reset_junction();
                    DebugLog_Write(LOG_INFO, "SONAR_BEGIN");
                } else if (g_route.step_index + 1u >= ROUTE_ACTIVE_STEP_COUNT) {
                    enter_state(ROUTE_CRUISE);
                    reset_junction();
                    DebugLog_Write(LOG_INFO, "DEBUG_END");
                } else {
                    begin_recovery(); /* 先确认离开原路口，才允许下一段接收新事件。 */
                }
                return;
            }

            /* LEFT/RIGHT 转完接入口；cal_after 时先 align-hold 校准再入口循迹。 */
            if (step->cal_after) {
                s_align_count_before = g_mode_fsm.align_count;
                s_align_next_state = ROUTE_ENTRY;
                s_align_cal_flag = 1u;
                enter_state(ROUTE_ALIGN);
                Car_AlignToHold();
                return;
            }
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

    case ROUTE_REVERSE: //大台阶入口先后退，再由灰度循迹向前对正
        if (elapsed_ms >= ROUTE_REVERSE_MS) {
            enter_state(ROUTE_ENTRY);
            Car_Track(); /* 本帧开始前进循迹，入口的 3 秒单独从零计时。 */
            return;
        }
        break;

    case ROUTE_ENTRY: //项目入口通过循迹对正
        if (elapsed_ms >= step->entry_ms) {
            if (step->final_seq == 2u) {
                /* 末项复用入口循迹计时：从转完实际开始循迹的首帧起算，到时只停车一次。
                 * 先关闭本轮调度，避免下一帧把正常停车当作外部接管而改写 FINISHED。 */
                s_active = g_route.enabled = 0u;
                enter_state(ROUTE_FINISHED);
                set_track_parameters(0u);
                Car_Stop();
                DebugLog_Write(LOG_INFO, "ROUTE_DONE");
            } else if (step->pass_ms == 0u)
                begin_recovery(); /* 纯循迹项目(隧道)：入口循迹到时直接恢复循迹，不走 IMU 通行 */
            else
                begin_project(step);
            return;
        }
        break;

    case ROUTE_OBSTACLE: //障碍物使用imu通行
        if (elapsed_ms >= step->pass_ms) {
            if (step->final_seq == 3u) {
                /* 定时 IMU 直行完成：先结束路线调度，再提交停车，防止下一帧误判为接管。
                 * 停车仍由唯一运动出口执行，不恢复循迹，也不等待灰度出口线。 */
                s_active = g_route.enabled = 0u;
                enter_state(ROUTE_FINISHED);
                set_track_parameters(0u);
                Car_Stop();
                DebugLog_Write(LOG_INFO, "ROUTE_DONE");
            } else if (step->final_seq == 1u) {
                /* 该段通行到时触发高台特殊收尾：原地左转 90°、IMU 直行 3 秒、倒车、右转。
                 * MODE_IMU 使 FSM 在转完的同一帧保持 +90°目标、以 500 PWM 直行。 */
                reset_junction();
                s_turn_count_before = g_mode_fsm.turn_count;
                enter_state(ROUTE_FINAL_TURN);
                g_route.ideal_yaw += route_action_yaw_delta(ROUTE_ACTION_LEFT);
                Car_TurnLeft90(MODE_IMU);
            } else if (g_route.step_index + 1u >= ROUTE_ACTIVE_STEP_COUNT) {
                /* 调试截止：持续循迹，不执行收尾。 */
                reset_junction();
                enter_state(ROUTE_CRUISE);
                Car_Track();
                DebugLog_Write(LOG_INFO, "DEBUG_END");
            } else {
                begin_recovery();
            }
            return;
        }
        break;

    case ROUTE_FINAL_TURN: //最终转弯，高台
        if (g_mode_fsm.turn_count != s_turn_count_before) {
            enter_state(ROUTE_FINAL_FORWARD);
            /* Route 在本帧 IMU 更新前运行，FSM 上一帧转完后已执行了一帧直行。
             * 把这一帧计入 3 秒；不重复发 IMU 请求，避免重建航向和清除 PID 历史。 */
            g_route.state_frames = g_mode_fsm.state_frames;
            return;
        }
        break;

    case ROUTE_FINAL_FORWARD: //最终直行
        if (elapsed_ms >= ROUTE_FINAL_FORWARD_MS) {
            /* 高台直行到时：进入平台停车对齐保持(对齐 ideal_yaw+800ms+校准)，再倒车。
             * 不在此 Car_Stop 进 FINISHED——CAR_STOPPED 会被 Route 中止检查当作被接管，
             * 对齐保持是唯一能在停车保持期间维持 CAR_RUNNING 的机制。 */
            s_align_count_before = g_mode_fsm.align_count;
            s_align_next_state = ROUTE_REVERSE_DRIVE;
            enter_state(ROUTE_ALIGN);
            Car_AlignToHold();
            DebugLog_Write(LOG_INFO, "PLATFORM_STOP");
        }
        return; /* 固定 3 秒由本阶段计时结束，不再套用项目通行的超时参数。 */

    case ROUTE_REVERSE_DRIVE: //高台倒车：保持 ideal_yaw 倒退 3.2 秒，再停车对齐校准
        if (elapsed_ms >= ROUTE_REVERSE_DRIVE_MS) {
            Int_Beep_Off(); /* 倒车计时结束即关闭，后续停车校准和右转期间不再鸣叫。 */
            s_align_count_before = g_mode_fsm.align_count;
            s_align_next_state = ROUTE_FINAL_RIGHT_TURN;
            enter_state(ROUTE_ALIGN);
            Car_AlignToHold(); /* 倒车后停车保持 800ms+对齐 ideal_yaw+校准，仿进项目 */
            DebugLog_Write(LOG_INFO, "REV_STOP");
        }
        return; /* 跳过 step->timeout_ms 检查，仿 ROUTE_FINAL_FORWARD */

    case ROUTE_FINAL_RIGHT_TURN: //倒车后右转 90°，完成接 S1 直行
        if (g_mode_fsm.turn_count != s_turn_count_before) {
            /* 右转完成不停车，进 S1：以 PWM500 保持航向直行 3.5 秒，再切循迹找十字路口。 */
            enter_state(ROUTE_S1_IMU);
            Car_ImuForward(ROUTE_S1_PWM);
            DebugLog_Write(LOG_INFO, "S1_FWD");
        }
        return;

    case ROUTE_S1_IMU: //高台右转后以 PWM500 保持航向直行 3.5 秒
        if (elapsed_ms >= ROUTE_S1_IMU_MS) {
            if (g_route.step_index + 1u >= ROUTE_ACTIVE_STEP_COUNT) {
                /* 截止在第 9 项时仍完整执行离台动作，到此持续循迹，不进入下一表项。 */
                enter_state(ROUTE_CRUISE);
                reset_junction();
                Car_Track();
                DebugLog_Write(LOG_INFO, "DEBUG_END");
            } else {
                /* 离台完成立即交回普通路线表，由第 10 项统一检测并处理第一个十字。
                 * begin_step 清除旧路口确认状态，并在 IMU→TRACK 时只提交一次循迹请求。 */
                finish_step();
            }
            DebugLog_Write(LOG_INFO, "S1_TRACK");
        }
        return;

    case ROUTE_RECOVER:
        if (confirm_exit(sensor_data)) {
            if (g_route.step_index + 1u >= ROUTE_ACTIVE_STEP_COUNT) {
                /* 循迹左弯也可作为调试截止项：确认出弯后持续循迹，不进入截止范围外的表项。 */
                enter_state(ROUTE_CRUISE);
                reset_junction();
                DebugLog_Write(LOG_INFO, "DEBUG_END");
            } else {
                finish_step();
            }
            return;
        }
        break;
    default:
        break;
    }

    if (elapsed_ms >= step->timeout_ms) {
        /* 不先清阶段或计时，运动层捕获的故障现场仍指向发生超时的位置。 */
        s_active = g_route.enabled = 0u;
        update_route_beep(1u);
        Car_RouteTimeout();
        set_track_parameters(0u);
    }
}

/**
 * @brief 在主循环服务一次异步测距，保持超声波驱动的前台调用约定。
 * @note 短临界区串行化阶段判断、结果发布和触发，防止退出阶段后又发出脉冲。
 *       驱动自行限制至少 60 ms 触发间隔；不等待回波，超时只丢弃本次结果。
 */
void Route_ServiceSonar(void)
{
    SR04_Result result;
    uint32_t saved_primask = __get_PRIMASK();
    __disable_irq();
    if (!s_active || !g_route.enabled || g_mode_fsm.status != CAR_RUNNING ||
        (g_route.state != ROUTE_SONAR_FOLLOW && g_route.state != ROUTE_SONAR_OPEN_LOOP))
    {
        s_sonar_pending = 0u;
        s_sonar_valid = 0u;
        __set_PRIMASK(saved_primask);
        return;
    }
    if (s_sonar_pending != 0u)
    {
        Int_SR04_GetResult(&result);
        if (result.status == SR04_OK || result.status == SR04_TIMEOUT)
        {
            s_sonar_pending = 0u;
            if (result.status == SR04_OK)
            {
                s_sonar_distance = result.distance_cm;
                s_sonar_valid = 1u;
            }
        }
    }
    if (s_sonar_pending == 0u && s_sonar_valid == 0u)
        s_sonar_pending = Int_SR04_Trigger();
    __set_PRIMASK(saved_primask);
}

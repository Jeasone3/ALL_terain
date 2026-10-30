/**
 * @file Route.h
 * @brief 分段路线、项目参数和运行进度；路线层只提交动作，不写 PID 或电机。
 */
#ifndef __ROUTE_H__
#define __ROUTE_H__

#include <stdint.h>

typedef enum {
    JUNCTION_NONE = 0,             /* 未发现宽路口图案 */
    JUNCTION_LEFT,                 /* 左侧四路在线，不等同于已经证明有左出口 */
    JUNCTION_RIGHT,                /* 右侧四路在线 */
    JUNCTION_CROSS                 /* 八路在线，同帧判定优先 */
} RouteJunction;

typedef enum {
    ROUTE_FOLLOW = 0,              /* 循迹等待当前段指定类型的新路口 */
    ROUTE_APPROACH,                /* 按标定时间前移到转弯中心 */
    ROUTE_TURN,                    /* 等待运动层完成 90 度转弯 */
    ROUTE_READY,                   /* 已检查标定参数，下一控制帧开始 */
    ROUTE_ALIGN,                   /* 功能切换前原地慢转到 ideal_yaw 对齐理想坐标系 */
    ROUTE_ENTRY,                   /* 入口对正：保持灰度循迹，按 entry_ms 计时 */
    ROUTE_OBSTACLE,                /* 项目通行：保持 IMU 航向，按 pass_ms 计时 */
    ROUTE_RECOVER,                 /* 恢复循迹并再次确认窄黑线，屏蔽重复路口 */
    ROUTE_UNCALIBRATED,            /* 参数未标定，保持停车 */
    ROUTE_ABORTED,                 /* 被接管、关闭或故障打断，不能自动续跑 */
    ROUTE_CRUISE,                  /* 短段调试截止后持续循迹 */
    ROUTE_FINAL_TURN,              /* 窄桥通行后进入高台流程，原地 IMU 左转 90° */
    ROUTE_FINAL_FORWARD,           /* 保持左转后的目标航向，以 500 PWM 直行 3 秒 */
    ROUTE_FINISHED,                /* 完整路线执行完毕，已提交停车且关闭本轮调度 */
    ROUTE_REVERSE,                 /* 大台阶转后保持航向倒退 2 秒，再进入入口循迹 */
    ROUTE_REVERSE_DRIVE,           /* 高台停车校准后保持 ideal_yaw 倒退 3.2 秒 */
    ROUTE_FINAL_RIGHT_TURN,        /* 倒车后停车校准、右转 90°，完成接 S1 直行 */
    ROUTE_S1_IMU,                  /* 高台右转后保持航向直行，再接普通路线表找十字 */
    ROUTE_SONAR_FOLLOW,            /* 第 12 项后循迹并测距，等待距离小于 40 cm */
    ROUTE_SONAR_OPEN_LOOP          /* 固定差速通行，等待距离大于 40 cm */
} RouteState;

typedef enum {
    ROUTE_ACTION_LEFT = 0,         /* 接受指定路口后前移、左转，再入口对正 */
    ROUTE_ACTION_RIGHT,            /* 接受指定路口后前移、右转，再入口对正 */
    ROUTE_ACTION_STRAIGHT,         /* 接受指定路口后继续循迹，直接入口对正 */
    ROUTE_ACTION_RIGHT_TRACK,      /* 前移、右转后恢复循迹，不进入入口与 IMU 通行 */
    ROUTE_ACTION_LEFT_TRACK,       /* 前移、IMU 左转后恢复循迹，不进入入口与 IMU 通行 */
    ROUTE_ACTION_FOLLOW_LEFT,      /* 全程灰度循迹通过左弯，确认居中窄线后接下一项 */
    ROUTE_ACTION_STRAIGHT_TRACK    /* 路口直行后确认窄线接下一项；cal_before 可启用直行前对齐校准 */
} RouteAction;

/**
 * @brief 一段指定路口后的动作参数，时间单位均为毫秒。
 * @note project_id 表示本次路线的段号，不推断图片中的项目编号。
 *       trigger_junction 必须与两帧确认的路口一致；左右单侧要求新事件，十字允许从单侧升级。
 *       entry_ms 全程使用灰度循迹；pass_ms 到时恢复循迹，不等待出口线。
 *       LEFT_TRACK / RIGHT_TRACK 均先对齐、前移、转 90°，再恢复循迹；项目参数均为 0。
 *       FOLLOW_LEFT 全程由循迹 PID 沿左弯行驶，项目参数也为 0，不执行 IMU 对齐与转弯。
 *       STRAIGHT_TRACK 循迹直行，cal_before = 1 时先对齐保持校准，确认离开路口后接下一项。
 *       STRAIGHT_TRACK 项目 PWM、入口与通行时间均为 0，不执行定时项目通行。
 *       LEFT / RIGHT 转后按 entry_ms 循迹；STRAIGHT 接受路口后按 entry_ms 循迹。
 *       STRAIGHT 的 cal_before = 1 时先停车对齐保持并校准，再开始入口循迹计时。
 *       pass_ms 为 0 时只定时循迹；base_pwm 保留非零供参数检查，不控制循迹速度。
 *       第 8 项转后倒退暂时停用，直接入口循迹；具体时间以路线表为准。
 *       非截止段恢复窄线后接下一段；短段调试截止后持续循迹。
 *       final_seq = 1 在通行后接高台及 S1；2 在定时循迹结束后停车；3 在定时 IMU 通行结束后停车。
 *       timeout_ms 限制本段每个等待阶段；最终 CRUISE 不设路线超时。
 */
typedef struct {
    RouteAction action;            /* 路口继续循迹，或先前移转弯 */
    uint8_t project_id;             /* 本次路线段号，从 1 开始 */
    uint16_t base_pwm;              /* 本段 IMU 通行 PWM，不控制转弯前移；纯循迹项仅用于参数检查 */
    uint32_t entry_ms;              /* 入口灰度循迹持续时间；RIGHT_TRACK/LEFT_TRACK 不使用 */
    uint32_t pass_ms;               /* IMU 直行持续时间；RIGHT_TRACK/LEFT_TRACK 不使用，纯循迹项目为 0 */
    uint32_t timeout_ms;            /* 等路口、入口、转弯、恢复的阶段超时 */
    RouteJunction trigger_junction; /* 本段唯一接受的路口类型 */
    uint8_t cal_before;             /* 路口动作前对齐保持校准；两类直行动作校准后切回循迹 */
    uint8_t cal_after;              /* 转后、入口或恢复前 align-hold 校准 */
    uint8_t final_seq;              /* 0=普通；1=OBSTACLE 后接高台；2=ENTRY 到时停车；3=OBSTACLE 到时停车 */
} RouteStep;

#define ROUTE_STEP_COUNT 20u       /* 第 17 项后第二段超声波通行，再接第 18～20 项左、右、右转。 */

/* 本次从路线表第几项开始，使用 1 至 ROUTE_STEP_COUNT 的表项序号，不按 project_id 查找。
 * 例如设为 9u：从窄桥触发路口前循迹出发，完成高台及 S1 后继续执行后面的表项。
 * 设为 1u 从头运行；修改后重新编译烧录，并将车辆放在所选项目的触发路口前。 */
#ifndef ROUTE_START_STEP
#define ROUTE_START_STEP 10u
#endif

/* 调试截止表项：20 或 0 执行全部路线，第 20 项十字右转、循迹 1 秒、对齐后 IMU 直行 5 秒停车。
 * 截止为 12 时完成超声波一次通行后持续循迹，不进入第 13 项。
 * 第 9 项的高台及 S1 属于该项自身的后续流程，截止为 9 时仍会执行，随后持续循迹。
 * 截止为 17 时完成第二段超声波通行后持续循迹，不进入第 18 项。
 * 当前设为 20：执行全部已编写路线；截止为 10 或 11 时对应右转完成后持续循迹。
 * 启用后续项时，第 10、11 项须先连续两帧确认居中窄线，才接收下一路口。
 * 允许宿主测试通过编译参数覆盖，实车默认值仍在此处维护。 */
#ifndef ROUTE_DEBUG_STEP_LIMIT
#define ROUTE_DEBUG_STEP_LIMIT 20u
#endif

#if ROUTE_DEBUG_STEP_LIMIT > ROUTE_STEP_COUNT
#error ROUTE_DEBUG_STEP_LIMIT_exceeds_defined_route_steps
#endif

#if ROUTE_START_STEP < 1u || ROUTE_START_STEP > ROUTE_STEP_COUNT
#error ROUTE_START_STEP_outside_defined_route_steps
#endif

#if ROUTE_DEBUG_STEP_LIMIT != 0u && ROUTE_START_STEP > ROUTE_DEBUG_STEP_LIMIT
#error ROUTE_START_STEP_exceeds_debug_step_limit
#endif

/* 只在停车且 Route 未运行时修改；实际部署编辑 Route.c 后重新编译。
 * entry_ms、pass_ms 和 base_pwm 均在表内维护；路口转弯前移时间与 PWM 由 Route.c 统一设置。 */
extern RouteStep g_route_plan[ROUTE_STEP_COUNT];

typedef struct {
    RouteState state;              /* 当前路线执行阶段 */
    RouteJunction junction;        /* 两帧确认的传感器路口图案 */
    uint8_t junction_event;        /* 单帧新路口事件，项目内部不产生 */
    uint8_t line_found;            /* 任意一路在线的两帧稳定值 */
    uint8_t line_found_event;      /* 本帧稳定有线事件 */
    uint8_t line_lost_event;       /* 本帧稳定无线事件 */
    uint8_t enabled;               /* 路线许可；关闭不直接改电机 */
    uint32_t state_frames;         /* 当前阶段已完成的 10 ms 控制帧数 */
    uint16_t step_index;           /* 当前表项，内部从 0 开始，屏幕从 1 显示 */
    uint8_t project_id;            /* 当前路线段号，与表项中的 project_id 一致 */
    float ideal_yaw;               /* 绝对理想朝向(90°倍数)：IMU 动作目标参考，偏差不累积 */
} Route_t;

extern volatile Route_t g_route;

/** @brief 初始化检测与进度，默认不启动路线；无参数、无返回，不发送运动请求。 */
void Route_Init(void);

/**
 * @brief 检查本次调试范围内的参数是否完整有效，不启动电机。
 * @return 1：启用段全部有效；0：至少一个时间缺失或参数不合法。
 * @note 不推测实际距离，不替代实车标定和路口事件顺序验证。
 */
uint8_t Route_IsCalibrated(void);

/**
 * @brief 停车状态下从 ROUTE_START_STEP 指定的表项开始，随后按原顺序执行。
 * @return 1：已准备，下一控制帧开始；0：未标定或车辆尚未停车。
 * @note 被打断后须将车放回所选项目的触发路口前，再显式调用；Enable(1) 不恢复进度。
 */
uint8_t Route_Start(void);

/**
 * @brief 设置自动路线许可，监测始终保留。
 * @param enabled 0 关闭；非零允许已经由 Route_Start 启动的路线。
 * @note 关闭会中止正在运行的路线，但不直接停车；需要停车时调用 Car_Stop。
 */
void Route_Enable(uint8_t enabled);

/**
 * @brief 每 10 ms 更新灰度并执行当前路线阶段，只在动作切换时发一次请求。
 * @param sensor_data 八路 IN1 至 IN8 的本帧采样；空指针只清事件。
 * @param external_command 本帧已接受外部请求时为 1，路线让出控制权并中止。
 * @note 仅由 ModeFSM_Tick 调用；普通监测可在手动模式继续，项目内部屏蔽路口。
 */
void Route_Tick(const uint16_t *sensor_data, uint8_t external_command);

/** @brief 前台调用；仅在超声波路线阶段触发测距并发布新结果，不输出电机 PWM。 */
void Route_ServiceSonar(void);

#endif

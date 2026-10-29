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
    ROUTE_FINAL_TURN,              /* 完整路线最后一个项目结束后，原地 IMU 左转 90° */
    ROUTE_FINAL_FORWARD,           /* 保持左转后的目标航向，以 500 PWM 直行 3 秒 */
    ROUTE_FINISHED,                /* 完整路线执行完毕，已提交停车且关闭本轮调度 */
    ROUTE_REVERSE                  /* 大台阶转后保持航向倒退 2 秒，再进入入口循迹 */
} RouteState;

typedef enum {
    ROUTE_ACTION_LEFT = 0,         /* 接受指定路口后前移、左转，再入口对正 */
    ROUTE_ACTION_RIGHT,            /* 接受指定路口后前移、右转，再入口对正 */
    ROUTE_ACTION_STRAIGHT,         /* 接受指定路口后继续循迹，直接入口对正 */
    ROUTE_ACTION_RIGHT_TRACK       /* 前移、右转后恢复循迹，不进入入口与 IMU 通行 */
} RouteAction;

/**
 * @brief 一段指定路口后的动作参数，时间单位均为毫秒。
 * @note project_id 表示本次路线的段号，不推断图片中的项目编号。
 *       trigger_junction 必须与两帧确认的路口一致；左右单侧要求新事件，十字允许从单侧升级。
 *       entry_ms 全程使用灰度循迹；pass_ms 到时恢复循迹，不等待出口线。
 *       RIGHT_TRACK 只前移、右转后循迹，base_pwm、entry_ms、pass_ms 均为 0。
 *       第 8 项左转后由状态机单独判断，先以 300 PWM 倒退 2 秒，再按 entry_ms 循迹。
 *       非截止段恢复窄线后接下一段；短段调试截止后持续循迹。
 *       完整路线末尾追加原地左转 90°、IMU 直行 3 秒，再停车。
 *       timeout_ms 限制本段每个等待阶段；最终 CRUISE 不设路线超时。
 */
typedef struct {
    RouteAction action;            /* 路口继续循迹，或先前移转弯 */
    uint8_t project_id;             /* 本次路线段号，从 1 开始 */
    uint16_t base_pwm;              /* 本段 IMU 通行基础 PWM；RIGHT_TRACK 不使用 */
    uint32_t entry_ms;              /* 入口灰度循迹持续时间；RIGHT_TRACK 不使用 */
    uint32_t pass_ms;               /* IMU 直行持续时间；RIGHT_TRACK 不使用 */
    uint32_t timeout_ms;            /* 等路口、入口、转弯、恢复的阶段超时 */
    RouteJunction trigger_junction; /* 本段唯一接受的路口类型 */
} RouteStep;

#define ROUTE_STEP_COUNT 9u        /* 当前已经按新顺序编写的段数；后续追加表项时同步增加 */

/* 本次从路线表第几项开始，使用 1 至 9 的表项序号，不按 project_id 查找。
 * 例如设为 6u：从第 6 项对应路口前循迹出发，顺序执行第 6、7、8、9 项及原有收尾。
 * 默认 1u 保持从头运行；修改后重新编译烧录，并将车辆放在所选项目的触发路口前。 */
#ifndef ROUTE_START_STEP
#define ROUTE_START_STEP 8u
#endif

/* 集中设置本次调试截止段数：9 或 0 均执行到路线末尾，再左转、直行 3 秒停车。
 * 1 至 8 只运行起始项至截止项，截止后持续循迹，不执行完整路线的收尾动作。
 * 允许宿主测试通过编译参数覆盖，实车默认值仍在此处维护。 */
#ifndef ROUTE_DEBUG_STEP_LIMIT
#define ROUTE_DEBUG_STEP_LIMIT 9u
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
 * 入口 1000/2000 ms、通行 4000/5000/6000 ms 来自用户要求；PWM、前移与图案需实车标定。 */
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

#endif

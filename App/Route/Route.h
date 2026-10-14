/**
 * @file Route.h
 * @brief 固定路线、项目参数和运行进度；路线层只提交动作，不写 PID 或电机。
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
    ROUTE_FOLLOW = 0,              /* 循迹等待当前表项对应的路口 */
    ROUTE_APPROACH,                /* 按标定时间前移到转弯中心 */
    ROUTE_TURN,                    /* 等待运动层完成 90 度转弯 */
    ROUTE_CROSSING,                /* 保持航向，确认离开当前路口 */
    ROUTE_READY,                   /* 已检查标定参数，下一控制帧开始 */
    ROUTE_ENTRY,                   /* 前一路口完成后循迹接近项目入口 */
    ROUTE_OBSTACLE,                /* 项目通行：IMU 直行或 U 型隧道循迹 */
    ROUTE_RECOVER,                 /* 恢复循迹并再次确认窄黑线，屏蔽重复路口 */
    ROUTE_UNCALIBRATED,            /* 参数未标定，保持停车 */
    ROUTE_ABORTED,                 /* 被接管、关闭或故障打断，不能自动续跑 */
    ROUTE_FINISHED                 /* 已在项目 9 定时停车 */
} RouteState;

typedef enum {
    ROUTE_ACTION_LEFT = 0,         /* 接受路口后前移并左转 */
    ROUTE_ACTION_RIGHT,            /* 接受路口后前移并右转 */
    ROUTE_ACTION_STRAIGHT,         /* 穿过路口，离开后及时恢复循迹 */
    ROUTE_ACTION_IMU_PASS,         /* 无黑线的直通项目 */
    ROUTE_ACTION_TRACK_PASS,       /* U 型隧道，项目内部仍循迹 */
    ROUTE_ACTION_TIMED_STOP        /* 项目 9，计时登台后结束整条路线 */
} RouteAction;

/**
 * @brief 一个道路动作或项目通行表项，时间单位均为毫秒。
 * @note project_id=0 表示道路动作，1 至 9 对应图中的项目。
 *       道路 entry_ms 是接受路口前的最短等待，可为 0；pass_ms 是前移/最短穿越时间。
 *       项目 entry_ms 是前一步完成后的入口延时；pass_ms 是最短通行或最终登台时长。
 *       timeout_ms 是该表项每个等待阶段的上限，必须大于 pass_ms 和 entry_ms。
 */
typedef struct {
    RouteAction action;            /* 本次完成什么动作 */
    uint8_t project_id;             /* 图中项目编号；石块会出现两次 */
    uint16_t base_pwm;              /* IMU 直行基础 PWM，循迹表项固定 400 */
    uint32_t entry_ms;              /* 入口延时或路口识别时间窗起点 */
    uint32_t pass_ms;               /* 前移、最短越障或最终停车时间 */
    uint32_t timeout_ms;            /* 等路口、通行、出口确认的阶段超时 */
} RouteStep;

#define ROUTE_STEP_COUNT 24u

/* 只在停车且 Route 未运行时修改时间；实际部署建议编辑 Route.c 后重新编译。
 * 默认时间为 0，表示未标定；行为、项目编号和 PWM 已按确认路线填写。 */
extern RouteStep g_route_plan[ROUTE_STEP_COUNT];

typedef struct {
    RouteState state;              /* 当前路线执行阶段 */
    RouteJunction junction;        /* 三帧确认的传感器路口图案 */
    uint8_t junction_event;        /* 单帧新路口事件，项目内部不产生 */
    uint8_t line_found;            /* 任意一路在线的三帧稳定值 */
    uint8_t line_found_event;      /* 本帧稳定有线事件 */
    uint8_t line_lost_event;       /* 本帧稳定无线事件 */
    uint8_t enabled;               /* 路线许可；关闭不直接改电机 */
    uint32_t state_frames;         /* 当前阶段已完成的 10 ms 控制帧数 */
    uint16_t step_index;           /* 当前表项，内部从 0 开始，屏幕从 1 显示 */
    uint8_t project_id;            /* 当前项目编号，0 为道路 */
} Route_t;

extern volatile Route_t g_route;

/** @brief 初始化检测与进度，默认不启动路线；无参数、无返回，不发送运动请求。 */
void Route_Init(void);

/**
 * @brief 检查整张表是否已填入有效标定时间，不启动电机。
 * @return 1：全部有效；0：至少一个时间未标定或参数不合法。
 * @note 不推测实际距离，不替代实车标定和路口事件顺序验证。
 */
uint8_t Route_IsCalibrated(void);

/**
 * @brief 停车状态下从起点重新开始整条路线。
 * @return 1：已准备，下一控制帧开始；0：未标定或车辆尚未停车。
 * @note 被打断后必须将车辆放回起点再显式调用；单纯 Enable(1) 不恢复进度。
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

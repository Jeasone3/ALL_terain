/**
 * @file Route.h
 * @brief 小车上层路线接口：观察条件并选择动作，不参与 PID 和电机输出。
 * @note 灰度检测在所有运动模式下更新；关闭自动路线后仍可读取检测结果。
 */
#ifndef __ROUTE_H__
#define __ROUTE_H__

#include <stdint.h>

typedef enum {
    JUNCTION_NONE = 0,             /* 没有识别到路口 */
    JUNCTION_LEFT,                 /* 左侧四路同时在线 */
    JUNCTION_RIGHT,                /* 右侧四路同时在线 */
    JUNCTION_CROSS                 /* 八路同时在线，判定优先级最高 */
} RouteJunction;

typedef enum {
    ROUTE_FOLLOW = 0,              /* 等待循迹期间出现路线条件 */
    ROUTE_APPROACH,                /* 用 IMU 保持航向前进到转弯位置 */
    ROUTE_TURN,                    /* 等待已经发起的 IMU 转弯完成 */
    ROUTE_CROSSING                 /* 用 IMU 保持航向穿过十字路口 */
} RouteState;

typedef struct {
    RouteState state;              /* 当前示例路线步骤，不代表运动控制模式 */
    RouteJunction junction;        /* 十字两帧、左右三帧确认后的路口类型 */
    uint8_t junction_event;        /* 本帧新路口事件，持续一帧 */
    uint8_t line_found;            /* 任意通道在线，连续三帧确认后的稳定值 */
    uint8_t line_found_event;      /* 本帧由无线变为有线的事件 */
    uint8_t line_lost_event;       /* 本帧由有线变为无线的事件 */
    uint8_t enabled;               /* 1：启用示例决策；0：只监测条件 */
    uint32_t state_frames;         /* 当前步骤帧数；运动步骤跟随运动层计数 */
} Route_t;

/* TIM4 内更新。外部只通过 Route_Enable 修改开关，不直接修改其他字段。 */
extern volatile Route_t g_route;

/**
 * @brief 初始化路口监测和示例路线，默认开启自动条件决策。
 * @param 无。
 * @return 无。
 * @note 在启动 TIM4 前调用；不发送动作、不复位 PID，也不驱动电机。
 */
void Route_Init(void);

/**
 * @brief 开启或关闭示例路线决策，保留当前运动动作和灰度监测。
 * @param enabled 0：关闭自动决策；非零：开启自动决策。
 * @return 无。
 * @note 关闭后下一控制帧取消未完成路线步骤，但不自动停车或切回循迹。
 *       若需要立即停车，请另外调用 Car_Stop；重新开启也不会立即切换模式。
 */
void Route_Enable(uint8_t enabled);

/**
 * @brief 更新灰度条件，并按当前路线步骤提交下一运动请求。
 * @param sensor_data 已采样的八路灰度数组，顺序为 IN1 至 IN8；空指针跳过本帧检测与决策。
 * @param external_command 1：运动层本帧已经接受外部请求；0：允许示例路线提交动作。
 * @return 无。
 * @note 仅由 ModeFSM_Tick 每 10 ms 调用一次；所有事件只保持当前帧。
 *       外部请求取消路线步骤且当帧禁止自动覆盖。停车、故障和关闭路线时仍更新灰度条件。
 */
void Route_Tick(const uint16_t *sensor_data, uint8_t external_command);

#endif /* __ROUTE_H__ */

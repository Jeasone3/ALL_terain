/**
 * @file Mode_FSM.h
 * @brief 小车运动执行接口：上层选择动作，10 ms 状态机独占运动控制。
 * @note Car_* 只提交请求，不在调用者上下文读取 IMU 或驱动电机。
 */
#ifndef __MODE_FSM_H__
#define __MODE_FSM_H__

#include "main.h"
#include <stdint.h>

/* 两个大模式；停车和故障由运行状态表示，不增加第三个运动模式。 */
typedef enum {
    MODE_TRACK = 0,                 /* 灰度循迹 PID 控制电机 */
    MODE_IMU                       /* IMU 角度 PID 控制电机 */
} CarMode;

typedef enum {
    IMU_FORWARD = 0,               /* 保持目标航向直行 */
    IMU_TURN_LEFT,                 /* 相对动作起点左转 90° */
    IMU_TURN_RIGHT                 /* 相对动作起点右转 90° */
} ImuState;

typedef enum {
    CAR_RUNNING = 0,
    CAR_STOPPED,
    CAR_FAULT
} CarStatus;

typedef enum {
    CAR_FAULT_NONE = 0,
    CAR_FAULT_IMU,                 /* IMU 不可用、读取失败或角度无效 */
    CAR_FAULT_TURN_TIMEOUT,        /* 转弯 3 秒内未完成 */
    CAR_FAULT_COMMAND              /* 转后模式参数不合法 */
} CarFault;

typedef struct {
    CarMode mode;                  /* 当前大模式 */
    ImuState imu_state;             /* IMU 子状态；循迹时不使用 */
    CarStatus status;               /* 运行、停车或故障 */
    CarFault fault;                 /* 故障原因，显式新动作可重新尝试 */
    CarMode after_turn_mode;        /* 开始转弯时指定的后续模式 */
    float target_yaw;               /* 相对动作起点的目标角度，单位：度 */
    uint32_t state_frames;          /* 当前动作已执行帧数，每帧 10 ms */
    uint32_t turn_count;            /* 已完成转弯总数，上层比较前后值获取完成事件 */
    uint32_t command_id;            /* 已接受指令序号，用于调试动作切换 */
    uint8_t imu_valid;              /* 1：本控制帧取得有限 IMU 角度；循迹、停止或读取失败为 0 */
} ModeFSM_t;

/* 中断内维护，外部读取请用 ModeFSM_GetSnapshot，禁止直接修改字段。 */
extern volatile ModeFSM_t g_mode_fsm;

/**
 * @brief 初始化运动状态和示例 Route，默认进入循迹模式。
 * @param 无。
 * @return 无。
 * @note 在启动 TIM4 前调用；不启动定时器，不直接输出电机 PWM。
 */
void ModeFSM_Init(void);

/**
 * @brief 执行一帧采样、条件判断、请求处理和当前模式的运动控制。
 * @param 无。
 * @return 无。
 * @note 仅由 TIM4 每 10 ms 调用一次；禁止主循环重复调用。
 */
void ModeFSM_Tick(void);

/**
 * @brief 提交循迹请求，由灰度传感器和循迹 PID 自动跟线。
 * @param 无。
 * @return 无。
 * @note 下一控制帧生效，暂停 IMU 采样与解算；路口去向由 Route 决定。
 */
void Car_Track(void);

/**
 * @brief 提交 IMU 直行请求，以接受动作时的朝向为零度基准。
 * @param 无。
 * @return 无。
 * @note 持续运行到下一请求或故障，灰度只供条件检测，不运行循迹 PID。
 */
void Car_ImuForward(void);

/**
 * @brief 提交相对动作起点左转 90°的请求，同时选择转后模式。
 * @param next_mode MODE_TRACK：转后循迹；MODE_IMU：保持转弯目标直行。
 * @return 无。
 * @note 直接原地转弯，不内置路口前移；前移由 Route 单独安排。
 */
void Car_TurnLeft90(CarMode next_mode);

/**
 * @brief 提交相对动作起点右转 90°的请求，同时选择转后模式。
 * @param next_mode MODE_TRACK：转后循迹；MODE_IMU：保持转弯目标直行。
 * @return 无。
 * @note 直接原地转弯；非法 next_mode 会进入指令故障并停车。
 */
void Car_TurnRight90(CarMode next_mode);

/**
 * @brief 提交停车请求，下一控制帧清 PID 并将两轮输出置零。
 * @param 无。
 * @return 无。
 * @note 同一帧内停车优先于其他待处理请求；停车后须显式发送新动作。
 */
void Car_Stop(void);

/**
 * @brief 取得一致的运动状态副本，供主循环显示或上层读取。
 * @param snapshot 接收副本的指针；传入空指针时不执行复制。
 * @return 无。
 * @note 复制期间短暂屏蔽中断并恢复原状态；格式化和 OLED 通信应在复制后进行。
 */
void ModeFSM_GetSnapshot(ModeFSM_t *snapshot);

#endif /* __MODE_FSM_H__ */

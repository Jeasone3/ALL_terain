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

/* IMU 子状态，用于 MODE_IMU 模式。 */
typedef enum {
    IMU_FORWARD = 0,               /* 保持目标航向直行 */
    IMU_TURN_LEFT,                 /* 相对动作起点左转 90° */
    IMU_TURN_RIGHT                 /* 相对动作起点右转 90° */
} ImuState;

/* 运行状态同时适用于循迹和 IMU 模式。 */
typedef enum {
    CAR_RUNNING = 0,
    CAR_STOPPED,
    CAR_FAULT
} CarStatus;

typedef enum {
    CAR_FAULT_NONE = 0,
    CAR_FAULT_IMU,                 /* IMU 不可用、读取失败或角度无效 */
    CAR_FAULT_TURN_TIMEOUT,        /* 转弯 5 秒内未进容差，或截止确认期间离开容差 */
    CAR_FAULT_COMMAND,             /* 转后模式或直行 PWM 参数不合法 */
    CAR_FAULT_ROUTE_TIMEOUT,       /* 路口等待、项目通行或出口确认超时 */
    CAR_FAULT_CONTROL_PERIOD       /* TIM4 通知积压或控制帧执行达到 10 ms */
} CarFault;

//状态机对象，用于保存当前状态
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
    uint8_t imu_valid;              /* 1：本帧角度有效；0：循迹、停车或读取失败 */
    uint16_t base_pwm;              /* 当前 IMU 直行基础 PWM，与动作一起接受，不代表实测车速 */
} ModeFSM_t;

/* 由 ControlTask 维护，其他任务读取请用 ModeFSM_GetSnapshot，禁止直接修改字段。 */
extern volatile ModeFSM_t g_mode_fsm;

/**
 * @brief 初始化运动状态和固定 Route，默认停车，选择测试或有效路线后才开始运行。
 * @param 无。
 * @return 无。
 * @note 在启动 TIM4 前调用；不启动定时器，不直接输出电机 PWM。
 */
void ModeFSM_Init(void);

/**
 * @brief 执行一帧采样、条件判断、请求处理和当前模式的运动控制。
 * @param 无。
 * @return 无。
 * @note 仅由 ControlTask 接收 TIM4 通知后每 10 ms 调用一次；禁止中断重复调用。
 */
void ModeFSM_Tick(void);

/**
 * @brief 控制任务发现节拍失效时，保存现场、取消旧请求并立即停车。
 * @param 无。
 * @return 无。
 * @note 仅由 ControlTask 调用；中止自动路线，后续必须显式提交新动作。
 *       不补跑丢失的控制帧，故障发生前等待的运动请求不能在下一帧自动重启。
 */
void ModeFSM_ControlPeriodFault(void);

//-----------------------------------------------Car_* 接口：提交请求，下一控制帧生效。---------------------------------/

/**
 * @brief 提交循迹请求，由灰度传感器和循迹 PID 自动跟线。
 * @param 无。
 * @return 无。
 * @note 下一控制帧生效，暂停 IMU 采样与解算；路口去向由 Route 决定。
 */
void Car_Track(void);

/**
 * @brief 提交 IMU 直行请求，以接受动作时的朝向为零度基准。
 * @param base_pwm 基础 PWM，范围 1 至 1000；普通项目先用 500，台阶和斜坡先用 600。
 * @return 无。
 * @note 动作与 PWM 原子提交；持续运行到下一请求或故障，灰度不参与直行纠偏。
 */
void Car_ImuForward(uint16_t base_pwm);

/**
 * @brief 单项目标定用定时直行，从接受动作开始计算，时间到后在控制任务内停车。
 * @param base_pwm 基础 PWM，范围 1 至 1000。
 * @param duration_ms 非零测试时长，向上取整到 10 ms。
 * @return 无。
 * @note OLED 或前台通信阻塞不会延长此测试；不用于自动路线的项目出口判断。
 */
void Car_ImuForwardTimed(uint16_t base_pwm, uint32_t duration_ms);

/**
 * @brief 提交路线超时故障，由唯一运动入口锁存现场并停车。
 * @param 无。
 * @return 无。
 * @note 仅供 Route 的超时分支调用；不在调用者上下文写电机，不能覆盖已等待的 STOP。
 */
void Car_RouteTimeout(void);

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


//-----------------------------------------------状态机快照接口：短暂屏蔽中断，保证一致性。----------------------------/
/**
 * @brief 取得一致的运动状态副本，供其他任务显示或上层读取。
 * @param snapshot 接收副本的指针；传入空指针时不执行复制。
 * @return 无。
 * @note 复制期间短暂屏蔽中断并恢复原状态；格式化和 OLED 通信应在复制后进行。
 */
void ModeFSM_GetSnapshot(ModeFSM_t *snapshot);

#endif /* __MODE_FSM_H__ */

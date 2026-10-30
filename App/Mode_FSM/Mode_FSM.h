/**
 * @file Mode_FSM.h
 * @brief 小车运动执行接口：上层选择动作，10 ms 状态机独占运动控制。
 * @note Car_* 只提交请求，不在调用者上下文读取 IMU 或驱动电机。
 */
#ifndef __MODE_FSM_H__
#define __MODE_FSM_H__

#include "main.h"
#include <stdint.h>

/* 运动模式互斥；停车和故障由独立运行状态表示。 */
typedef enum {
    MODE_TRACK = 0,                 /* 灰度循迹 PID 控制电机 */
    MODE_IMU,                      /* IMU 角度 PID 控制电机 */
    MODE_OPEN_LOOP                 /* 超声波通行：固定两轮 PWM，不读取 IMU 或运行循迹 PID */
} CarMode;

/* IMU 子状态，用于 MODE_IMU 模式。 */
typedef enum {
    IMU_FORWARD = 0,               /* 保持目标航向直行 */
    IMU_TURN_LEFT,                 /* 相对动作起点左转 90° */
    IMU_TURN_RIGHT,                /* 相对动作起点右转 90° */
    IMU_REVERSE,                   /* 保持目标航向倒退，两轮 PWM 均不大于零 */
    IMU_ALIGN                      /* 功能切换时原地慢转到 g_route.ideal_yaw 对齐理想坐标系 */
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
    CAR_FAULT_ROUTE_TIMEOUT        /* 路口等待、项目通行或出口确认超时 */
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
    uint16_t base_pwm;              /* 当前 IMU 前进或倒退的基础 PWM 幅度，与动作一起接受，不代表实测车速 */
    uint32_t align_count;           /* 已完成对齐总数，Route 比较前后值获取对齐完成事件，仿 turn_count */
} ModeFSM_t;

/* 中断内维护，外部读取请用 ModeFSM_GetSnapshot，禁止直接修改字段。 */
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
 * @note 仅由 TIM4 每 10 ms 调用一次；禁止主循环重复调用。
 */
void ModeFSM_Tick(void);

//-----------------------------------------------Car_* 接口：提交请求，下一控制帧生效。---------------------------------/

/**
 * @brief 提交循迹请求，由灰度传感器和循迹 PID 自动跟线。
 * @param 无。
 * @return 无。
 * @note 下一控制帧生效，暂停 IMU 采样与解算；路口去向由 Route 决定。
 */
void Car_Track(void);

/** @brief 请求第一段超声波通行的固定开环调参，下一控制帧生效。 */
void Car_OpenLoop(void);

/** @brief 原子提交两轮开环前进 PWM，范围均为 1～1000；下一控制帧生效。 */
void Car_OpenLoopWithPWM(uint16_t left_pwm, uint16_t right_pwm);

/**
 * @brief 提交 IMU 直行请求，以接受动作时的朝向为零度基准。
 * @param base_pwm 基础 PWM，范围 1 至 1000；普通项目先用 500，台阶和斜坡先用 600。
 * @return 无。
 * @note 动作与 PWM 原子提交；持续运行到下一请求或故障，灰度不参与直行纠偏。
 */
void Car_ImuForward(uint16_t base_pwm);

/**
 * @brief 提交保持当前航向倒退请求，目标取 g_route.ideal_yaw，持续到下一请求。
 * @param base_pwm 倒退基础 PWM，范围 1 至 1000；调度层内部取负，yaw 纠偏方向不变。
 * @return 无。
 * @note 下一帧生效；持续运行到下一请求或故障，结束条件由上层(Route 数帧)决定。
 */
void Car_Reverse(uint16_t base_pwm);

/**
 * @brief 单项目标定用定时直行，从接受动作开始计算，时间到后在控制中断内停车。
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
 * @brief 提交带局部容差的闭环左转 90°，不修改全局转弯参数。
 * @param next_mode 转完后的模式，与 Car_TurnLeft90 相同。
 * @param tolerance_deg 本次容差，单位度，含边界；0 使用全局值，有效范围为 [0, 90)。
 * @return 无；动作与容差一起提交，下一动作自动恢复默认判据。
 */
void Car_TurnLeft90WithTolerance(CarMode next_mode, float tolerance_deg);

/**
 * @brief 提交相对动作起点右转 90°的请求，同时选择转后模式。
 * @param next_mode MODE_TRACK：转后循迹；MODE_IMU：保持转弯目标直行。
 * @return 无。
 * @note 直接原地转弯；非法 next_mode 会进入指令故障并停车。
 */
void Car_TurnRight90(CarMode next_mode);

/**
 * @brief 请求原地对齐到 g_route.ideal_yaw（90° 倍数），完成后停车等 Route 衔接。
 * @param 无。
 * @return 无。
 * @note 功能切换时调用；目标直接取 g_route.ideal_yaw，不重建零度基准；到位后 align_count++。
 */
void Car_AlignTo(void);

/**
 * @brief 请求原地对齐到 g_route.ideal_yaw，到位后原地保持 800ms 再停车等 Route 衔接。
 * @param 无。
 * @return 无。
 * @note 进项目切换（循迹对正→IMU 项目通行）时调用；比 Car_AlignTo 多一段到位保持，
 *       等待角度稳定对齐项目后再启动陀螺仪。路口转弯前前移仍用 Car_AlignTo。
 */
void Car_AlignToHold(void);

/**
 * @brief 提交停车请求，下一控制帧清 PID 并将两轮输出置零。
 * @param 无。
 * @return 无。
 * @note 同一帧内停车优先于其他待处理请求；停车后须显式发送新动作。
 */
void Car_Stop(void);


//-----------------------------------------------状态机快照接口：短暂屏蔽中断，保证一致性。----------------------------/
/**
 * @brief 取得一致的运动状态副本，供主循环显示或上层读取。
 * @param snapshot 接收副本的指针；传入空指针时不执行复制。
 * @return 无。
 * @note 复制期间短暂屏蔽中断并恢复原状态；格式化和 OLED 通信应在复制后进行。
 */
void ModeFSM_GetSnapshot(ModeFSM_t *snapshot);

#endif /* __MODE_FSM_H__ */

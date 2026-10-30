/**
 * @file Mode_FSM.c
 * @brief 两层运动状态机：统一处理请求，循迹和 IMU 控制互斥运行。
 * @note TIM4 是唯一调度入口；Route 只选择动作，不直接操作 PID 和电机。
 */
#include "Mode_FSM.h"
#include "Route.h"
#include "Track_Task.h"
#include "Int_Track.h"
#include "Int_MPU6050.h"
#include "Attitude.h"
#include "IMU_Task.h"
#include "Debug_Log.h"
#include "motor.h"
#include <math.h>

#define IMU_FAILURE_FRAMES 3u   /* 连续三帧两次采样均失败才锁存通信故障，暂态失败帧输出零 PWM。 */

#define TURN_DONE_FRAMES 2u      /* 误差连续两帧在容差内才算完成 */
#define ALIGN_DONE_FRAMES 10u   /* 对齐要求连续 10 帧(100ms)在容差内才完成，避免过冲振荡时偶然判完成 */
#define ALIGN_HOLD_FRAMES 80u   /* 进项目对齐到位后原地保持 80 帧(800ms) 再切 IMU 直行，等待角度稳定对齐项目 */
#define TURN_TIMEOUT_FRAMES 500u /* 最多等待 5 秒；提前转完立即衔接，截止时已进容差最多补一帧 */

/* 本帧八路灰度数据；循迹用于控制，IMU 模式用于路线条件检测。 */
extern uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];
/* 由 TIM4 维护的当前运动状态。 */
volatile ModeFSM_t g_mode_fsm;
extern Motor_Struct motorLeft, motorRight;

#define OPEN_LOOP_LEFT_PWM  560u /* 超声波通行时左轮固定前进 PWM。 */
#define OPEN_LOOP_RIGHT_PWM 300u /* 超声波通行时右轮固定前进 PWM。 */

/* 邮箱中的动作类型，与实际执行状态分开保存。 */
typedef enum
{
    COMMAND_TRACK = 0,              //循迹态
    COMMAND_OPEN_LOOP,             /* 超声波阶段固定差速，不执行姿态或循迹闭环。 */
    COMMAND_IMU_FORWARD,            //IMU 直行态
    COMMAND_IMU_TIMED,              //转向前移
    COMMAND_LEFT,                   //左转
    COMMAND_RIGHT,              //右转
    COMMAND_STOP,                   //停车  
    COMMAND_ROUTE_TIMEOUT,       //路线超时停车
    COMMAND_ALIGN,              //功能切换时原地对齐到 ideal_yaw
    COMMAND_ALIGN_HOLD,         //进项目对齐：到位后原地保持 800ms 再切 IMU 直行
    COMMAND_REVERSE             //IMU 倒退：保持目标航向 ideal_yaw，两轮 PWM 不大于零
} CarCommand;



/* pending 表示邮箱中有尚未接受的请求；next_mode 只在转弯时使用。 */
typedef struct
{
    CarCommand command; // 要执行什么动作
    CarMode next_mode;  // 转弯完成后的模式
    uint16_t base_pwm;  /* IMU 直行基础占空比，与 command 在同一临界区复制 */
    uint16_t right_pwm; /* 开环右轮 PWM；开环左轮使用 base_pwm，两轮随同一动作提交。 */
    uint32_t duration_ms; /* 单项目测试限时，普通动作不使用 */
    float turn_tolerance; /* 本次转弯的局部容差；0 使用全局值，与动作一起提交 */
    uint8_t pending;    // 请求标志
} CarRequest;

/* 单个请求邮箱：正常请求最后一个生效，同帧停车请求优先。 存储外部命令信息 */
static volatile CarRequest s_request;
static uint8_t s_imu_failed_frames; /* 连续无有效采样的帧数；一次成功立即清零。 */
static uint8_t s_turn_done_frames; // 转弯完成帧数
static uint8_t s_align_done_frames; // 对齐完成帧数
static uint8_t s_align_hold;        /* 本次对齐是否到位后保持 800ms，由 apply_request 按 command 设置 */
static uint8_t s_align_hold_frames; /* 对齐到位后已保持帧数，达到 ALIGN_HOLD_FRAMES 才 finish_align */
static uint32_t s_forward_limit_frames; /* 单项目测试最多执行帧数；0 表示普通持续直行 */
static uint16_t s_open_left_pwm, s_open_right_pwm; /* 已接受的开环两轮输出，切换前不受新请求影响。 */

/**
 * @brief 将动作写入请求邮箱，使上层无需直接改运动状态。
 * @param command 请求动作，不在此函数内执行。
 * @param next_mode 转弯完成后的模式；非转弯动作不使用此参数。
 * @param base_pwm 直行 PWM，开环时为左轮 PWM；转弯后接 IMU 时使用普通直行值 500。
 * @param duration_ms 单项目测试时长，普通动作传 0。
 * @param turn_tolerance 本次转弯容差，0 沿用全局值；非转弯动作传 0。
 * @param right_pwm 开环右轮 PWM；其他动作传 0，与左轮参数在同一临界区提交。
 * @return 无。
 * @note 可从主循环或 Route 调用；短暂屏蔽中断，待处理停车不会被运动请求覆盖。
 */
static void submit_request(CarCommand command, CarMode next_mode, uint16_t base_pwm,
                           uint32_t duration_ms, float turn_tolerance, uint16_t right_pwm)
{
    uint32_t saved_primask = __get_PRIMASK();
    __disable_irq();

    /* 已等待处理的 STOP 不能被其他请求覆盖；其余请求最后一个生效。 */
    if (s_request.pending == 0u || s_request.command != COMMAND_STOP)
    {
        s_request.command = command;
        s_request.next_mode = next_mode;
        s_request.base_pwm = base_pwm;
        s_request.right_pwm = right_pwm;
        s_request.duration_ms = duration_ms;
        s_request.turn_tolerance = turn_tolerance;
        s_request.pending = 1u; // 产生请求
    }

    __set_PRIMASK(saved_primask);
}

/**
 * @brief 取出并清除一个待处理请求，避免同一动作每帧重复初始化。
 * @param request 接收请求副本的有效指针，由内部调用者保证非空。
 * @return 1：已取得请求；0：当前没有请求。
 * @note 仅在 10 ms 控制入口调用；恢复调用前的中断屏蔽状态。
 */
static uint8_t take_request(CarRequest *request)
{
    uint8_t pending;
    uint32_t saved_primask = __get_PRIMASK();
    __disable_irq();

    /* 在临界区内复制并清标志，保证取出的动作与请求标志属于同一次提交。 */
    pending = s_request.pending;
    // 仅在有请求时才写入副本，避免无效数据覆盖。
    if (pending != 0u)
    {
        *request = s_request;
        s_request.pending = 0u; // 清除标志位
    }

    __set_PRIMASK(saved_primask);
    return pending;
}

/**
 * @brief 清除两类控制器的历史，防止上一动作的积分和微分带入新动作。
 * @param target_yaw IMU 新目标角度，单位：度。
 * @return 无。
 * @note 参数和 PWM 限幅保留；仅设置目标并清历史，不驱动电机。
 */
static void reset_controllers(float target_yaw)
{
    PID_clear(&g_line_controller.pid);
    g_line_controller.line_lost = false;
    IMUTask_SetTarget(target_yaw);
    s_turn_done_frames = 0u;
    s_align_done_frames = 0u;
    s_align_hold_frames = 0u; /* 清保持计数；s_align_hold 在 switch 里设置，此处不清 */
}

/**
 * @brief 锁存故障原因并结束本次运动，随后由统一出口停车。
 * @param fault IMU、转弯超时或指令参数故障。
 * @return 无。
 * @note 先保存停车前的模式、路线、角度、灰度和 PWM，再清 PID；显式新运动请求才会解除日志冻结。
 */
static void set_fault(CarFault fault)
{
    /* 必须在修改状态、清 PID 和停车前捕获，避免故障现场被零 PWM 覆盖。 */
    DebugLog_CaptureFault(fault);
    g_mode_fsm.status = CAR_FAULT;
    g_mode_fsm.fault = fault;
    reset_controllers(g_mode_fsm.target_yaw); // 出现错误，清PID
}


/**
 * @brief 接受一个动作请求，统一初始化模式、角度基准与控制历史。
 * @param request 已从邮箱取出的有效请求副本。
 * @return 无。
 * @note 只有新 IMU 动作才重建零度基准；有效运动请求解除故障日志冻结，停车保留现场。
 *       日志在实际接受时记录，被邮箱覆盖的请求不产生记录；IMU 控制等待本帧有效采样。
 *        应用请求
 */
static void apply_request(const CarRequest *request)
{
    const char *command_text = 0; /* 本次真正接受的动作日志；被覆盖的请求不记录。 */
    uint8_t is_turn = (uint8_t)(request->command == COMMAND_LEFT ||
                              request->command == COMMAND_RIGHT); //判断是否是转弯请求

    g_mode_fsm.command_id++;    // 增加动作计数

    if (request->command == COMMAND_ROUTE_TIMEOUT) // 路线超时，不执行动作
    {
        set_fault(CAR_FAULT_ROUTE_TIMEOUT);
        return;
    }

    /* 先校验，非法请求保留故障现场，不解除日志冻结。 */
    if ((is_turn && (request->next_mode != MODE_TRACK && request->next_mode != MODE_IMU)) ||
        (is_turn && (!isfinite(request->turn_tolerance) || request->turn_tolerance < 0.0f ||
                     request->turn_tolerance >= 90.0f)) ||
        ((request->command == COMMAND_IMU_FORWARD || request->command == COMMAND_IMU_TIMED ||
          request->command == COMMAND_REVERSE) &&
         (request->base_pwm == 0u || request->base_pwm > 1000u)) ||
        (request->command == COMMAND_IMU_TIMED && request->duration_ms == 0u) ||
        (request->command == COMMAND_OPEN_LOOP &&
         (request->base_pwm == 0u || request->base_pwm > 1000u ||
          request->right_pwm == 0u || request->right_pwm > 1000u)))
    {
        set_fault(CAR_FAULT_COMMAND);
        return;
    }

    if (request->command != COMMAND_STOP) // 非停车请求解除故障日志冻结
    {
        DebugLog_ClearFault(); /* 参数已经校验；新运动允许再次捕获故障。 */
    }


    //写入动作
    g_mode_fsm.status = CAR_RUNNING;
    g_mode_fsm.fault = CAR_FAULT_NONE;
    g_mode_fsm.state_frames = 0u;
    g_mode_fsm.target_yaw = 0.0f;
    g_mode_fsm.imu_state = IMU_FORWARD;
    g_mode_fsm.imu_valid = 0u; /* 新动作必须等待本帧有效采样。 */
    s_forward_limit_frames = 0u;

    /* 切换动作时保留姿态基准；循迹与开环期间也持续采样，避免漏记实际偏航。
     * update_imu 只处理 IMU 动作的完成条件；IMU 段目标统一取 g_route.ideal_yaw。 */
    if (request->command != COMMAND_STOP && request->command != COMMAND_TRACK &&
        request->command != COMMAND_OPEN_LOOP) //IMU 动作
    {
        g_mode_fsm.mode = MODE_IMU;
    }

    /* 每个动作的状态设置与日志放在同一分支，避免分开维护两套动作判断。 */
    s_align_hold = 0u; /* 默认非保持；仅 COMMAND_ALIGN_HOLD 在下方 switch 内覆盖为 1 */
    switch (request->command) //根据请求的动作类型设置状态
    {
    case COMMAND_OPEN_LOOP:
        g_mode_fsm.mode = MODE_OPEN_LOOP;
        s_open_left_pwm = request->base_pwm;
        s_open_right_pwm = request->right_pwm;
        command_text = "OPEN_LOOP";
        break;
    case COMMAND_TRACK:
        g_mode_fsm.mode = MODE_TRACK;
        command_text = "TRACK";
        break;
    case COMMAND_IMU_FORWARD:
    case COMMAND_IMU_TIMED:
        g_mode_fsm.base_pwm = request->base_pwm; // 设置基础 PWM
        g_mode_fsm.target_yaw = g_route.ideal_yaw; // 绝对目标航向，偏差不累积
        if (request->command == COMMAND_IMU_TIMED) { // 设置前进时间
            s_forward_limit_frames = request->duration_ms / 10u; // 计算前进的帧数
            if (request->duration_ms % 10u != 0u) // 向上取整，如果有余数则加一帧
                ++s_forward_limit_frames;
        }
        command_text = "IMU_FWD";
        break;
    case COMMAND_REVERSE:
        g_mode_fsm.base_pwm = request->base_pwm; // 倒退基础 PWM，调度层内部取负
        g_mode_fsm.imu_state = IMU_REVERSE; // 覆盖默认 IMU_FORWARD
        g_mode_fsm.target_yaw = g_route.ideal_yaw; // 保持当前航向倒退，偏差不累积
        command_text = "IMU_REV";
        break;
    case COMMAND_LEFT:
    case COMMAND_RIGHT:
        g_mode_fsm.base_pwm = 600u; // 转弯后的普通 IMU 直行基础 PWM
        g_mode_fsm.imu_state = request->command == COMMAND_LEFT ? IMU_TURN_LEFT : IMU_TURN_RIGHT;
        /* 绝对目标：Route 提交转弯前已把 ideal_yaw 更新为转后朝向（90° 倍数），
         * 起点偏则转完仍对齐理想坐标系，偏差不累积。 */
        g_mode_fsm.target_yaw = g_route.ideal_yaw;
        g_mode_fsm.after_turn_mode = request->next_mode; // 转弯完成后的模式
        
        if (request->command == COMMAND_LEFT)
        {
            command_text = request->next_mode == MODE_TRACK ? "L90>TRK" : "L90>IMU";
        }
        else
        {
            command_text = request->next_mode == MODE_TRACK ? "R90>TRK" : "R90>IMU";
        }
        break;

    case COMMAND_ALIGN:
    case COMMAND_ALIGN_HOLD:
        /* 功能切换对齐：原地慢转到 ideal_yaw，不重建零度基准；base_pwm 不使用保持 0。
         * COMMAND_ALIGN_HOLD 用于进项目切换，到位后原地保持 800ms 再切 IMU 直行；
         * 普通 COMMAND_ALIGN（路口转弯前前移）到位即完成，保持原快速行为。 */
        g_mode_fsm.imu_state = IMU_ALIGN;
        g_mode_fsm.target_yaw = g_route.ideal_yaw;
        s_align_hold = (uint8_t)(request->command == COMMAND_ALIGN_HOLD);
        command_text = s_align_hold ? "ALIGN_H" : "ALIGN";
        break;
        case COMMAND_STOP:
        g_mode_fsm.status = CAR_STOPPED;
        command_text = "STOP";
        break;
    default:
        break;
    }

    reset_controllers(g_mode_fsm.target_yaw); // 清除控制器历史
    if (is_turn)
        IMUTask_SetTurnTolerance(request->turn_tolerance);
    
    if (command_text != 0)
    {
        DebugLog_Write(LOG_INFO, command_text);
    }
}

/**
 * @brief 记录转弯完成，并按开始时指定的模式自动衔接。
 * @param 无。
 * @return 无。
 * @note 完成日志先于状态重置；接 IMU 前进或倒退时不复位姿态，保持原 ±90°目标。
 *       接循迹时取消本帧角度有效标记，屏幕显示 --；本帧只运行后续控制器。
 */
static void finish_turn(void)
{
    /* 大台阶后退暂时停用，保留原分支便于恢复；转完后直接衔接指定模式。
     * Route 中对应的倒退阶段入口也已注释，避免继续等待原来的 2 秒倒退计时。 */
    // if (g_route.enabled && g_route.state == ROUTE_TURN && g_route.step_index == 7u &&
    //     g_mode_fsm.imu_state == IMU_TURN_LEFT && g_mode_fsm.after_turn_mode == MODE_TRACK)
    // {
    //     DebugLog_Write(LOG_INFO, "DONE>REV");
    //     g_mode_fsm.mode = MODE_IMU;
    //     g_mode_fsm.imu_state = IMU_REVERSE;
    //     g_mode_fsm.base_pwm = 300u; /* 大台阶倒退的基础 PWM；后续通行仍读取表中的 400 PWM */
    // }
    // else
    // {
    DebugLog_Write(LOG_INFO, g_mode_fsm.after_turn_mode == MODE_TRACK ? "DONE>TRK" : "DONE>IMU");
    g_mode_fsm.imu_state = IMU_FORWARD;
    g_mode_fsm.mode = g_mode_fsm.after_turn_mode;
    // }

    g_mode_fsm.turn_count++; //转弯次数+1
    g_mode_fsm.state_frames = 0u; // 重置状态帧计数
    if (g_mode_fsm.mode == MODE_TRACK) //如果是循迹模式，重置目标角度和角度有效标记
    {
        g_mode_fsm.target_yaw = 0.0f;
        g_mode_fsm.imu_valid = 0u;
    }
    reset_controllers(g_mode_fsm.target_yaw);
}

/**
 * @brief 记录对齐完成：停车并递增 align_count，供 Route 轮询衔接下一个功能。
 * @param 无。
 * @return 无。
 * @note 对齐到 g_route.ideal_yaw 后调用；不重建零度基准，yaw 保持连续。
 *       本帧由 ModeFSM_Tick 统一出口停车，下一帧 Route 提交新动作。
 */
static void finish_align(void)
{
    DebugLog_Write(LOG_INFO, "ALIGN_DONE");
    g_mode_fsm.status = CAR_STOPPED;
    g_mode_fsm.align_count++;
    g_mode_fsm.state_frames = 0u;
    s_align_hold = 0u;       /* 防御清零，下一动作从干净状态开始 */
    s_align_hold_frames = 0u;
    reset_controllers(g_mode_fsm.target_yaw);
}

/**
 * @brief 检查转弯与对齐的完成和超时；IMU 读取与姿态解算已在 ModeFSM_Tick 每帧完成。
 * @param 无。
 * @return 无；超时故障时更新 status，由 ModeFSM_Tick 的统一出口停车。
 * @note 本帧 IMU 无效（imu_valid=0）时不做完成判断，暂态暂停由 ModeFSM_Tick 处理。
 *       禁止用旧 yaw 判完成；第 500 帧已进容差而确认不足两帧时最多补一帧。
 */
static void update_imu(void)
{
    float error;
    /* IMU 读取与 Attitude_Tick 已在 ModeFSM_Tick 每帧执行；本帧无效时直接返回，不使用旧数据。 */
    if (g_mode_fsm.imu_valid == 0u)
        return;

    /* 直行只需要有效角度；转弯才需要完成确认和超时判断。 */
    if (g_mode_fsm.imu_state == IMU_FORWARD || g_mode_fsm.imu_state == IMU_REVERSE)
    {
        return;
    }

    /* 对齐：误差进 IMU_ALIGN_DONE_TOLERANCE 连续 ALIGN_DONE_FRAMES 帧即到位。
     * 进项目对齐（s_align_hold）到位后原地保持 ALIGN_HOLD_FRAMES 帧(800ms) 再 finish_align，
     * 期间 imu_state 仍 IMU_ALIGN、status 仍 CAR_RUNNING，AlignTick 容差内输出 0 = 停车，
     * 漂出容差则清计数自动重新对齐；路口转弯前对齐(s_align_hold=0)到位即完成，行为不变。
     * 复用转弯超时机制：保持期间 s_align_done_frames>=ALIGN_DONE_FRAMES，超时分支不触发。 */
    if (g_mode_fsm.imu_state == IMU_ALIGN)
    {
        error = IMUTask_GetError();
        if (fabsf(error) < IMU_ALIGN_DONE_TOLERANCE)
            s_align_done_frames++;
        else
        {
            s_align_done_frames = 0u;
            s_align_hold_frames = 0u; /* 漂出容差：保持计数清零，重新累积到位确认 */
        }
        if (s_align_done_frames >= ALIGN_DONE_FRAMES)
        {
            if (s_align_hold)
            {
                s_align_hold_frames++;
                if (s_align_hold_frames >= ALIGN_HOLD_FRAMES)
                    finish_align();
            }
            else
            {
                finish_align();
            }
        }
        else if (g_mode_fsm.state_frames + 1u >= TURN_TIMEOUT_FRAMES && s_align_done_frames == 0u)
        {
            g_mode_fsm.state_frames++;
            set_fault(CAR_FAULT_TURN_TIMEOUT);
        }
        return;
    }
    
    if (IMUTask_InTurnTolerance()) // 控制输出与完成确认共用本次动作的容差
    {
        s_turn_done_frames++;
    }
    else
    {
        s_turn_done_frames = 0u; /* 容差外一帧即重新计数，避免掠过目标便判完成。 */
    }
    //大于帧数直接判定完成
    if (s_turn_done_frames >= TURN_DONE_FRAMES)
    {
        finish_turn();
    }
    else if (g_mode_fsm.state_frames + 1u >= TURN_TIMEOUT_FRAMES &&
             s_turn_done_frames == 0u)
    {
        /* 已在容差内的一帧允许补足确认，避免目标 -90°、反馈 -86°时被提前故障截断。
         * 连续命中达到两帧就完成，离开范围则计数归零并立即故障，因此最多额外运行一帧。 */
        g_mode_fsm.state_frames++;
        set_fault(CAR_FAULT_TURN_TIMEOUT);
    }
}

/**
 * @brief 上电初始化为停车，清空请求和完成计数，并初始化日志与 Route。
 * @note 在启动 TIM4 前调用；此处不读取外设，也不输出电机 PWM。
 */
void ModeFSM_Init(void)
{
    ModeFSM_t initial = {MODE_TRACK, IMU_FORWARD, CAR_STOPPED, CAR_FAULT_NONE,
                         MODE_TRACK, 0.0f, 0u, 0u, 0u, 0u, 500u, 0u};
    g_mode_fsm = initial;    //不直接修改动作，避免中断时出错
    s_request.pending = 0u;
    s_imu_failed_frames = 0u;
    s_turn_done_frames = 0u;
    s_align_done_frames = 0u;
    s_align_hold = 0u;
    s_align_hold_frames = 0u;
    s_forward_limit_frames = 0u;

    DebugLog_Init();
    DebugLog_Write(LOG_INFO, "BOOT");
    Route_Init();
}

/**
 * @brief 执行一帧灰度检测、上层动作选择和互斥的运动控制。
 * @param 无。
 * @return 无。
 * @note 外部请求优先于 Route；灰度在 IMU 模式中仅用于条件观察，不能调用循迹 PID。
 *       每帧先撤销角度有效标记，只有本帧成功采样并解算有限 yaw 后才能恢复。
 *       顺序：采样灰度 → 接受外部请求 → Route 决策 → 接受本帧请求 → IMU 更新 → 控制输出。
 */
void ModeFSM_Tick(void)
{
    CarRequest request;           /* 从邮箱取出的请求副本。 */
    uint8_t external_command;      /* 本帧接受外部动作后，禁止 Route 覆盖。 */
    uint8_t stop_this_frame = 0u;  /* 已接受停车时，新运动请求留到下一帧。 */

    g_mode_fsm.imu_valid = 0u;      /* 清除的是本帧有效标志，角度数值不在这里清零。 */
    Read_All_Track(g_sensor_data);  //获取灰度数据

    /* 第一次取请求：处理主循环或其他中断已经提交的动作。 */
    external_command = take_request(&request);
    //如果请求不为空
    if (external_command != 0u)
    {
        stop_this_frame = (uint8_t)(request.command == COMMAND_STOP); //判断外部请求是否为停车
        apply_request(&request);
    }
    
    //路径规划
    Route_Tick(g_sensor_data, external_command);

    //执行Route请求
    if (stop_this_frame == 0u && take_request(&request) != 0u)
    {   //不是停车，有执行请求
        apply_request(&request);
    }

    /* 受控快转（对齐/转弯）时禁用 yaw 突变剔除：正常快转不该被当传感器异常冻结 yaw，
     * 否则 PWM≥400 时车体角速度≥500°/s 触发剔除，对齐 PID 读不到真实 yaw 而失效。
     * 循迹和直行时保留剔除，防御传感器尖峰。放在 apply_request 后，imu_state 已是本帧生效值。 */
    Attitude_SetSpikeCheck(
        !(g_mode_fsm.mode == MODE_IMU &&
          (g_mode_fsm.imu_state == IMU_ALIGN ||
           g_mode_fsm.imu_state == IMU_TURN_LEFT ||
           g_mode_fsm.imu_state == IMU_TURN_RIGHT)));

    /* ready 表示设备已初始化；只有 Tick 返回成功，才允许解算并使用本帧数据。
     * 重试仍失败时立即停输出但保留动作，下一帧继续采样；成功后清连续失败计数。
     * 连续三帧失败撤销 ready，交给前台重初始化；已锁存的运动故障不会自动解除。 */
    if (Int_MPU6050_Tick() != 0u)
    {
        s_imu_failed_frames = 0u;
        Attitude_Tick();
        if (isfinite(g_euler.yaw))
            g_mode_fsm.imu_valid = 1u;
        else if (g_mode_fsm.status == CAR_RUNNING)
            set_fault(CAR_FAULT_IMU_DATA);
    }
    else
    {
        if (s_imu_failed_frames < IMU_FAILURE_FRAMES)
            s_imu_failed_frames++;
        if (s_imu_failed_frames >= IMU_FAILURE_FRAMES)
            g_imu_ready = 0u;
        if (g_mode_fsm.status == CAR_RUNNING && g_imu_ready == 0u)
            set_fault(CAR_FAULT_IMU);
    }

    /* 限时依据动作经过的 10 ms 帧数，包含暂停输出的失败帧。 */
    if (g_mode_fsm.status == CAR_RUNNING && g_mode_fsm.mode == MODE_IMU &&
        g_mode_fsm.imu_state == IMU_FORWARD && s_forward_limit_frames != 0u &&
        g_mode_fsm.state_frames >= s_forward_limit_frames)
    {
        g_mode_fsm.status = CAR_STOPPED; //停车
        reset_controllers(g_mode_fsm.target_yaw);
        DebugLog_Write(LOG_INFO, "TEST_DONE");
    }

    /* 暂态失败不能使用旧 yaw 判到位或执行任一运动控制器。
     * 失效帧打断连续到位/保持确认；动作计时仍推进，防止偶发失败反复延长超时。
     * 在转弯/对齐截止帧仍没有有效反馈时按原 5 秒限制结束动作。 */
    if (g_mode_fsm.status == CAR_RUNNING && g_mode_fsm.imu_valid == 0u)
    {
        s_turn_done_frames = 0u;
        s_align_done_frames = 0u;
        s_align_hold_frames = 0u;
        g_mode_fsm.state_frames++;
        if (g_mode_fsm.mode == MODE_IMU &&
            g_mode_fsm.imu_state != IMU_FORWARD &&
            g_mode_fsm.imu_state != IMU_REVERSE &&
            g_mode_fsm.state_frames >= TURN_TIMEOUT_FRAMES)
            set_fault(CAR_FAULT_TURN_TIMEOUT);
        IMUTask_Stop();
        return;
    }

    if (g_mode_fsm.status == CAR_RUNNING && g_mode_fsm.mode == MODE_IMU)
    {
        update_imu();
    }

    /* 主动停车和所有故障统一输出零 PWM，运行帧只执行一个控制器。 */
    if (g_mode_fsm.status != CAR_RUNNING)
    {
        IMUTask_Stop();
        return;
    }

    g_mode_fsm.state_frames++;//状态帧数加一

    if (g_mode_fsm.mode == MODE_OPEN_LOOP)
    {
        /* 唯一运动出口直接给定前进 PWM，IMU 和灰度都不参与修正。 */
        Motor_SetSpeed(&motorLeft, s_open_left_pwm);
        Motor_SetSpeed(&motorRight, s_open_right_pwm);
    }
    else if (g_mode_fsm.mode == MODE_TRACK)  //循迹态
    {
        follow_line(&g_line_controller, g_sensor_data, LINE_RAW_VALUE);
    }
    else if (g_mode_fsm.imu_state == IMU_FORWARD) //直行态
    {
        IMUTask_ForwardTick(g_mode_fsm.base_pwm);
    }
    else if (g_mode_fsm.imu_state == IMU_REVERSE) //保持转后航向倒退
    {
        IMUTask_ReverseTick(g_mode_fsm.base_pwm);
    }
    else if (g_mode_fsm.imu_state == IMU_ALIGN) //功能切换原地对齐
    {
        IMUTask_AlignTick();
    }
    else//转弯态
    {
        IMUTask_TurnTick();
    }
}

//-----------------------------------------------Car_* 接口：提交请求，下一控制帧生效。---------------------------------/
/**
 * @brief 请求启动循迹，使灰度传感器与循迹 PID 接管运动。
 * @param 无。
 * @return 无。
 * @note 仅在需要切换动作时调用一次，不要在主循环每次迭代反复提交。
 */
void Car_Track(void)
{
    submit_request(COMMAND_TRACK, MODE_TRACK, 0u, 0u, 0.0f, 0u);
}

void Car_OpenLoop(void)
{
    Car_OpenLoopWithPWM(OPEN_LOOP_LEFT_PWM, OPEN_LOOP_RIGHT_PWM);
}

void Car_OpenLoopWithPWM(uint16_t left_pwm, uint16_t right_pwm)
{
    submit_request(COMMAND_OPEN_LOOP, MODE_OPEN_LOOP, left_pwm, 0u, 0.0f, right_pwm);
}

/**
 * @brief 请求以当前朝向建立零度基准并持续进行 IMU 直行。
 * @param base_pwm 基础 PWM，范围 1 至 1000，与本次动作一起提交。
 * @return 无。
 * @note 下一帧接受动作；不设定距离或自动结束时间，结束条件由上层决定。
 */
void Car_ImuForward(uint16_t base_pwm)
{
    submit_request(COMMAND_IMU_FORWARD, MODE_IMU, base_pwm, 0u, 0.0f, 0u);
}

/**
 * @brief 请求以当前航向为基准持续倒退，目标取 g_route.ideal_yaw，持续到下一请求。
 * @param base_pwm 倒退基础 PWM；调度层内部取负，yaw 纠偏方向不反转。
 * @return 无。
 * @note 下一帧接受动作；不设定时长，结束条件由上层(Route 数帧)决定。
 */
void Car_Reverse(uint16_t base_pwm)
{
    submit_request(COMMAND_REVERSE, MODE_IMU, base_pwm, 0u, 0.0f, 0u);
}

/**
 * @brief 原子提交测试 PWM 和限时，避免先启动运动、再设置定时造成竞争。
 * @param base_pwm 基础 PWM；非法值由运动层接受请求时故障停车。
 * @param duration_ms 非零测试时长，计时不从主循环提交时刻开始。
 * @return 无。
 */
void Car_ImuForwardTimed(uint16_t base_pwm, uint32_t duration_ms)
{
    submit_request(COMMAND_IMU_TIMED, MODE_IMU, base_pwm, duration_ms, 0.0f, 0u);
}

/**
 * @brief 请求路线超时停车，在接受请求时捕获尚未清零的电机与路线现场。
 * @param 无。
 * @return 无。
 * @note 只提交一次故障请求；停车优先规则与普通运动请求相同。
 */
void Car_RouteTimeout(void)
{
    submit_request(COMMAND_ROUTE_TIMEOUT, MODE_TRACK, 0u, 0u, 0.0f, 0u);
}

/**
 * @brief 请求左转 90°，完成后自动接指定模式。
 * @param next_mode MODE_TRACK 接循迹；MODE_IMU 保持左转目标直行。
 * @return 无。
 * @note 保留左转目标为 +90°的当前约定，不在本接口中安排转前前移。
 */
void Car_TurnLeft90(CarMode next_mode)
{
    submit_request(COMMAND_LEFT, next_mode, 500u, 0u, 0.0f, 0u);
}

/**
 * @brief 提交带局部容差的左转 90°请求，仅影响本次动作。
 * @param next_mode 转完后的运动模式，与普通左转接口一致。
 * @param tolerance_deg 本次到位角差（度，含边界），0 使用原全局容差。
 * @return 无；容差与动作在同一临界区提交，停车或下一动作自动清除覆盖值。
 */
void Car_TurnLeft90WithTolerance(CarMode next_mode, float tolerance_deg)
{
    submit_request(COMMAND_LEFT, next_mode, 500u, 0u, tolerance_deg, 0u);
}

/**
 * @brief 请求右转 90°，完成后自动接指定模式。
 * @param next_mode MODE_TRACK 接循迹；MODE_IMU 保持右转目标直行。
 * @return 无。
 * @note 保留右转目标为 -90°的当前约定；参数非法时下一帧故障停车。
 */
void Car_TurnRight90(CarMode next_mode)
{
    submit_request(COMMAND_RIGHT, next_mode, 500u, 0u, 0.0f, 0u);
}

/**
 * @brief 请求原地对齐到 g_route.ideal_yaw，完成后停车等 Route 衔接。
 * @param 无。
 * @return 无。
 * @note 功能切换时调用；目标由 FSM 在 apply_request 直接取 g_route.ideal_yaw，
 *       不重建零度基准；到位后 align_count++，Route 轮询到变化即提交下一动作。
 */
void Car_AlignTo(void)
{
    submit_request(COMMAND_ALIGN, MODE_IMU, 0u, 0u, 0.0f, 0u);
}

/**
 * @brief 请求原地对齐到 g_route.ideal_yaw，到位后原地保持 800ms 再停车等 Route 衔接。
 * @param 无。
 * @return 无。
 * @note 进项目切换（循迹对正→IMU 项目通行）时调用；相对 Car_AlignTo 多一段到位保持，
 *       等待角度稳定对齐项目后再由 Route 提交 IMU 直行。路口转弯前前移仍用 Car_AlignTo。
 */
void Car_AlignToHold(void)
{
    submit_request(COMMAND_ALIGN_HOLD, MODE_IMU, 0u, 0u, 0.0f, 0u);
}

/**
 * @brief 请求停车，取消待处理运动请求并清除控制器历史。
 * @param 无。
 * @return 无。
 * @note 下一控制帧统一输出零 PWM；当帧停车优先，故障日志和现场保持冻结。
 *       后续有效新运动动作可显式重新启动并解除日志冻结。
 */
void Car_Stop(void)
{
    submit_request(COMMAND_STOP, MODE_TRACK, 0u, 0u, 0.0f, 0u);
}

/**
 * @brief 将中断维护的状态复制成一致的快照。
 * @param snapshot 输出结构体指针；空指针不做任何处理。
 * @return 无。
 * @note 临界区只复制结构体，禁止把 OLED、串口或等待操作放进临界区。
 */
void ModeFSM_GetSnapshot(ModeFSM_t *snapshot)
{
    uint32_t saved_primask;
    if (snapshot == 0)
    {
        return;
    }
    saved_primask = __get_PRIMASK();
    __disable_irq();
    *snapshot = g_mode_fsm;
    __set_PRIMASK(saved_primask);
}

/**
 * @brief 由 HAL 定时器回调进入唯一的小车控制节拍。
 * @param htim 触发回调的定时器句柄；空指针或非 TIM4 均忽略。
 * @return 无。
 * @note TIM4 周期为 10 ms；主循环和其他中断不得重复执行 ModeFSM_Tick。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim != 0 && htim->Instance == TIM4)
    {
        ModeFSM_Tick();
    }
}

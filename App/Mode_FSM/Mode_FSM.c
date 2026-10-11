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
#include <math.h>

#define TURN_DONE_TOLERANCE 7.0f /* 转弯完成误差，单位：度 */
#define TURN_DONE_FRAMES 3u      /* 转弯完成确认帧数,超时->????? */
#define TURN_TIMEOUT_FRAMES 300u /* 300 × 10 ms = 3 秒，上车后校准 */

// 循迹数组
extern uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];
// new状态机对象
volatile ModeFSM_t g_mode_fsm;

// 动作对象
typedef enum
{
    COMMAND_TRACK = 0,
    COMMAND_IMU_FORWARD,
    COMMAND_LEFT,
    COMMAND_RIGHT,
    COMMAND_STOP
} CarCommand;

//
typedef struct
{
    CarCommand command; // 要执行什么动作
    CarMode next_mode;  // 转弯完成后的模式
    uint8_t pending;    // 请求标志
} CarRequest;

/* 单个请求邮箱：正常请求最后一个生效，同帧停车请求优先。 存储外部命令信息 */
static volatile CarRequest s_request;
static uint8_t s_turn_done_frames; // 转弯完成帧数

/**
 * @brief 将动作写入请求邮箱，使上层无需直接改运动状态。
 * @param command 请求动作，不在此函数内执行。
 * @param next_mode 转弯完成后的模式；非转弯动作不使用此参数。
 * @return 无。
 * @note 可从主循环或 Route 调用；短暂屏蔽中断，待处理停车不会被运动请求覆盖。
 */
static void submit_request(CarCommand command, CarMode next_mode)
{
    uint32_t saved_primask = __get_PRIMASK();
    __disable_irq();

    // 仅在邮箱空或新请求为非停车时写入，避免同帧停车被覆盖。
    if (s_request.pending == 0u || s_request.command != COMMAND_STOP)
    {
        s_request.command = command;
        s_request.next_mode = next_mode;
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
    // 这里的pending防止中断被打断，导致请求状态不一致
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
    reset_controllers(g_mode_fsm.target_yaw); // 清PID
}

/**
 * @brief 接受一个动作请求，统一初始化模式、角度基准与控制历史。
 * @param request 已从邮箱取出的有效请求副本。
 * @return 无。
 * @note 只有新 IMU 动作才重建零度基准；有效运动请求解除故障日志冻结，停车保留现场。
 *       日志在实际接受时记录，被邮箱覆盖的请求不产生记录；IMU 控制等待本帧有效采样。
 */
static void apply_request(const CarRequest *request)
{
    // 用于调试，记录请求动作次数
    g_mode_fsm.command_id++;

    // 如果请求是左右转，转后模式必须是循迹或 IMU
    if ((request->command == COMMAND_LEFT || request->command == COMMAND_RIGHT) &&
        request->next_mode != MODE_TRACK && request->next_mode != MODE_IMU)
    {
        set_fault(CAR_FAULT_COMMAND);
        return;
    }

    if (request->command != COMMAND_STOP)
    {
        DebugLog_ClearFault(); /* 参数已经校验；新运动允许再次捕获故障。 */
    }

    g_mode_fsm.status = CAR_RUNNING;
    g_mode_fsm.fault = CAR_FAULT_NONE; // 清除当前故障原因
    g_mode_fsm.state_frames = 0u;      // 动作计时从零开始。
    g_mode_fsm.target_yaw = 0.0f;
    g_mode_fsm.imu_state = IMU_FORWARD; // 因为默认角度是0，所以默认是前进
    g_mode_fsm.imu_valid = 0u;          // 本帧角度暂时无效，必须重新采样确认。

    if (request->command == COMMAND_STOP)
    {
        g_mode_fsm.status = CAR_STOPPED;
    }
    else if (request->command == COMMAND_TRACK) //循迹态
    {
        g_mode_fsm.mode = MODE_TRACK;
    }
    else
    {
        g_mode_fsm.mode = MODE_IMU;     // IMU态
        Attitude_Reset(); /* 新动作以此刻朝向为起点，不沿用上次转弯角度。 */
        if (request->command == COMMAND_LEFT || request->command == COMMAND_RIGHT)
        {
            g_mode_fsm.imu_state = (request->command == COMMAND_LEFT) ? IMU_TURN_LEFT : IMU_TURN_RIGHT;
            g_mode_fsm.target_yaw = (request->command == COMMAND_LEFT) ? 90.0f : -90.0f;
            g_mode_fsm.after_turn_mode = request->next_mode;
        }
    }
    reset_controllers(g_mode_fsm.target_yaw); //重置控制器，避免上一动作的历史影响新动作
    /* 仅记录真正被状态机接受的动作，转弯消息同时说明完成后的去向。 */
    switch (request->command)
    {
    case COMMAND_TRACK:
        DebugLog_Write(LOG_INFO, "TRACK");
        break;
    case COMMAND_IMU_FORWARD:
        DebugLog_Write(LOG_INFO, "IMU_FWD");
        break;
    case COMMAND_LEFT:                                           //左转后是循迹   imu态
        DebugLog_Write(LOG_INFO, request->next_mode == MODE_TRACK ? "L90>TRK" : "L90>IMU");
        break;
    case COMMAND_RIGHT:
        DebugLog_Write(LOG_INFO, request->next_mode == MODE_TRACK ? "R90>TRK" : "R90>IMU");
        break;
    case COMMAND_STOP:
        DebugLog_Write(LOG_INFO, "STOP"); /* 故障冻结时忽略此记录，保留原现场。 */
        break;
    default:
        break;
    }
}

/**
 * @brief 记录转弯完成，并按开始时指定的模式自动衔接。
 * @param 无。
 * @return 无。
 * @note 完成日志先于状态重置；接 IMU 直行时不复位姿态，保持原 ±90°目标。
 *       接循迹时取消本帧角度有效标记，屏幕显示 --；本帧只运行后续控制器。
 */
static void finish_turn(void)
{
    DebugLog_Write(LOG_INFO, g_mode_fsm.after_turn_mode == MODE_TRACK ? "DONE>TRK" : "DONE>IMU");
    g_mode_fsm.turn_count++;
    g_mode_fsm.state_frames = 0u;
    g_mode_fsm.imu_state = IMU_FORWARD; // 转弯完成，IMU 状态恢复为前进
    g_mode_fsm.mode = g_mode_fsm.after_turn_mode;
    if (g_mode_fsm.mode == MODE_TRACK)
    {
        g_mode_fsm.target_yaw = 0.0f;
        g_mode_fsm.imu_valid = 0u;
    }
    reset_controllers(g_mode_fsm.target_yaw);
}

/**
 * @brief 取得本帧有效 IMU 角度，并检查转弯完成与超时。
 * @param 无。
 * @return 1：可继续控制；0：发生故障，应停车。
 * @note 先读六轴、确认成功、再解算角度；仅有限 yaw 才设置本帧 imu_valid。
 *       禁止用读取失败前的旧 yaw 执行 PID，读取失败和超时均先锁存故障现场。
 */
static uint8_t update_imu(void)
{
    float error;
    if (g_imu_ready == 0u)
    {
        set_fault(CAR_FAULT_IMU);
        return 0u;
    }
    Int_MPU6050_Tick();
    if (g_imu_ready == 0u)
    {
        set_fault(CAR_FAULT_IMU);
        return 0u;
    }
    Attitude_Tick();
    if (!isfinite(g_euler.yaw))
    {
        set_fault(CAR_FAULT_IMU);
        return 0u;
    }
    g_mode_fsm.imu_valid = 1u;

    if (g_mode_fsm.imu_state != IMU_FORWARD)
    {
        error = IMUTask_GetError();
        if (fabsf(error) < TURN_DONE_TOLERANCE)
        {
            s_turn_done_frames++;
        }
        else
        {
            s_turn_done_frames = 0u; /* 容差外一帧即重新计数，避免掠过目标便判完成。 */
        }
        if (s_turn_done_frames >= TURN_DONE_FRAMES)
        {
            finish_turn();
        }
        else if (g_mode_fsm.state_frames + 1u >= TURN_TIMEOUT_FRAMES)
        {
            g_mode_fsm.state_frames++;
            set_fault(CAR_FAULT_TURN_TIMEOUT);
            return 0u;
        }
    }
    return 1u;
}

/**
 * @brief 初始化模式状态机
 * @details 该函数用于初始化模式状态机的各个参数，包括初始模式、IMU状态、
 *          车辆状态、故障状态、转向完成帧数等，并初始化路线规划模块
 *
 * 上电后是循迹模式，IMU状态为前进，车辆状态为运行，故障状态为无故障，目标角度为0，状态帧数为0，转向完成帧数为0，转向次数为0
 */
void ModeFSM_Init(void)
{
    // 定义并初始化初始状态结构体，包含模式、IMU状态、车辆状态、故障状态等
    ModeFSM_t initial = {MODE_TRACK, IMU_FORWARD, CAR_RUNNING, CAR_FAULT_NONE,
                         MODE_TRACK, 0.0f, 0u, 0u, 0u, 0u};
    // 将初始状态赋值给全局状态机变量
    g_mode_fsm = initial;
    // 清空请求标志位
    s_request.pending = 0u;
    s_turn_done_frames = 0u;
    DebugLog_Init();
    DebugLog_Write(LOG_INFO, "BOOT");
    Route_Init();
}

//-----------------------------------------------ModeFSM_Tick：主循环，每帧调用一次。---------------------------------/
/**
 * @brief 执行一帧灰度检测、上层动作选择和互斥的运动控制。
 * @param 无。
 * @return 无。
 * @note 外部请求优先于 Route；灰度在 IMU 模式中仅用于条件观察，不能调用循迹 PID。
 *       每帧先撤销角度有效标记，只有本帧成功读取 IMU 后才能恢复，循迹期间显示 --。
 *
 * 第一次取请求：处理已有动作
        ↓
    Route_Tick：检测路口，可能提交新动作
        ↓
    第二次取请求：处理刚产生的动作
        ↓
    执行当前模式的 PID
 */
/**
 * @brief 模式状态机主函数，处理车辆运行状态和模式切换
 * 该函数负责读取传感器数据、处理外部请求、更新车辆状态和执行相应控制任务
 */
void ModeFSM_Tick(void)
{
    CarRequest request;           // 由于封装思想，对象属性不能直接访问，车辆请求结构体，存储外部命令信息
    uint8_t external_command;     // 记录第一次取请求是否成功，告诉 Route 本帧是否已经接受外部动作
    uint8_t stop_this_frame = 0u; // 记录本帧是否已经接受停车，防止后面的新动作覆盖它

    g_mode_fsm.imu_valid = 0u; /* 防止暂停采样或读取失败后把上一帧角度当作实时值。
                                   清零表示：本帧还没有取得有效 IMU 角度，不是把角度数值清零，也不是认定传感器坏了。
                                   灰度在循迹、IMU 直行、转弯等模式下都会读取。循迹时用于控制，IMU 模式下用于观察路口和黑线条件。*/
    // 读取所有传感器数据
    Read_All_Track(g_sensor_data);
    // 获取外部请求  并  提取命令类型
    external_command = take_request(&request);
    // 如果有外部命令，检查是否为停车命令并应用请求
    if (external_command != 0u)
    {
        stop_this_frame = (uint8_t)(request.command == COMMAND_STOP); // 如果停车 stop_this_frame置1 否则置0
        apply_request(&request);                                      // 采取外部请求
    }

    // 执行路径跟踪任务，传入传感器数据和外部命令
    Route_Tick(g_sensor_data, external_command);

    /* 已接受停车时，之后由其他中断提交的新动作留到下一帧，确保本帧确实输出零。 */
    // 如果本帧没有停止，检查是否有新的外部请求并应用
    if (stop_this_frame == 0u && take_request(&request) != 0u)
    { // 利用逻辑短路，如果stop_this_frame为1，则不执行apply_request(&request);
        apply_request(&request);
    }

    // 更新IMU数据 如果车辆处于运行状态           且        使用IMU模式
    if (g_mode_fsm.status == CAR_RUNNING && g_mode_fsm.mode == MODE_IMU)
    {
        (void)update_imu();
    }

    // 所有的故障最后都走这里，如果车辆不处于运行状态，停止IMU任务并返回 停车状态
    if (g_mode_fsm.status != CAR_RUNNING)
    {
        IMUTask_Stop();
        return;
    }

    // 增加状态帧计数
    g_mode_fsm.state_frames++;

    // 根据当前模式执行不同的控制任务
    if (g_mode_fsm.mode == MODE_TRACK)
    {
        // 跟踪线路模式,循迹PID
        follow_line(&g_line_controller, g_sensor_data, LINE_RAW_VALUE);
    }
    else if (g_mode_fsm.imu_state == IMU_FORWARD)
    {
        // IMU前进模式, IMU直行PID
        IMUTask_ForwardTick();
    }
    else
    {
        // IMU转向模式 转向PID
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
    submit_request(COMMAND_TRACK, MODE_TRACK);
}

/**
 * @brief 请求以当前朝向建立零度基准并持续进行 IMU 直行。
 * @param 无。
 * @return 无。
 * @note 下一帧接受动作；不设定距离或自动结束时间，结束条件由上层决定。
 */
void Car_ImuForward(void)
{
    submit_request(COMMAND_IMU_FORWARD, MODE_IMU);
}

/**
 * @brief 请求左转 90°，完成后自动接指定模式。
 * @param next_mode MODE_TRACK 接循迹；MODE_IMU 保持左转目标直行。
 * @return 无。
 * @note 保留左转目标为 +90°的当前约定，不在本接口中安排转前前移。
 */
void Car_TurnLeft90(CarMode next_mode)
{
    submit_request(COMMAND_LEFT, next_mode);
}

/**
 * @brief 请求右转 90°，完成后自动接指定模式。
 * @param next_mode MODE_TRACK 接循迹；MODE_IMU 保持右转目标直行。
 * @return 无。
 * @note 保留右转目标为 -90°的当前约定；参数非法时下一帧故障停车。
 */
void Car_TurnRight90(CarMode next_mode)
{
    submit_request(COMMAND_RIGHT, next_mode);
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
    submit_request(COMMAND_STOP, MODE_TRACK);
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

#ifndef __MODE_FSM_H__
#define __MODE_FSM_H__

#include "main.h"
#include <stdint.h>

/**
 * @file Mode_FSM.h
 * @brief 状态机参数、状态与事件类型、诊断记录，以及主程序可调用的接口。
 * “状态”描述当前动作，“事件”记录最近一次已确认路口，“故障”记录停车原因。
 * 参数单位以各项注释为准：u 是无符号整数后缀，f 是单精度浮点后缀，不是单位。
 * 以下数值是调参起点，必须结合横线宽度、车速、探头安装位置和串口记录实测。
 */

/* 控制调用的名义周期。实际 TIM4 配置在 tim.c 中，修改此宏不会改变硬件定时器。 */
#define MODEFSM_TICK_MS                 10u
/* 候选最多观察 12 帧，约 120ms；只有单侧有效证据时等到此期限才选择转弯。
 * 左右证据同时有效可提前确认十字；恢复稳定窄线也可提前取消候选。 */
#define JUNCTION_CONFIRM_FRAMES         12u
/* 证据从最近一次至少两票成立时计龄，最多保留 8 帧，约 80ms。
 * 允许左右先后经过横线，但不允许用很早的左证据去拼接后来的右证据。 */
#define JUNCTION_PAIR_GAP_FRAMES         8u
/* 候选中连续 3 帧恢复普通窄线就取消；任一帧不满足窄线条件会重新计数。 */
#define JUNCTION_CANCEL_LINE_FRAMES     3u
/* 候选或转前推进中，IN4/IN5 连续 3 帧都离线就停车；不是全部状态的丢线阈值。 */
#define JUNCTION_CENTER_GAP_FRAMES       3u
/* 候选和推进沿进入时的相对 0° 前进，航向误差绝对值超过 15° 就停车。 */
#define JUNCTION_MAX_HEADING_ERROR      15.0f
/* 候选与转前推进合计输出 12 帧低速前进，再开始单侧转弯。
 * 同速条件下才按帧合并；PWM 并非实际车速，该数值不直接代表距离。
 * 默认与确认窗口相同，所以 APPROACH_TURN 通常无需追加推进。 */
#define JUNCTION_ADVANCE_FRAMES         12u
/* 候选、转前推进、驶离使用的基础 PWM；角度环会在此基础上产生左右差速。 */
#define JUNCTION_BASE_PWM               250.0f
/* 驶离分支到第 5 帧才允许累计恢复窄线，防止在路口内部立即切回循迹。 */
#define JUNCTION_EXIT_MIN_FRAMES         5u
/* 驶离时要连续 4 帧窄线才解锁新路口识别；宽线和全零都会打断计数。
 * 默认第 5 帧开始累计，第 5、6、7、8 帧都满足时，第 8 帧恢复循迹。 */
#define JUNCTION_EXIT_LINE_FRAMES        4u
/* 驶离/找线最多 100 帧，约 1 秒；到期未成功就停车，不把到时间当作驶离成功。 */
#define JUNCTION_EXIT_TIMEOUT_FRAMES     100u
/* 正式转弯最多 200 帧，约 2 秒；不包含之前的候选与转前推进。 */
#define TURN_TIMEOUT_FRAMES              200u
/* 连续 3 帧角度误差小于容差才算完成，中途超出容差会清零完成计数。 */
#define TURN_DONE_FRAMES                 3u
/* 完成容差，单位为度；代码使用严格小于，误差恰好为 7° 不计入完成帧。 */
#define TURN_DONE_TOLERANCE              7.0f

/* switch 根据当前状态选择本帧动作，编号 0..7 也写入串口诊断。
 * 保留原有前五个编号；只有 NORMAL_TRACK 能触发新路口，其他动作状态自然锁定事件。 */
typedef enum {
    STATE_NORMAL_TRACK = 0,     /* 0：普通循迹；单帧分支特征只能启动候选。 */
    STATE_CROSS,            /* 1：十字已确认的过渡状态，下一帧选择 0° 直行驶离。 */
    STATE_TURN_LEFT,        /* 2：使用 IMU 转到相对 +90°，按连续角度到位条件结束。 */
    STATE_TURN_RIGHT,       /* 3：使用 IMU 转到相对 -90°，与左转共用完成判断。 */
    STATE_FORWARD,              /* 4：保持已选航向驶离并找线，窄线稳定后恢复循迹。 */
    STATE_JUNCTION_PENDING,   /* 5：低速保持进入时的航向，三帧投票并关联左右证据。 */
    STATE_APPROACH_TURN,        /* 6：补足候选尚未完成的总推进帧数，目标仍为 0°。 */
    STATE_FAULT_STOP            /* 7：锁存停车；IMU 恢复通信也不自动继续旧动作。 */
} Run_State;

/* 最近一次已确认的路口类型，不等于当前状态。
 * 例如通过十字后 state 回到 NORMAL_TRACK，last_event 仍可以是 JUNCTION_CROSS。 */
typedef enum {
    JUNCTION_NONE = 0, /* 0：上电后尚未确认过路口。 */
    JUNCTION_CROSS,    /* 1：左右新鲜证据均成立，当前动作默认直行。 */
    JUNCTION_LEFT,     /* 2：确认期限内最终只有左侧新鲜证据，选择左转。 */
    JUNCTION_RIGHT     /* 3：确认期限内最终只有右侧新鲜证据，选择右转。 */
} Junction_Event;

/* 故障编号随日志输出，可按对应原因定位停车；初始化时为 NONE。 */
typedef enum {
    MODE_FAULT_NONE = 0,       /* 0：无故障。 */
    MODE_FAULT_IMU,            /* 1：通信失败，或姿态输出为非数值、无穷大、明显越界。 */
    MODE_FAULT_HEADING,        /* 2：候选或推进期间偏离进入航向超过上限。 */
    MODE_FAULT_CENTER_LOST,    /* 3：候选或推进期间中心线连续缺失。 */
    MODE_FAULT_AMBIGUOUS,      /* 4：确认期限到达，左右均无新鲜有效证据。 */
    MODE_FAULT_TURN_TIMEOUT,   /* 5：正式转弯达到上限仍未连续角度到位。 */
    MODE_FAULT_EXIT_TIMEOUT,   /* 6：驶离达到上限仍未连续找到普通窄线。 */
    MODE_FAULT_INVALID_STATE   /* 7：状态值不属于已定义枚举。 */
} ModeFSM_Fault;

/* 中断维护的公开状态信息；前台读取整个对象时应调用快照接口。 */
typedef struct {
    Run_State state;            /* 当前要执行的控制阶段。 */
    float target_yaw;           /* 记录的目标角，单位度；转后驶离保留 ±90°。
                                * 写此字段不会自动设置 IMU_Task 的内部目标。 */
    uint32_t state_frames;      /* 本状态的局部帧数，切换状态归零，候选首帧设为 1。
                                * 主要在候选、推进、转弯和驶离状态递增，并非全局时间。 */
    uint32_t event_count;       /* 确认路口时加一；取消候选不增加，通过成功与否不回退。 */
    uint32_t debug_dropped;     /* 日志队列满后丢弃的新记录数，与控制是否执行是两回事。 */
    Junction_Event last_event; /* 最近一次确认结果，恢复循迹时保留，供观察瞬时事件。 */
    ModeFSM_Fault fault;       /* 当前停车原因，故障后保持，重新初始化时清零。 */
    uint8_t raw_mask;           /* 本帧归一化在线位图，bit0=IN1、bit7=IN8。
                                * 在线位为 1，不一定与 GPIO 的高低电平直接相同。 */
    uint8_t left_votes;         /* 左侧最近三帧符合空间判据的帧数，0..3；候选中更新。 */
    uint8_t right_votes;        /* 右侧最近三帧符合空间判据的帧数，0..3；不是探头数量。 */
} ModeFSM_t;

/* 中断逐帧入队、主循环取出打印的诊断快照。整数角度便于串口输出。 */
typedef struct {
    uint32_t tick_ms;           /* 从状态机初始化起按调用次数累计的名义毫秒时间。 */
    uint32_t event_count;       /* 生成本条记录时的累计确认次数。 */
    int16_t yaw_ddeg;           /* 航向角乘 10 后截取整数，-123 表示约 -12.3°。
                                * 普通循迹和停车不更新姿态，此时可能是旧值。 */
    uint8_t raw_mask;           /* 本帧八路在线位图，CSV 中按两位十六进制打印。 */
    uint8_t state;              /* Run_State 编号，记录本帧控制后的最终状态。 */
    uint8_t left_votes;         /* 当前左侧三帧投票数；动作阶段不会继续更新历史。 */
    uint8_t right_votes;        /* 当前右侧三帧投票数。 */
    uint8_t event;              /* Junction_Event 编号，记录最近一次确认结果。 */
    uint8_t fault;              /* ModeFSM_Fault 编号，0 表示无故障。 */
    uint8_t imu_ready;          /* 模块通信状态；为 1 不代表普通循迹态的 yaw 是新数据。 */
} ModeFSM_DebugFrame;

/* 显示使用的诊断副本；不作为控制输入。周期计数来自 DWT，不是名义帧时间。
 * 角度只有 yaw_valid=1 时可用，yaw_fresh=0 表示这一帧未更新。
 * 左右证据年龄仅在候选阶段有效，255 表示未取得或不适用。 */
typedef struct {
    ModeFSM_t fsm;
    uint32_t tick_ms, yaw_tick_ms, advance_frames;
    uint32_t control_last_cycles, control_max_cycles, control_overruns;
    uint32_t interval_max_cycles, late_intervals;
    int16_t yaw_ddeg, pwm_left, pwm_right;
    uint8_t yaw_valid, yaw_fresh, imu_ready;
    uint8_t left_age, right_age, left_fresh, right_fresh;
    uint8_t center_gap, narrow_frames, turn_done;
} ModeFSM_Diagnostics;

/* 定义在 Mode_FSM.c；volatile 保留必要读写，但不能代替快照接口的短临界区。 */
extern volatile ModeFSM_t g_mode_fsm;

/* 上电时在开启 TIM4 之前调用，清零状态、证据和日志位置，进入普通循迹。
 * 不在主循环反复调用，否则会不断丢弃投票与计时记忆。 */
void ModeFSM_Init(void);
/* TIM4 每 10ms 调用一次：采样、执行本阶段的一步控制、保存诊断记录。 */
void ModeFSM_Tick(void);

/* 前台取得整个状态对象的一致副本；snapshot 必须指向调用者的有效结构体。
 * 内部仅复制期间屏蔽中断，并恢复原来的屏蔽状态，返回后再显示副本。 */
void ModeFSM_GetSnapshot(ModeFSM_t *snapshot);
void ModeFSM_GetDiagnostics(ModeFSM_Diagnostics *snapshot);
/* 前台取出最旧的一条日志，成功返回 1，队列为空或地址为空返回 0。
 * 串口打印放在返回后执行，不能在控制中断中排队等待打印。 */
uint8_t ModeFSM_PopDebugFrame(ModeFSM_DebugFrame *frame);

#endif /* __MODE_FSM_H__ */

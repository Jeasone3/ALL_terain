#ifndef __MODE_FSM_H__
#define __MODE_FSM_H__

#include "main.h"
#include <stdint.h>

/* 调参起点：每帧 10ms，时间参数必须结合横线宽度、速度和探头位置实测。 */
#define MODEFSM_TICK_MS                 10u
#define JUNCTION_CONFIRM_FRAMES         12u
#define JUNCTION_PAIR_GAP_FRAMES         8u
#define JUNCTION_CANCEL_LINE_FRAMES     3u
#define JUNCTION_CENTER_GAP_FRAMES       3u
#define JUNCTION_MAX_HEADING_ERROR      15.0f
#define JUNCTION_ADVANCE_FRAMES         12u
#define JUNCTION_BASE_PWM               250.0f
#define JUNCTION_EXIT_MIN_FRAMES         5u
#define JUNCTION_EXIT_LINE_FRAMES        4u
/* 保留原来 1 秒的设置作为驶离失败上限，不再到时间自动恢复循迹。 */
#define JUNCTION_EXIT_TIMEOUT_FRAMES     100u
#define TURN_TIMEOUT_FRAMES              200u
#define TURN_DONE_FRAMES                 3u
#define TURN_DONE_TOLERANCE              7.0f

/* 保留原有状态编号，新增候选、转前推进和锁存停车状态。 */
typedef enum {
    STATE_NORMAL_TRACK = 0,
    STATE_CROSS,
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT,
    STATE_FORWARD,              /* 保持选定航向，驶离路口并重新找线 */
    STATE_JUNCTION_PENDING,
    STATE_APPROACH_TURN,
    STATE_FAULT_STOP
} Run_State;

typedef enum {
    JUNCTION_NONE = 0,
    JUNCTION_CROSS,
    JUNCTION_LEFT,
    JUNCTION_RIGHT
} Junction_Event;

typedef enum {
    MODE_FAULT_NONE = 0,
    MODE_FAULT_IMU,
    MODE_FAULT_HEADING,
    MODE_FAULT_CENTER_LOST,
    MODE_FAULT_AMBIGUOUS,
    MODE_FAULT_TURN_TIMEOUT,
    MODE_FAULT_EXIT_TIMEOUT,
    MODE_FAULT_INVALID_STATE
} ModeFSM_Fault;

typedef struct {
    Run_State state;
    float target_yaw;           /* 当前角度控制目标；转后驶离仍保持 ±90° */
    uint32_t state_frames;
    uint32_t event_count;       /* 已确认的路口次数，不随短暂状态消失 */
    uint32_t debug_dropped;     /* 日志缓冲满时丢弃的新帧数 */
    Junction_Event last_event;
    ModeFSM_Fault fault;
    uint8_t raw_mask;           /* 在线掩码，bit0 对应 IN1，bit7 对应 IN8 */
    uint8_t left_votes;
    uint8_t right_votes;
} ModeFSM_t;

typedef struct {
    uint32_t tick_ms;
    uint32_t event_count;
    int16_t yaw_ddeg;           /* 角度乘 10，供前台输出整数 */
    uint8_t raw_mask;
    uint8_t state;
    uint8_t left_votes;
    uint8_t right_votes;
    uint8_t event;
    uint8_t fault;
    uint8_t imu_ready;
} ModeFSM_DebugFrame;

extern volatile ModeFSM_t g_mode_fsm;

/* 上电初始化；故障停车不会因通信自动恢复而重新启动。 */
void ModeFSM_Init(void);
void ModeFSM_Tick(void);

/* 前台短临界区读取；保留调用前的中断屏蔽状态。 */
void ModeFSM_GetSnapshot(ModeFSM_t *snapshot);
uint8_t ModeFSM_PopDebugFrame(ModeFSM_DebugFrame *frame);

#endif /* __MODE_FSM_H__ */

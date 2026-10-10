/**
 * @file Debug_Log.c
 * @brief 固定三条日志和一份故障现场；记录轻量，所有显示工作放在主循环。
 * @note 不分配动态内存，不调用运动控制函数，不在中断中格式化或通信。
 */
#include "Debug_Log.h"
#include "Route.h"
#include "Track_Task.h"
#include "Int_MPU6050.h"
#include "Attitude.h"
#include "motor.h"
#include <math.h>
#include <stdio.h>

#define DEBUG_LOG_CAPACITY       3u
#define DEBUG_LOG_TEXT_LENGTH   11u
#define DEBUG_LOG_ANGLE_LENGTH   8u
#define DEBUG_LOG_ANGLE_LIMIT  999.9f /* 带符号和一位小数最多六列，确保整行不超 21 列。 */

extern uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];
extern Motor_Struct motorLeft;
extern Motor_Struct motorRight;

typedef struct {
    uint32_t timestamp_ms;           /* HAL 毫秒节拍；显示时换算为开机秒数 */
    LogLevel level;
    char text[DEBUG_LOG_TEXT_LENGTH + 1u];
} DebugLog_Record;

typedef struct {
    ModeFSM_t motion;
    RouteState route_state;
    float yaw;
    uint8_t imu_ready;
    uint8_t line_mask;               /* bit0 至 bit7 对应 IN1 至 IN8，1 表示在线 */
    int16_t left_pwm;
    int16_t right_pwm;
} DebugLog_Snapshot;

typedef struct {
    DebugLog_Snapshot state;
    DebugLog_Record records[DEBUG_LOG_CAPACITY];
    uint8_t count;
} DebugLog_DisplaySnapshot;

/* 主循环和 TIM4 都可能访问；所有更新、复制均在恢复原 PRIMASK 的短临界区内完成。 */
static volatile DebugLog_Record s_records[DEBUG_LOG_CAPACITY];
static volatile DebugLog_Snapshot s_fault;
static volatile uint8_t s_count;
static volatile uint8_t s_frozen;

/**
 * @brief 复制当前运动、路线和传感器状态，形成一致的调试现场。
 * @param snapshot 接收现场的有效指针，内部调用保证非空。
 * @return 无。
 * @note 调用者必须已屏蔽中断；只复制数值，不格式化、不访问硬件或修改控制状态。
 *       原始灰度电平按 LINE_RAW_VALUE 转换，屏幕上的 1 始终表示检测到线。
 */
static void capture_state_locked(DebugLog_Snapshot *snapshot)
{
    uint8_t channel;

    snapshot->motion = g_mode_fsm;
    snapshot->route_state = g_route.state;
    snapshot->yaw = g_euler.yaw;
    snapshot->imu_ready = g_imu_ready;
    snapshot->left_pwm = motorLeft.speed;
    snapshot->right_pwm = motorRight.speed;
    snapshot->line_mask = 0u;
    for (channel = 0u; channel < GRAYSCALE_SENSOR_CHANNELS; channel++) {
        if (g_sensor_data[channel] == LINE_RAW_VALUE) {
            snapshot->line_mask |= (uint8_t)(1u << channel);
        }
    }
}

/**
 * @brief 把准备好的记录追加到固定数组，满时左移以保留最新三条。
 * @param record 已包含时间、等级和终止字符的记录，内部保证指针有效。
 * @return 无。
 * @note 调用者必须已屏蔽中断且允许记录；本函数不检查冻结、不读取字符串或格式化。
 */
static void append_record_locked(const DebugLog_Record *record)
{
    uint8_t index;

    if (s_count == DEBUG_LOG_CAPACITY) {
        for (index = 1u; index < DEBUG_LOG_CAPACITY; index++) {
            s_records[index - 1u] = s_records[index];
        }
    } else {
        s_count++;
    }
    s_records[s_count - 1u] = *record;
}

/**
 * @brief 清空固定记录及故障锁存，完成一次开机的调试模块初始化。
 * @param 无。
 * @return 无。
 * @note TIM4 启动前调用；OLED 通信重试只重新初始化 OLED，不重新清空日志。
 */
void DebugLog_Init(void)
{
    uint8_t index;
    DebugLog_Record empty_record = { 0u, LOG_INFO, { 0 } };
    DebugLog_Snapshot empty_state = {
        { MODE_TRACK, IMU_FORWARD, CAR_STOPPED, CAR_FAULT_NONE,
          MODE_TRACK, 0.0f, 0u, 0u, 0u, 0u },
        ROUTE_FOLLOW, 0.0f, 0u, 0u, 0, 0
    };
    uint32_t saved_primask = __get_PRIMASK();

    __disable_irq();
    for (index = 0u; index < DEBUG_LOG_CAPACITY; index++) {
        s_records[index] = empty_record;
    }
    s_fault = empty_state;
    s_count = 0u;
    s_frozen = 0u;
    __set_PRIMASK(saved_primask);
}

/**
 * @brief 有界复制用户文本，并在允许记录时追加时间和等级。
 * @param level 三种有效日志等级之一；其他值直接忽略。
 * @param text 待记录的字符串；空指针或空串忽略，最多保存 11 个字符。
 * @return 无。
 * @note 字符检查在进入临界区前进行；临界区只更新三条固定记录。
 *       可在中断调用，百分号作为普通文本保存；冻结后所有普通写入均忽略。
 */
void DebugLog_Write(LogLevel level, const char *text)
{
    uint8_t index;
    DebugLog_Record record = { 0u, LOG_INFO, { 0 } };
    uint32_t saved_primask;

    if ((level != LOG_INFO && level != LOG_WARN && level != LOG_ERROR) ||
        text == 0 || text[0] == '\0') {
        return;
    }
    record.level = level;
    for (index = 0u; index < DEBUG_LOG_TEXT_LENGTH && text[index] != '\0'; index++) {
        unsigned char character = (unsigned char)text[index];
        record.text[index] = (character >= 32u && character <= 126u) ?
                             (char)character : '?';
    }
    record.text[index] = '\0';

    saved_primask = __get_PRIMASK();
    __disable_irq();
    if (s_frozen == 0u) {
        record.timestamp_ms = HAL_GetTick();
        append_record_locked(&record);
    }
    __set_PRIMASK(saved_primask);
}

/**
 * @brief 取得故障的短英文消息，供中断直接记录而无需格式化。
 * @param fault 本次运动故障类型。
 * @return 静态字符串：IMU_ERR、TURN_TO 或 BAD_CMD。
 * @note 未知故障类型也显示 BAD_CMD；只返回常量，不修改任何状态。
 */
static const char *fault_text(CarFault fault)
{
    switch (fault) {
        case CAR_FAULT_IMU:          return "IMU_ERR";
        case CAR_FAULT_TURN_TIMEOUT: return "TURN_TO";
        case CAR_FAULT_COMMAND:      return "BAD_CMD";
        default:                    return "BAD_CMD";
    }
}

/**
 * @brief 保存首次故障发生前的现场，追加原因并冻结日志。
 * @param fault 本次 CarFault 故障原因；CAR_FAULT_NONE 忽略，未知非零值按指令故障记录。
 * @return 无。
 * @note 在运动层清 PID、置故障状态和停止电机前调用；后续故障不覆盖首次现场。
 *       只把副本标记为 CAR_FAULT，不改变真实状态机；不会调用 OLED 或运动函数。
 */
void DebugLog_CaptureFault(CarFault fault)
{
    uint8_t index;
    const char *text = fault_text(fault);
    DebugLog_Record record = { 0u, LOG_ERROR, { 0 } };
    DebugLog_Snapshot snapshot;
    uint32_t saved_primask;

    if (fault == CAR_FAULT_NONE) {
        return;  /* 没有故障的通知不能锁存现场或阻止后续正常日志。 */
    }

    /* 故障消息都是短常量；在屏蔽中断前完成有界复制。 */
    for (index = 0u; index < DEBUG_LOG_TEXT_LENGTH && text[index] != '\0'; index++) {
        record.text[index] = text[index];
    }
    record.text[index] = '\0';

    saved_primask = __get_PRIMASK();
    __disable_irq();
    if (s_frozen == 0u) {
        capture_state_locked(&snapshot);
        snapshot.motion.status = CAR_FAULT;
        snapshot.motion.fault = fault;
        s_fault = snapshot;
        record.timestamp_ms = HAL_GetTick();
        append_record_locked(&record);
        s_frozen = 1u;
    }
    __set_PRIMASK(saved_primask);
}

/**
 * @brief 解除首次故障的冻结，恢复实时现场显示和普通日志记录。
 * @param 无。
 * @return 无。
 * @note 接受新的有效运动动作时调用，普通停车不调用；已有历史保留。
 *       不清除真实故障或恢复运动，只改变日志模块的冻结标志。
 */
void DebugLog_ClearFault(void)
{
    uint32_t saved_primask = __get_PRIMASK();

    __disable_irq();
    s_frozen = 0u;
    __set_PRIMASK(saved_primask);
}

/**
 * @brief 将日志与实时或冻结现场复制到前台局部变量。
 * @param display 接收完整显示快照的有效指针，内部调用保证非空。
 * @return 无。
 * @note 临界区只复制有限数据，恢复中断后才能格式化和发送 OLED。
 */
static void get_display_snapshot(DebugLog_DisplaySnapshot *display)
{
    uint8_t index;
    uint32_t saved_primask = __get_PRIMASK();

    __disable_irq();
    if (s_frozen != 0u) {
        display->state = s_fault;
    } else {
        capture_state_locked(&display->state);
    }
    display->count = s_count;
    for (index = 0u; index < DEBUG_LOG_CAPACITY; index++) {
        display->records[index] = s_records[index];
    }
    __set_PRIMASK(saved_primask);
}

/**
 * @brief 将上层路线步骤转成屏幕可容纳的英文缩写。
 * @param state 快照中的 Route 步骤。
 * @return FOLLOW、APP、TURN、CROSS；非法步骤返回 ?。
 * @note 仅前台显示使用，不据此选择或改变动作。
 */
static const char *route_name(RouteState state)
{
    switch (state) {
        case ROUTE_FOLLOW:   return "FOLLOW";
        case ROUTE_APPROACH: return "APP";
        case ROUTE_TURN:     return "TURN";
        case ROUTE_CROSSING: return "CROSS";
        default:            return "?";
    }
}

/**
 * @brief 根据运动状态取得运行标识，供屏幕第一行显示。
 * @param motion 完整运动快照，内部保证指针有效。
 * @return FAULT、STOP、TRACK 或 IMU。
 * @note 故障和停车优先于模式显示；不读取实时全局变量。
 */
static const char *status_name(const ModeFSM_t *motion)
{
    if (motion->status == CAR_FAULT) {
        return "FAULT";
    }
    if (motion->status == CAR_STOPPED) {
        return "STOP";
    }
    return (motion->mode == MODE_TRACK) ? "TRACK" : "IMU";
}

/**
 * @brief 将快照中的运动模式及 IMU 子状态转成动作缩写。
 * @param motion 完整运动快照，内部保证指针有效。
 * @return TRK、FWD、L90、R90；非法 IMU 子状态返回 ?。
 * @note 冻结的故障快照保留故障发生时动作；普通停车显示运动层保留的当前模式与子状态。
 */
static const char *action_name(const ModeFSM_t *motion)
{
    if (motion->mode == MODE_TRACK) {
        return "TRK";
    }
    switch (motion->imu_state) {
        case IMU_FORWARD:    return "FWD";
        case IMU_TURN_LEFT:  return "L90";
        case IMU_TURN_RIGHT: return "R90";
        default:            return "?";
    }
}

/**
 * @brief 判断角度是否有限且能在屏幕预留的六列内完整显示。
 * @param value 待显示的角度，单位为度。
 * @return 1：有限且绝对值不超过 999.9 度；0：无效或超出显示宽度。
 * @note 仅限制显示，不限制运动目标或修改真实角度；避免异常值挤掉同一行其他字段。
 */
static uint8_t angle_is_displayable(float value)
{
    return (uint8_t)(isfinite(value) && fabsf(value) <= DEBUG_LOG_ANGLE_LIMIT);
}

/**
 * @brief 把可显示角度格式化为带正负号的一位小数文本，否则显示 --。
 * @param buffer 至少 DEBUG_LOG_ANGLE_LENGTH 字节的目标缓冲区。
 * @param value 待显示的角度，单位为度。
 * @param valid 非零允许显示；零表示暂停或无效，即使 value 有限也显示 --。
 * @return 无。
 * @note 仅恢复中断后调用；先检查有限性和显示宽度，不将 NaN 或无穷值转换为整数。
 */
static void format_angle(char *buffer, float value, uint8_t valid)
{
    if (valid != 0u && angle_is_displayable(value) != 0u) {
        (void)snprintf(buffer, DEBUG_LOG_ANGLE_LENGTH, "%+.1f", (double)value);
    } else {
        buffer[0] = '-';
        buffer[1] = '-';
        buffer[2] = '\0';
    }
}

/**
 * @brief 在局部快照上计算目标减当前航向的最短角差。
 * @param target 与 yaw 使用同一基准的目标角度，单位为度。
 * @param yaw 当前角度，单位为度。
 * @return [-180, 180] 度内的最短角差；异常输入结果由显示侧有限性检查处理。
 * @note 与 IMU 控制器采用相同折返公式，不读取其静态目标，不修改控制器。
 */
static float shortest_angle_error(float target, float yaw)
{
    float error = fmodf(target - yaw, 360.0f);

    if (error > 180.0f) {
        error -= 360.0f;
    } else if (error < -180.0f) {
        error += 360.0f;
    }
    return error;
}

/**
 * @brief 绘制五行状态和三条事件，并返回本次 OLED 刷新状态。
 * @param 无。
 * @return OLED_Update 的结果；通信失败由主循环沿用原有重连流程处理。
 * @note 每 100 ms 从主循环调用；临界区结束后才进行浮点格式化和 I2C 通信。
 *       冻结现场优先显示，循迹角度显示 --；消息通过 %s 绘制，百分号不会成为格式串。
 */
OLED_Status DebugLog_Display(void)
{
    DebugLog_DisplaySnapshot display;
    char yaw_text[DEBUG_LOG_ANGLE_LENGTH];
    char target_text[DEBUG_LOG_ANGLE_LENGTH];
    char error_text[DEBUG_LOG_ANGLE_LENGTH];
    char line_text[GRAYSCALE_SENSOR_CHANNELS + 1u];
    uint8_t row;
    uint8_t first_log_row;
    uint8_t imu_mode;
    uint8_t angle_valid;
    uint8_t target_valid;
    float error = 0.0f;

    get_display_snapshot(&display);
    imu_mode = (uint8_t)(display.state.motion.mode == MODE_IMU);
    angle_valid = (uint8_t)(imu_mode != 0u &&
                            display.state.motion.imu_valid != 0u &&
                            angle_is_displayable(display.state.yaw) != 0u);
    target_valid = (uint8_t)(imu_mode != 0u &&
                             angle_is_displayable(display.state.motion.target_yaw) != 0u);
    if (angle_valid != 0u && target_valid != 0u) {
        error = shortest_angle_error(display.state.motion.target_yaw, display.state.yaw);
    }
    format_angle(yaw_text, display.state.yaw, angle_valid);
    format_angle(target_text, display.state.motion.target_yaw, target_valid);
    format_angle(error_text, error,
                 (uint8_t)(angle_valid != 0u && target_valid != 0u));
    for (row = 0u; row < GRAYSCALE_SENSOR_CHANNELS; row++) {
        line_text[row] = ((display.state.line_mask & (uint8_t)(1u << row)) != 0u) ? '1' : '0';
    }
    line_text[GRAYSCALE_SENSOR_CHANNELS] = '\0';

    OLED_PrintLine(0u, "%s %s R:%s", status_name(&display.state.motion),
                   action_name(&display.state.motion), route_name(display.state.route_state));
    OLED_PrintLine(1u, "Y:%s T:%s", yaw_text, target_text);
    OLED_PrintLine(2u, "E:%s N:%lu", error_text,
                   (unsigned long)display.state.motion.state_frames);
    OLED_PrintLine(3u, "G:%s I:%u", line_text, (unsigned int)(display.state.imu_ready != 0u));
    OLED_PrintLine(4u, "L:%+d R:%+d", (int)display.state.left_pwm, (int)display.state.right_pwm);
    first_log_row = (uint8_t)(DEBUG_LOG_CAPACITY - display.count);
    for (row = 0u; row < DEBUG_LOG_CAPACITY; row++) {
        /* 不满三条时顶部留空，使最新日志始终位于屏幕最后一行。 */
        if (row >= first_log_row) {
            const DebugLog_Record *record = &display.records[row - first_log_row];
            char level_character = (record->level == LOG_ERROR) ? 'E' :
                                   ((record->level == LOG_WARN) ? 'W' : 'I');
            OLED_PrintLine((uint8_t)(5u + row), "%lu %c %s",
                           (unsigned long)(record->timestamp_ms / 1000u),
                           (int)level_character, record->text);
        } else {
            OLED_PrintLine((uint8_t)(5u + row), "%s", "");
        }
    }
    return OLED_Update();
}

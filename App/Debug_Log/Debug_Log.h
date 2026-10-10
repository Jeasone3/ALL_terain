/**
 * @file Debug_Log.h
 * @brief 简单 OLED 调试日志：五行状态与三条最近事件，故障时保留停车前现场。
 * @note 记录接口可从主循环或中断调用；显示接口只能在主循环调用。
 */
#ifndef __DEBUG_LOG_H__
#define __DEBUG_LOG_H__

#include "Mode_FSM.h"
#include "Int_OLED.h"

typedef enum {
    LOG_INFO = 0,                 /* 一般事件，屏幕显示 I */
    LOG_WARN,                     /* 寻线、丢线等提醒，屏幕显示 W */
    LOG_ERROR                     /* 故障事件，屏幕显示 E */
} LogLevel;

/**
 * @brief 清空日志和故障锁存，使新一次开机从空记录开始。
 * @param 无。
 * @return 无。
 * @note 启动 TIM4 前调用一次；不初始化或刷新 OLED。OLED 重连时不要再次调用。
 */
void DebugLog_Init(void);

/**
 * @brief 保存一条带开机时间和等级的短日志，超过三条时覆盖最旧记录。
 * @param level LOG_INFO、LOG_WARN 或 LOG_ERROR；其他取值忽略。
 * @param text 消息字符串，最多保存前 11 个字符；空指针或空字符串忽略。
 * @return 无。
 * @note 可在主循环或中断调用；只复制文本，不格式化、不访问 OLED、不驱动电机。
 *       ASCII 32 至 126 原样保存（包括百分号），其余字节替换为问号。
 *       首次故障冻结后忽略普通记录，包括 LOG_ERROR；临界区恢复原中断屏蔽状态。
 */
void DebugLog_Write(LogLevel level, const char *text);

/**
 * @brief 在运动故障入口锁存停车前状态，并把故障原因作为最后一条错误日志。
 * @param fault 本次故障原因，使用 CarFault 中的 IMU、转弯超时或指令错误；NONE 忽略。
 * @return 无。
 * @note 必须在修改真实运动状态、清 PID 和电机停车之前调用。
 *       仅首次故障生效，随后现场及三条记录冻结；不修改运动层或驱动电机。
 */
void DebugLog_CaptureFault(CarFault fault);

/**
 * @brief 解除故障现场和日志的冻结，允许重新显示实时状态、追加日志。
 * @param 无。
 * @return 无。
 * @note 保留已有三条记录；运动层只在接受新的有效运动动作时调用，普通停车不调用。
 *       只管理调试显示，不能清除真实运动故障或恢复电机输出。
 */
void DebugLog_ClearFault(void);

/**
 * @brief 绘制五行实时状态或冻结现场，以及三条按时间排列的最近日志。
 * @param 无。
 * @return OLED_Update 的状态，供主循环判断通信失败并按原流程重试。
 * @note 仅主循环每 100 ms 调用；本函数不调度、不等待、不初始化 OLED。
 *       先在短临界区复制完整快照，恢复中断后才进行浮点格式化和 OLED 通信。
 *       循迹时 Y/T/E 显示 --；IMU 本帧无效时 Y/E 显示 --，有限目标 T 仍可显示。
 *       绝对值超过 999.9 度的角度也显示 --；帧数保留完整 uint32_t，最长一行 21 列。
 */
OLED_Status DebugLog_Display(void);

#endif /* __DEBUG_LOG_H__ */

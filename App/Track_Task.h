#ifndef __TRACK_TASK_H__
#define __TRACK_TASK_H__

#include "main.h"
#include "Int_Track.h"
#include "stdbool.h"
#include "math.h"
#include "Com_Limit.h"
#include "Com_pid.h"
#include "motor.h"

/* 传感器在线上时返回的原始电平(0/1), 按传感器极性修改 */
#define LINE_RAW_VALUE   1

/* 偏差死区: |error|<此值时置 0 喂 PID, 抑制 P/I(微分也被压平, 简化处理) */
#define LINE_DEADBAND    1.0f

/* 运行中丢线时整体目标降速系数, 防止冲出赛道 */
#define LINE_LOST_SCALE  0.6f

typedef struct {
    /* PID 实例(位置式, 含积分分离/微分滤波), 替代原手搓 kp/ki/kd/integral/last_error */
    pid_type_def pid;

    /* 控制参数 (单位: PWM 量纲 0-1000, 无编码器直接驱动占空比) */
    int16_t base_speed;     // 基础速度(直行占空比)
    int16_t max_speed;      // 最大速度(限幅)

    /* 传感器位置权重 */
    float sensor_weights[GRAYSCALE_SENSOR_CHANNELS];

    /* 运行状态 */
    bool track_started;     // 是否已进入循迹
    bool line_lost;         // 最近一次检测是否丢线(供 follow_line 降速)
} line_following_t;

/* 全局循迹控制对象 */
extern line_following_t g_line_controller;

void line_following_init(line_following_t* controller);
float calculate_error(line_following_t* controller, uint16_t* sensor_values, uint16_t line_raw_value);
void follow_line(line_following_t* controller, uint16_t* sensor_values,
                 uint16_t line_raw_value);

/* 循迹节拍入口: 读传感器 + 循迹计算, 供 TIM4 中断调用 */
void TrackTask_Tick(void);


#endif /* __TRACK_TASK_H__ */

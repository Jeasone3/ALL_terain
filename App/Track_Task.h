#ifndef __TRACK_TASK_H__
#define __TRACK_TASK_H__

#include "main.h"
#include "Int_Track.h"
#include "stdbool.h"
#include "math.h"
#include "Com_Limit.h"
#include "motor.h"

/* 传感器在线上时返回的原始电平(0/1), 按传感器极性修改 */
#define LINE_RAW_VALUE   1

/* 偏差死区: |error|<此值时不产生 P/I 纠正, 但保留原始值用于微分与过零检测 */
#define LINE_DEADBAND    0.6f

/* 运行中丢线时整体目标降速系数, 防止冲出赛道 */
#define LINE_LOST_SCALE  0.6f

typedef struct {
    /* PID 参数 PID Parameters */
    float kp;               // 比例系数
    float ki;               // 积分系数
    float kd;               // 微分系数

    float last_error;       // 上次原始偏差(未受死区影响)
    float integral;         // 积分累加

    /* 控制参数 (单位: 线速度 mm/s) */
    int16_t base_speed;     // 基础速度
    int16_t max_speed;      // 最大速度

    /* 传感器位置权重 */
    float sensor_weights[GRAYSCALE_SENSOR_CHANNELS];

    /* 运行状态 */
    // bool motor_locked;      // 上电首次解锁闸, 解锁后不再因丢线重新锁定
    bool track_started;     // 是否已进入循迹
    bool line_lost;         // 最近一次检测是否丢线(供 follow_line 降速)
} line_following_t;

/* 全局循迹控制对象 */
extern line_following_t g_line_controller;

/* 循迹环输出给速度环的左右轮线速度目标(mm/s): 循迹环写, 速度环读 */
extern float g_left_target_speed;
extern float g_right_target_speed;

void line_following_init(line_following_t* controller);
bool check_sensors_safe(line_following_t* controller, uint16_t* sensor_values);
float calculate_error(line_following_t* controller, uint16_t* sensor_values, uint16_t line_raw_value);
float pid_control(line_following_t* controller, float error);
void differential_speed_control(line_following_t* controller, float pid_output,
                                float* left_target, float* right_target);
void follow_line(line_following_t* controller, uint16_t* sensor_values,
                 uint16_t line_raw_value);

/* 循迹节拍入口: 读传感器 + 循迹计算, 供 TIM6 软分频调用 */
void TrackTask_Tick(void);




#endif /* __TRACK_TASK_H__ */

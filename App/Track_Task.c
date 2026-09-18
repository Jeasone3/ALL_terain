#include "Track_Task.h"

//全局变量传感器数据
extern uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];

extern Motor_Struct motorRight;
extern Motor_Struct motorLeft;

//循迹控制对象
line_following_t g_line_controller;

/* 默认位置权重表: 关于中心对称, 负左正右; 改通道数时只需改 GRAYSCALE_SENSOR_CHANNELS */
static const float k_default_weights[GRAYSCALE_SENSOR_CHANNELS] = {
    -5.0f, -4.0f, -2.0f, -1.0f, 1.0f, 2.0f, 4.0f, 5.0f
};

void line_following_init(line_following_t *controller)
{
    /* PID 参数: 串级化后输出量纲变为 mm/s 增量, 以下为起点, 需上电机实测整定 */
    controller->kp = 270.0f;
    controller->ki = 0.5f;
    controller->kd = 50.0f;

    controller->last_error = 0.0f;
    controller->integral   = 0.0f;

    /* 单位: 线速度 mm/s; 参考 TARGET_LINEAR_SPEED≈848mm/s(3 圈/秒) */
    controller->base_speed = 300;
    controller->max_speed  = 1000;

    for (uint8_t i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        controller->sensor_weights[i] = k_default_weights[i];
    }

    //controller->motor_locked  = true;   /* 上电锁定, 传感器就位再解锁 */
    controller->track_started = false;     //循迹开始标志
    controller->line_lost     = false;    //丢线判断

    g_left_target_speed  = 0.0f;
    g_right_target_speed = 0.0f;
}

/**
 * @brief 计算偏差(加权平均)
 * @note  丢线时返回上次偏差并置 line_lost, 由 follow_line 据此降速回正
 */
float calculate_error(line_following_t* controller, uint16_t* sensor_values, uint16_t line_raw_value) {
    float weighted_sum = 0.0f;
    int active_sensors = 0;

    for (int i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        if (sensor_values[i] == line_raw_value) {
            weighted_sum += controller->sensor_weights[i];
            active_sensors++;
        }
    }
    /**
     * @brief 丢线返回最近一次的偏差
     * 
     */
    if (active_sensors == 0) {
        controller->line_lost = true;      /* 丢线: 沿用上次偏差回正 */
        return controller->last_error;
    }
    //回正
    controller->line_lost = false;
    return weighted_sum / active_sensors;
}


/**
 * @brief 循迹 PID 控制
 * @note  死区只抑制 P/I 贡献, 原始 error 保留用于过零检测与微分, 避免历史被压平;
 *        丢线时 error 恒等于 last_error, 跳过积分累加防止饱和
 */
float pid_control(line_following_t* controller, float error) {
    float raw = error;
    //死区控制
    float eff = (fabsf(raw) < LINE_DEADBAND) ? 0.0f : raw;

    /* 过零点清积分(用原始值判断, 不被死区掩盖) */
    if ((controller->last_error > 0.0f && raw < 0.0f) ||
        (controller->last_error < 0.0f && raw > 0.0f)) {
        controller->integral = 0.0f;
    }

    /* 动态积分限幅 */
    float integral_limit;
    if (fabsf(raw) > 3.0f) {
        integral_limit = 80.0f;
    } else if (fabsf(raw) > 1.5f) {
        integral_limit = 50.0f;
    } else {
        integral_limit = 20.0f;
    }

    /* 丢线时不累加积分(error=last_error 恒定, 累加会漂向饱和) */
    if (!controller->line_lost) {
        controller->integral += eff;
        controller->integral = Com_Limit(controller->integral, -integral_limit, integral_limit);
    }

    /* 微分用原始值, 不被死区压平 */
    float derivative = raw - controller->last_error;

    float output = (controller->kp * eff +
                    controller->ki * controller->integral +
                    controller->kd * derivative);

    controller->last_error = raw;
    return output;
}


/**
 * @brief 差速控制: 由 PID 输出算左右轮线速度目标, 限幅后写入全局目标
 */
void differential_speed_control(line_following_t* controller, float pid_output,
                                float* left_target, float* right_target) {
    float base = (float)controller->base_speed;
    float maxv = (float)controller->max_speed;

    float left  = base + pid_output;
    float right = base - pid_output;

    *left_target  = Com_Limit(left,  -maxv, maxv);
    *right_target = Com_Limit(right, -maxv, maxv);
}

/**
 * @brief 循迹主函数: 输出左右轮速度目标, 不直接驱动电机(由速度环跟踪)
 */
void follow_line(line_following_t* controller, uint16_t* sensor_values,
                 uint16_t line_raw_value) {

    // /* 上电安全锁: 传感器不全同才解锁起步; 解锁后不再因丢线重新锁定 */
    // if (controller->motor_locked) {
    //     if (check_sensors_safe(controller, sensor_values)) {
    //         controller->motor_locked  = false;
    //         controller->track_started = true;
    //     } else {
    //         g_left_target_speed  = 0.0f;
    //         g_right_target_speed = 0.0f;
    //         return;
    //     }
    // }

    float error       = calculate_error(controller, sensor_values, line_raw_value);
    float pid_output  = pid_control(controller, error);

    differential_speed_control(controller, pid_output,
                               (float*)&motorLeft.speed, (float*)&motorRight.speed);

    /* 运行中丢线: 沿 last_error 方向回正的同时整体降速, 避免冲出 */
    if (controller->line_lost) {
        motorLeft.speed  *= LINE_LOST_SCALE;
        motorRight.speed *= LINE_LOST_SCALE;
    }
}


void TrackTask_Tick(void) {
    // 读取所有传感器的数据并存储到全局变量g_sensor_data中
    Read_All_Track(g_sensor_data);
    follow_line(&g_line_controller, g_sensor_data, LINE_RAW_VALUE);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim->Instance == TIM4){
        TrackTask_Tick();
    }
}


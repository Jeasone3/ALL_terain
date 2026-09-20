#include "Track_Task.h"
#include "Int_MPU6050.h"

//全局变量传感器数据
extern uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];

extern Motor_Struct motorRight;
extern Motor_Struct motorLeft;

//创建循迹控制对象
line_following_t g_line_controller;

/* 默认位置权重表: 关于中心对称, 负左正右; 改通道数时只需改 GRAYSCALE_SENSOR_CHANNELS */
static const float k_default_weights[GRAYSCALE_SENSOR_CHANNELS] = {
    -5.0f, -4.0f, -2.0f, -1.0f, 1.0f, 2.0f, 4.0f, 5.0f
};

void line_following_init(line_following_t *controller)
{
    /* PID 参数: 输出量纲为 PWM 增量(±max_speed); 以下为起点, 需上电机实测整定 */
    pid_real_t pid_params[3] = {300.0f, 0.5f, 50.0f};
    PID_init(&controller->pid, PID_POSITION, pid_params,
             /*max_out=*/1000.0f, /*max_iout=*/90.0f);
    PID_set_integral_separation(&controller->pid, 3.0f);   /* 近似原"动态积分限幅" */
    PID_set_deriv_filter(&controller->pid, 0.7f);          /* 微分低通, 抗噪 */

    /* 单位: PWM 量纲 0-1000(无编码器, 直接驱动占空比) */
    controller->base_speed = 400;
    controller->max_speed  = 1000;

    for (uint8_t i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        controller->sensor_weights[i] = k_default_weights[i];
    }

    controller->track_started = false;     //循迹开始标志
    controller->line_lost     = false;    //丢线判断
}

/**
 * @brief 计算偏差(加权平均)
 * @note  丢线时返回上次偏差并置 line_lost, 由 follow_line 据此降速回正
 *        上次偏差取自 pid.error[0](由 PID_calc 每周期更新, 等价于原 last_error)
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

    if (active_sensors == 0) {
        controller->line_lost = true;       /* 丢线: 沿用上次偏差回正 */
        return controller->pid.error[0];     /* 返回最近一次偏差 */
    }

    controller->line_lost = false;
    return weighted_sum / active_sensors;
}


/**
 * @brief 循迹主函数: 算偏差 -> PID -> 差速 -> 直接驱动电机(开环, 无速度环)
 * @note  权重左负右正: 线在右 error>0 -> out>0 -> 左轮加速/右轮减速 -> 右转回正
 *        PID_calc 内部 err = set - ref, 故传 (ref=0, set=error) 使 err=error
 *        丢线时沿用 last_error 喂 PID + 整体降速, 依赖 max_iout 抗积分饱和
 */
void follow_line(line_following_t* controller, uint16_t* sensor_values,
                 uint16_t line_raw_value) {

    float error = calculate_error(controller, sensor_values, line_raw_value);

    /* 死区: |error|<阈值时置 0, 抑制 P/I(微分也被压平, 简化处理) */
    if (fabsf(error) < LINE_DEADBAND) {
        error = 0.0f;
    }

    /* PID 算纠正量: ref=0, set=error -> 内部 err = error, out ∝ error */
    float out = PID_calc(&controller->pid, 0.0f, error);

    /* 差速: base ± out, 限幅到 ±max_speed(PWM 量纲) */
    float base = (float)controller->base_speed;
    float maxv = (float)controller->max_speed;
    float left  = base + out;
    float right = base - out;
    left  = Com_Limit(left,  -maxv, maxv);
    right = Com_Limit(right, -maxv, maxv);

    /* 丢线时整体降速, 防冲出 */
    if (controller->line_lost) {
        left  *= LINE_LOST_SCALE;
        right *= LINE_LOST_SCALE;
    }

    Motor_SetSpeed(&motorLeft,  (int16_t)left);
    Motor_SetSpeed(&motorRight, (int16_t)right);
}


void TrackTask_Tick(void) {
    // 读取所有传感器的数据并存储到全局变量g_sensor_data中
    Read_All_Track(g_sensor_data);
    follow_line(&g_line_controller, g_sensor_data, LINE_RAW_VALUE);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim->Instance == TIM4){
        Int_MPU6050_Tick();   /* 六轴读取，10ms 周期 */
        TrackTask_Tick();
    }
}

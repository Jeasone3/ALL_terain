#include "motor.h"

#if DC_MOTOR == 1

//创建电机对象

Motor_Struct motorRight = {
    .htim = &htim8,
    .channel = TIM_CHANNEL_1,
    .in1_port = AIN1_GPIO_Port,
    .in1_pin = AIN1_Pin,
    .in2_port = AIN2_GPIO_Port,
    .in2_pin = AIN2_Pin,
    .speed = 0,
    .direction = MOTOR_STOP
};
Motor_Struct motorLeft = {
    .htim = &htim2,
    .channel = TIM_CHANNEL_4,
    .in1_port = BIN1_GPIO_Port,
    .in1_pin = BIN1_Pin,
    .in2_port = BIN2_GPIO_Port,
    .in2_pin = BIN2_Pin,
    .speed = 0,
    .direction = MOTOR_STOP
};


/* 方向真值表,按 Motor_Direction 枚举顺序索引 */
static const struct {
    GPIO_PinState in1;
    GPIO_PinState in2;
} s_dirTable[] = {
    {GPIO_PIN_RESET, GPIO_PIN_RESET}, /* MOTOR_STOP:     两端低, 滑行 */
    {GPIO_PIN_SET,   GPIO_PIN_RESET}, /* MOTOR_FORWARD:  IN1 高 IN2 低 */
    {GPIO_PIN_RESET, GPIO_PIN_SET},   /* MOTOR_BACKWARD: IN1 低 IN2 高 */
    {GPIO_PIN_SET,   GPIO_PIN_SET},   /* MOTOR_BRAKE:    两端高, 刹车 */
};

/**
 * @brief 初始化电机,占空比为0,开启定时器pwm模式
 *
 * @param motor 传入的电机结构体指针
 */
void Motor_Init(Motor_Struct *motor)
{
    //PCB 给连死了，不用打开
    // //tb6612初始化，开启stby
    // HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET);

    motor->direction = MOTOR_STOP;
    motor->speed = 0;
    Motor_SetDirection(motor);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);
    HAL_TIM_PWM_Start(motor->htim, motor->channel);//开启PWM
    motor->lin_speed = 0.0f;
}

/**
 * @brief 将 motor->direction 应用到 TB6612 引脚,调用前先设置 direction 字段
 *
 * @param motor 传入的电机结构体指针
 */
void Motor_SetDirection(Motor_Struct *motor)
{
    if (motor->direction > MOTOR_BRAKE){
        motor->direction = MOTOR_STOP; /* 防非法枚举值越界 */
    }
    HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, s_dirTable[motor->direction].in1);
    HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, s_dirTable[motor->direction].in2);
}


/**
 * @brief 设置电机速度,范围为-1000到1000,负数为反转,正数为正转,0为停机滑行
 *
 * @param motor 传入的电机结构体指针
 * @param speed 速度值(-1000-1000)
 */
void Motor_SetSpeed(Motor_Struct *motor, int16_t speed)
{
    speed = Com_Limit(speed,-1000,1000) ;
    motor->speed = speed;

    if (speed < 0){
        motor->direction = MOTOR_BACKWARD;
        speed = -speed;
    }else if (speed > 0){
        motor->direction = MOTOR_FORWARD;
    }else{
        motor->direction = MOTOR_STOP; //MOTOR_BRAKE;
    }

    Motor_SetDirection(motor);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, speed);
}


#endif

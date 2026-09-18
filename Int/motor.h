#ifndef __MOTOR_H__
#define __MOTOR_H__

/**
 * @file motor.h
 * @author  Jeason
 * @brief   电机没有使用编码器闭环，只是使用了角度闭环
 * @version 0.1
 * @date 2026-09-18
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "Com_Limit.h"

#define DC_MOTOR 1

typedef enum {
    MOTOR_STOP = 0,
    MOTOR_BACKWARD,
    MOTOR_FORWARD,
    MOTOR_BRAKE
} Motor_Direction;

typedef struct {
    //----------------PWM-----------------//
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    GPIO_TypeDef *in1_port;
    uint16_t in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t in2_pin;
    int16_t speed;
    Motor_Direction direction;
    
    float   lin_speed;               /* 轮子线速度(mm/s),由 Motor_Clac_Speed 更新 */
} Motor_Struct;



void Motor_Init(Motor_Struct *motor);
void Motor_SetSpeed(Motor_Struct *motor, int16_t speed);
/* 将 motor->direction 应用到 TB6612 引脚,调用前先设置 direction 字段 */
void Motor_SetDirection(Motor_Struct *motor);


#endif /* __MOTOR_H__ */

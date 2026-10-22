/** @brief 致命错误出口使用的电机字段，PWM 写入由测试替身记录。 */
#ifndef TEST_RTOS_MOTOR_H
#define TEST_RTOS_MOTOR_H
#include "main.h"
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    int16_t speed;
} Motor_Struct;
#endif

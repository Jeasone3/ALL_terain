#ifndef TEST_MOTOR_H
#define TEST_MOTOR_H

#include "main.h"

typedef struct {
    uint8_t side;
} Motor_Struct;

extern Motor_Struct motorLeft;
extern Motor_Struct motorRight;
void Motor_SetSpeed(Motor_Struct *motor, int16_t speed);

#endif

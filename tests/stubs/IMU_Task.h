#ifndef TEST_IMU_TASK_H
#define TEST_IMU_TASK_H

void IMUTask_SetTarget(float target);
void IMUTask_TurnTick(void);
void IMUTask_ForwardTick(void);
void IMUTask_ForwardTickWithSpeed(float base_speed);
void Temp_Turn_Forward(void);
void IMUTask_Stop(void);

#endif

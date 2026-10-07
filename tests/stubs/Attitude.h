#ifndef TEST_ATTITUDE_H
#define TEST_ATTITUDE_H

#include "main.h"

typedef struct {
    float yaw;
    float pitch;
    float roll;
} Euler_struct;

extern Euler_struct g_euler;
void Attitude_Reset(void);
void Attitude_Tick(void);

#endif

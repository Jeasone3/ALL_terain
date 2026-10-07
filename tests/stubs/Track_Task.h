#ifndef TEST_TRACK_TASK_H
#define TEST_TRACK_TASK_H

#include "main.h"
#include <stdbool.h>

/* 同一组样例分别以高、低有效电平构建，防止路口判据绕过极性配置。 */
#ifndef LINE_RAW_VALUE
#define LINE_RAW_VALUE 1
#endif

typedef struct {
    float error[3];
} pid_type_def;

typedef struct {
    pid_type_def pid;
    int16_t base_speed;
    int16_t max_speed;
    bool track_started;
    bool line_lost;
} line_following_t;

extern line_following_t g_line_controller;
void PID_clear(pid_type_def *pid);
void follow_line(line_following_t *controller, uint16_t *sensor_values,
                 uint16_t line_raw_value);

#endif

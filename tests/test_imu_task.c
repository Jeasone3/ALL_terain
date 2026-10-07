#include "IMU_Task.h"
#include "Attitude.h"
#include "motor.h"
#include <stdio.h>

/* 直接调用真实角度控制和真实 PID，替身仅记录左右轮最终命令。 */
Euler_struct g_euler;
Motor_Struct motorLeft = {0};
Motor_Struct motorRight = {1};
static int16_t s_left_command;
static int16_t s_right_command;

void Motor_SetSpeed(Motor_Struct *motor, int16_t speed)
{
    if (motor == &motorLeft) s_left_command = speed;
    if (motor == &motorRight) s_right_command = speed;
}

#define CHECK(condition) do { \
    if (!(condition)) { \
        printf("FAIL %s:%d: %s (L=%d R=%d)\n", __func__, __LINE__, \
               #condition, s_left_command, s_right_command); \
        return 1; \
    } \
} while (0)

int main(void)
{
    unsigned int i;
    const float disturbance[] = {0.0f, 5.0f, 20.0f, -20.0f, 80.0f, -80.0f};

    IMUTask_Init();
    IMUTask_SetTarget(0.0f);
    g_euler.yaw = 0.0f;
    IMUTask_ForwardTickWithSpeed(250.0f);
    CHECK(s_left_command == 250 && s_right_command == 250);
    printf("PASS low_speed_straight\n");

    IMUTask_SetTarget(0.0f);
    g_euler.yaw = 20.0f;
    IMUTask_ForwardTickWithSpeed(250.0f);
    CHECK(s_left_command > s_right_command);
    CHECK(s_left_command >= 0 && s_right_command >= 0);
    IMUTask_SetTarget(0.0f);
    g_euler.yaw = -20.0f;
    IMUTask_ForwardTickWithSpeed(250.0f);
    CHECK(s_left_command < s_right_command);
    CHECK(s_left_command >= 0 && s_right_command >= 0);
    printf("PASS correction_direction_for_both_yaw_signs\n");

    /* 连续跳变会激励实际微分项，低速输出仍应保持前进或停车。 */
    IMUTask_SetTarget(0.0f);
    for (i = 0u; i < (unsigned int)(sizeof(disturbance) / sizeof(disturbance[0])); i++) {
        g_euler.yaw = disturbance[i];
        IMUTask_ForwardTickWithSpeed(250.0f);
        CHECK(s_left_command >= 0 && s_right_command >= 0);
        CHECK(s_left_command <= 500 && s_right_command <= 500);
    }
    printf("PASS low_speed_heading_disturbance_never_reverses\n");

    IMUTask_SetTarget(90.0f);
    g_euler.yaw = 90.0f;
    IMUTask_ForwardTickWithSpeed(250.0f);
    CHECK(s_left_command == 250 && s_right_command == 250);
    IMUTask_SetTarget(-90.0f);
    g_euler.yaw = -90.0f;
    IMUTask_ForwardTickWithSpeed(250.0f);
    CHECK(s_left_command == 250 && s_right_command == 250);
    printf("PASS exit_keeps_selected_turn_target\n");

    /* 跨越正负一百八十度时，转弯和直行必须选择同一条最短转向。 */
    IMUTask_SetTarget(90.0f);
    g_euler.yaw = -179.0f;
    IMUTask_TurnTick();
    CHECK(s_left_command > 0 && s_right_command < 0);
    IMUTask_ForwardTickWithSpeed(250.0f);
    CHECK(s_left_command > s_right_command);
    CHECK(s_left_command >= 0 && s_right_command >= 0);
    IMUTask_SetTarget(-90.0f);
    g_euler.yaw = 179.0f;
    IMUTask_TurnTick();
    CHECK(s_left_command < 0 && s_right_command > 0);
    IMUTask_ForwardTickWithSpeed(250.0f);
    CHECK(s_left_command < s_right_command);
    CHECK(s_left_command >= 0 && s_right_command >= 0);
    printf("PASS turn_and_forward_use_shortest_wrapped_heading\n");

    g_euler.yaw = 0.0f;
    IMUTask_ForwardTickWithSpeed(0.0f);
    CHECK(s_left_command == 0 && s_right_command == 0);
    IMUTask_ForwardTickWithSpeed(-100.0f);
    CHECK(s_left_command == 0 && s_right_command == 0);
    IMUTask_Stop();
    CHECK(s_left_command == 0 && s_right_command == 0);
    printf("PASS zero_or_negative_speed_and_stop\n");

    printf("RESULT 6/6 passed\n");
    return 0;
}

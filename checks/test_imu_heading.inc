/* 合入临时归档样例后，调用真实角度控制与 PID；只断言最终电机命令。 */
static int check_early_heading_correction(void)
{
    unsigned int i;
    const float small_changes[] = {5.0f, 4.0f, -5.0f, -4.0f, 1.0f, -1.0f, 0.0f};

    IMUTask_Init();
    IMUTask_SetTarget(0.0f);
    g_euler.yaw = 5.0f;
    IMUTask_ForwardTickWithSpeed(250.0f);
    CHECK(s_left_command > s_right_command);
    CHECK(s_left_command >= 0 && s_right_command >= 0);
    CHECK(s_left_command <= 500 && s_right_command <= 500);
    IMUTask_SetTarget(0.0f);
    g_euler.yaw = -5.0f;
    IMUTask_ForwardTickWithSpeed(250.0f);
    CHECK(s_left_command < s_right_command);
    CHECK(s_left_command >= 0 && s_right_command >= 0);
    CHECK(s_left_command <= 500 && s_right_command <= 500);
    printf("PASS early_heading_correction_for_both_yaw_signs\n");

    /* 每次重新设目标清除旧微分历史，验证小幅静态误差仍保持等速。 */
    IMUTask_SetTarget(0.0f);
    g_euler.yaw = 1.0f;
    IMUTask_ForwardTickWithSpeed(250.0f);
    CHECK(s_left_command == 250 && s_right_command == 250);
    IMUTask_SetTarget(0.0f);
    g_euler.yaw = -1.0f;
    IMUTask_ForwardTickWithSpeed(250.0f);
    CHECK(s_left_command == 250 && s_right_command == 250);
    printf("PASS small_static_heading_error_keeps_equal_pwm\n");

    /* 转后驶离以选定的航向为基准，两侧五度偏差也应向目标纠偏。 */
    IMUTask_SetTarget(90.0f);
    g_euler.yaw = 85.0f;
    IMUTask_ForwardTickWithSpeed(250.0f);
    CHECK(s_left_command < s_right_command);
    CHECK(s_left_command >= 0 && s_right_command >= 0);
    CHECK(s_left_command <= 500 && s_right_command <= 500);
    IMUTask_SetTarget(90.0f);
    g_euler.yaw = 95.0f;
    IMUTask_ForwardTickWithSpeed(250.0f);
    CHECK(s_left_command > s_right_command);
    CHECK(s_left_command >= 0 && s_right_command >= 0);
    CHECK(s_left_command <= 500 && s_right_command <= 500);
    printf("PASS early_correction_uses_selected_exit_heading\n");

    /* 保留连续输入的微分历史；短时变化仍不得让低速前进变成反转。 */
    IMUTask_SetTarget(0.0f);
    for (i = 0u; i < (unsigned int)(sizeof(small_changes) / sizeof(small_changes[0])); ++i) {
        g_euler.yaw = small_changes[i];
        IMUTask_ForwardTickWithSpeed(250.0f);
        CHECK(s_left_command >= 0 && s_right_command >= 0);
        CHECK(s_left_command <= 500 && s_right_command <= 500);
    }
    printf("PASS small_heading_changes_keep_forward_pwm_bounded\n");
    printf("RESULT early heading correction 4/4 passed\n");
    return 0;
}

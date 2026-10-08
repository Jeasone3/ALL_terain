/* 与归档回归样例在临时目录合并，复用其硬件替身，调用真实诊断及回调。 */
static int test_diagnostic_freshness_and_evidence(void)
{
    ModeFSM_Diagnostics d;
    reset_fixture();
    step(0x18u, 0.0f);
    ModeFSM_GetDiagnostics(&d);
    CHECK(d.yaw_valid == 0u && d.yaw_fresh == 0u);
    CHECK(d.pwm_left == 400 && d.pwm_right == 400);
    CHECK(d.fsm.raw_mask == 0x18u);
    step(0x0fu, 0.0f);
    ModeFSM_GetDiagnostics(&d);
    CHECK(d.yaw_valid == 1u && d.yaw_fresh == 1u);
    CHECK(d.left_fresh == 0u && d.left_age == 255u);
    step(0x0fu, 4.0f);
    ModeFSM_GetDiagnostics(&d);
    CHECK(d.yaw_ddeg == 40 && d.left_fresh == 1u && d.left_age == 0u);
    step(0x18u, 0.0f); step(0x18u, 0.0f); step(0x18u, 0.0f);
    step(0x18u, 0.0f);
    ModeFSM_GetDiagnostics(&d);
    CHECK(d.fsm.state == STATE_NORMAL_TRACK && d.yaw_fresh == 0u && d.yaw_valid == 1u);
    CHECK(d.left_age == 255u && d.left_fresh == 0u);
    s_primask = 1u;
    ModeFSM_GetDiagnostics(&d);
    CHECK(s_primask == 1u);
    s_primask = 0u;
    s_fail_imu_once = 1u;
    step(0x0fu, 0.0f);
    ModeFSM_GetDiagnostics(&d);
    CHECK(d.yaw_valid == 0u && d.fsm.fault == MODE_FAULT_IMU);
    CHECK(d.pwm_left == 0 && d.pwm_right == 0);
    CHECK(d.fault_capture_valid == 1u && d.fault_yaw_valid == 0u);
    CHECK(d.fault_state == STATE_JUNCTION_PENDING && d.fault_mask == 0x0fu);

    reset_fixture();
    step(0x0fu, 0.0f);
    step(0x0fu, 20.0f);
    ModeFSM_GetDiagnostics(&d);
    CHECK(d.fsm.fault == MODE_FAULT_HEADING && d.fsm.state == STATE_FAULT_STOP);
    CHECK(d.fault_capture_valid == 1u && d.fault_yaw_valid == 1u);
    CHECK(d.fault_state == STATE_JUNCTION_PENDING && d.fault_frames == 2u);
    CHECK(d.fault_yaw_ddeg == 200 && d.fault_target_ddeg == 0);
    CHECK(d.fault_pwm_left == 250 && d.fault_pwm_right == 250);
    CHECK(d.pwm_left == 0 && d.pwm_right == 0);
    step(0x00u, 50.0f);
    ModeFSM_GetDiagnostics(&d);
    CHECK(d.fault_mask == 0x0fu && d.fault_yaw_ddeg == 200 && d.fault_frames == 2u);
    return 1;
}

static int test_control_timing_callback(void)
{
    TIM_HandleTypeDef timer = {TIM4};
    ModeFSM_Diagnostics d;
    void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
    reset_fixture();
    s_input_mask = 0x18u;
    test_control_cycles = 144000u;
    test_dwt.CYCCNT = 0xffff0000u;
    HAL_TIM_PeriodElapsedCallback(&timer);
    ModeFSM_GetDiagnostics(&d);
    CHECK(d.control_last_cycles == 144000u && d.control_max_cycles == 144000u);
    CHECK(d.control_overruns == 0u && d.late_intervals == 0u);
    test_dwt.CYCCNT = 0xffff0000u + 720000u;
    test_control_cycles = 800000u;
    HAL_TIM_PeriodElapsedCallback(&timer);
    ModeFSM_GetDiagnostics(&d);
    CHECK(d.control_last_cycles == 800000u && d.control_overruns == 1u);
    CHECK(d.interval_max_cycles == 720000u);
    test_dwt.CYCCNT += 720000u;
    test_control_cycles = 72000u;
    HAL_TIM_PeriodElapsedCallback(&timer);
    ModeFSM_GetDiagnostics(&d);
    CHECK(d.late_intervals == 1u && d.control_max_cycles == 800000u);
    test_control_cycles = 0u;
    return 1;
}

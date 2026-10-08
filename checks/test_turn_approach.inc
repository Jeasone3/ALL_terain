/* 与归档样例合并，执行真实 FSM；硬件替身只记录控制命令，不推进状态。 */
static int enter_confirmed_turn_approach(uint8_t mask, Junction_Event event)
{
    ModeFSM_Diagnostics d;
    steps(JUNCTION_CONFIRM_FRAMES, mask, 0.0f);
    ModeFSM_GetDiagnostics(&d);
    CHECK(d.fsm.state == STATE_APPROACH_TURN);
    CHECK(d.fsm.state_frames == 0u && d.advance_frames == 0u);
    CHECK(d.fsm.event_count == 1u && d.fsm.last_event == event);
    CHECK(close_float(d.fsm.target_yaw, 0.0f));
    CHECK(close_float(s_last_target, 0.0f));
    CHECK(s_turn_calls == 0);
    CHECK(s_forward_calls == (int)JUNCTION_CONFIRM_FRAMES);
    CHECK(s_left_command == (int16_t)JUNCTION_BASE_PWM);
    CHECK(s_right_command == (int16_t)JUNCTION_BASE_PWM);
    return 1;
}

static int test_turn_approach_finishes_before_rotation(void)
{
    unsigned int direction, frame;
    const uint8_t masks[] = {0x0fu, 0xf0u};
    const Junction_Event events[] = {JUNCTION_LEFT, JUNCTION_RIGHT};
    const Run_State turn_states[] = {STATE_TURN_LEFT, STATE_TURN_RIGHT};
    const float targets[] = {90.0f, -90.0f};
    for (direction = 0u; direction < 2u; ++direction) {
        ModeFSM_Diagnostics d;
        reset_fixture();
        CHECK(enter_confirmed_turn_approach(masks[direction], events[direction]));
        for (frame = 1u; frame <= TURN_APPROACH_FRAMES; ++frame) {
            step(0x18u, 0.0f);
            ModeFSM_GetDiagnostics(&d);
            CHECK(d.advance_frames == frame);
            CHECK(d.fsm.fault == MODE_FAULT_NONE && d.fsm.event_count == 1u);
            CHECK(s_turn_calls == 0);
            CHECK(s_left_command == (int16_t)TURN_APPROACH_BASE_PWM);
            CHECK(s_right_command == (int16_t)TURN_APPROACH_BASE_PWM);
            if (frame < TURN_APPROACH_FRAMES) {
                CHECK(d.fsm.state == STATE_APPROACH_TURN && d.fsm.state_frames == frame);
                CHECK(close_float(d.fsm.target_yaw, 0.0f));
                CHECK(close_float(s_last_target, 0.0f));
            } else {
                CHECK(d.fsm.state == turn_states[direction] && d.fsm.state_frames == 0u);
                CHECK(close_float(d.fsm.target_yaw, targets[direction]));
                CHECK(close_float(s_last_target, targets[direction]));
            }
        }
        CHECK(s_forward_calls == (int)(JUNCTION_CONFIRM_FRAMES + TURN_APPROACH_FRAMES));
        step(0xffu, 0.0f);
        CHECK(snapshot().state == turn_states[direction]);
        CHECK(snapshot().state_frames == 1u && snapshot().event_count == 1u);
        CHECK(s_turn_calls == 1);
        if (direction == 0u) CHECK(s_left_command < 0 && s_right_command > 0);
        else CHECK(s_left_command > 0 && s_right_command < 0);
    }
    return 1;
}

static int test_confirmed_approach_allows_center_loss_and_ignores_new_width(void)
{
    unsigned int frame;
    ModeFSM_Diagnostics d;
    reset_fixture();
    CHECK(enter_confirmed_turn_approach(0x0fu, JUNCTION_LEFT));
    for (frame = 1u; frame <= TURN_APPROACH_FRAMES; ++frame) {
        /* 越过已确认拐点后中心可能全部离线，之后重新遇宽线也不能重判十字。 */
        step(frame <= JUNCTION_CENTER_GAP_FRAMES + 2u ? 0x00u : 0xffu, 0.0f);
        ModeFSM_GetDiagnostics(&d);
        CHECK(d.fsm.fault == MODE_FAULT_NONE);
        CHECK(d.fsm.event_count == 1u && d.fsm.last_event == JUNCTION_LEFT);
        CHECK(d.advance_frames == frame);
        CHECK(s_turn_calls == 0);
        if (frame < TURN_APPROACH_FRAMES) CHECK(d.fsm.state == STATE_APPROACH_TURN);
        else CHECK(d.fsm.state == STATE_TURN_LEFT);
    }
    return 1;
}

static int test_turn_approach_imu_failure_stops_same_frame(void)
{
    int forward_calls;
    ModeFSM_Diagnostics d;
    reset_fixture();
    CHECK(enter_confirmed_turn_approach(0x0fu, JUNCTION_LEFT));
    step(0x00u, 0.0f);
    forward_calls = s_forward_calls;
    s_fail_imu_once = 1u;
    step(0x00u, 0.0f);
    ModeFSM_GetDiagnostics(&d);
    CHECK(d.fsm.state == STATE_FAULT_STOP && d.fsm.fault == MODE_FAULT_IMU);
    CHECK(s_left_command == 0 && s_right_command == 0);
    CHECK(s_forward_calls == forward_calls && s_turn_calls == 0);
    CHECK(d.fault_capture_valid && d.fault_state == STATE_APPROACH_TURN);
    CHECK(d.fault_pwm_left == (int16_t)TURN_APPROACH_BASE_PWM);
    CHECK(d.fault_pwm_right == (int16_t)TURN_APPROACH_BASE_PWM);
    g_imu_ready = 1u;
    step(0x18u, 0.0f);
    CHECK(snapshot().state == STATE_FAULT_STOP);
    CHECK(s_left_command == 0 && s_right_command == 0);
    return 1;
}

static int test_turn_approach_heading_limit_stops_same_frame(void)
{
    int forward_calls;
    ModeFSM_Diagnostics d;
    reset_fixture();
    CHECK(enter_confirmed_turn_approach(0xf0u, JUNCTION_RIGHT));
    step(0x00u, JUNCTION_MAX_HEADING_ERROR);
    CHECK(snapshot().state == STATE_APPROACH_TURN);
    forward_calls = s_forward_calls;
    step(0xffu, JUNCTION_MAX_HEADING_ERROR + 1.0f);
    ModeFSM_GetDiagnostics(&d);
    CHECK(d.fsm.state == STATE_FAULT_STOP && d.fsm.fault == MODE_FAULT_HEADING);
    CHECK(s_left_command == 0 && s_right_command == 0);
    CHECK(s_forward_calls == forward_calls && s_turn_calls == 0);
    CHECK(d.fault_capture_valid && d.fault_state == STATE_APPROACH_TURN);
    CHECK(d.fault_target_ddeg == 0 && d.fault_yaw_valid);
    CHECK(d.fsm.event_count == 1u && d.fsm.last_event == JUNCTION_RIGHT);
    return 1;
}

static int test_cross_skips_extra_turn_approach(void)
{
    ModeFSM_Diagnostics d;
    reset_fixture();
    CHECK(enter_forward_from_cross());
    ModeFSM_GetDiagnostics(&d);
    CHECK(d.fsm.state == STATE_FORWARD && d.advance_frames == 0u);
    CHECK(d.fsm.last_event == JUNCTION_CROSS && d.fsm.event_count == 1u);
    CHECK(s_forward_calls == 5 && s_turn_calls == 0);
    CHECK(close_float(s_last_forward_speed, JUNCTION_BASE_PWM));
    step(0xffu, 0.0f);
    CHECK(snapshot().state == STATE_FORWARD && snapshot().event_count == 1u);
    CHECK(close_float(s_last_forward_speed, JUNCTION_BASE_PWM));
    return 1;
}

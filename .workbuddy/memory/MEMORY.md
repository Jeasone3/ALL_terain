# 项目长期笔记

## 路径规划架构（App/Route/Route.c）

- **混合方案，非单一机制**：道路动作（LEFT/RIGHT/STRAIGHT, project_id=0）靠灰度循迹识别路口推进状态；项目通行（IMU_PASS/TRACK_PASS/TIMED_STOP, project_id=1~9）靠标定时间为主、出口窄线确认为辅。
- 调用链：main → ModeFSM_Tick (TIM4 每 10ms) → Route_Tick(sensor_data, external_command) → 内部 switch 各 RouteState → Car_* 提交请求到运动层 → 电机/PID。
- 关键状态机：READY→FOLLOW(等路口)/ENTRY(入口延时)→APPROACH/CROSSING→TURN(等turn_count变化)→RECOVER(confirm_exit窄线)→finish_step(step_index++)→下一项 begin_step。
- 路口识别：classify_sensor_frame 把八路分成 LEFT/RIGHT/CROSS，update_junction 三帧确认+锁存，只在 ROUTE_FOLLOW 状态消费 junction_event；项目内部(ENTRY/OBSTACLE/RECOVER)屏蔽路口 reset_junction。
- 时间三件套：entry_ms(入口/等路口窗)、pass_ms(前移/最短通行/停车)、timeout_ms(每阶段兜底超时→Car_RouteTimeout)。
- 标定表 g_route_plan[24]，默认时间必须实车标定，0=未标定显示 WAIT_CAL 停车。
- 出口判据 narrow_line：要求 IN3~IN6 内连续 1~3 路在线且含 IN4/IN5，三帧确认 confirm_exit。

# SR04 底层验证

在工程根目录执行 `./tests/sr04/run_tests.ps1`。脚本使用 GCC 编译真实的
`Int_SR04` 驱动，模拟 GPIO、DWT、HAL 时钟和中断屏蔽，产物写入 TEMP。
完整固件编译复用 `./tests/motion/build_keil.ps1`，不烧录设备。

宿主测试覆盖正常回波、12 us 触发脉冲、60 ms 触发间隔、30 ms 超时、缺失和迟到边沿、
Echo 卡高、重复及无关中断、DWT/HAL 计数回绕、系统频率换算和临界区快照。
这些测试不验证实物接线、中断延迟或实际测距精度。

## 上板手动验证

启动流程调用 `Int_SR04_Init()`；自动路线仅在第 12、17 项后的超声波阶段周期测距，
完成一次开环通行并恢复循迹后停止触发。独立验证底层时，可按需调用
`Int_SR04_Trigger()`；返回 1 后，在后续前台执行过程中调用
`Int_SR04_GetResult(&result)`，只有 `result.status == SR04_OK` 时使用
`result.distance_cm`。读取结果不会再次触发测距，也不会消耗结果；新请求接受后立即变为
`SR04_BUSY`，非成功状态的回波宽度和距离均为 0。

- 沿用 Trig→PA5、Echo→PA4；当前 GPIO 和 EXTI4 配置无需修改。
- 对准已知距离的平面，比较返回的 `pulse_us` 和 `distance_cm`。
- 无目标时确认能返回 `SR04_TIMEOUT`，等待足够触发间隔后可重新测量。
- 开启现有运动控制，检查测距结果和控制节拍；驱动不启停或改写 TIM4。
- 使用逻辑分析仪确认 Trig 脉宽及 Echo 边沿。不要用暂停 CPU 后的 DWT 数值判断实时时序。

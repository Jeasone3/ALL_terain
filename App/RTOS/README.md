# FreeRTOS 三任务运行说明

当前分支为 `FR`，内核来自用户提供的 FreeRTOS V10.5.1，采用原生 API、`RVDS/ARM_CM3` 移植层和 `heap_4`。Keil 仍使用 ARM Compiler 5，无需安装新的内核依赖。

## 执行顺序

1. [main.c](E:/Embedded_Development/All-terrain/ALL_terain_For_11/Core/Src/main.c) 初始化时钟、外设、电机、控制器及 IMU，完成静止校准，然后按原有测试开关提交一次启动请求。
2. [App_RTOS.c](E:/Embedded_Development/All-terrain/ALL_terain_For_11/App/RTOS/App_RTOS.c) 创建全部任务，再启动调度器。控制任务启动 TIM4，并阻塞等待通知。
3. TIM4 每 10 ms 只通知 `ControlTask`，由任务执行一次 `ModeFSM_Tick()`，保持灰度、两次请求处理、Route、IMU 与 PID 的原有顺序。
4. `DisplayTask` 独占 OLED，每 100 ms 处理显示，失败后每 1 s 重连；`RecoveryTask` 每 1 s 检查并重试不可用的 IMU。

| 任务 | 入口源码 | 优先级 | 栈深度 |
|---|---|---:|---:|
| `ControlTask` | [Control_Task.c](E:/Embedded_Development/All-terrain/ALL_terain_For_11/App/RTOS/Control_Task.c) | 3 | 512 字，2048 字节 |
| `DisplayTask` | [Display_Task.c](E:/Embedded_Development/All-terrain/ALL_terain_For_11/App/RTOS/Display_Task.c) | 2 | 768 字，3072 字节 |
| `RecoveryTask` | [Recovery_Task.c](E:/Embedded_Development/All-terrain/ALL_terain_For_11/App/RTOS/Recovery_Task.c) | 1 | 384 字，1536 字节 |

只有控制任务调用运动控制器；Route 仍是控制任务内的决策模块。OLED 初始化也在显示任务中，主循环不再重复执行显示或重连。

## 节拍和错误处理

[FreeRTOSConfig.h](E:/Embedded_Development/All-terrain/ALL_terain_For_11/Core/Inc/FreeRTOSConfig.h) 设定 1 ms 系统节拍、12 KiB 堆，以及四位 NVIC 抢占优先级。TIM4 优先级为 5，系统调用中断优先级边界为 5，内核异常优先级为 15。

SysTick 启动前只更新 HAL 时间，调度器启动后同时更新 FreeRTOS 时间。SVC 和 PendSV 由移植层提供。当前没有使用示例中的 TIM7 HAL 时间基准，也没有开启软件定时器。

控制任务收到多个积压通知，或测得一帧执行时间达到 10 ms，立即调用 `ModeFSM_ControlPeriodFault()`：保存现场，清除旧运动请求，中止路线、关闭路线短鸣、输出零 PWM。OLED 显示 `CTRL_TO`，后续不连续补跑 PID。故障后的显式新动作才允许重新尝试。

IMU 初始化只在 `g_imu_ready=0` 时执行，全部 I2C 事务完成后才发布就绪。恢复任务不重复写回函数返回值，不重新校准，也不自动恢复运动。现有 100 ms 复位等待位于低优先级任务，允许控制任务抢占。

分配失败、栈溢出和内核断言进入 `AppRTOS_Fatal()`：直接把两轮 PWM 比较寄存器置零，关闭中断并停在循环中。此时查看 `g_rtos_fatal_file` 和 `g_rtos_fatal_line`，不要期待 OLED 继续刷新。

## 调试器观察值

`g_rtos_stats` 可直接加入 Keil Watch：

| 字段 | 单位与含义 |
|---|---|
| `control_frames` | 已处理的控制唤醒次数 |
| `control_last_us` / `control_max_us` | 最近与最大单帧耗时，微秒 |
| `wake_interval_last_us` / `wake_interval_max_us` | 最近与最大任务唤醒间隔，微秒 |
| `control_period_faults` | 通知积压或执行超期次数 |
| `control_stack_free_words` | 控制任务历史最小剩余栈，字 |
| `display_stack_free_words` | 显示任务历史最小剩余栈，字 |
| `recovery_stack_free_words` | 恢复任务历史最小剩余栈，字 |
| `free_heap_bytes` | 最近采样的内核剩余堆，字节 |

栈水位和剩余堆由显示任务每秒采样。DWT 测量使用现有自由运行的 CYCCNT，不重置计数器，避免影响软件 I2C 的微秒延时。初始栈值需要覆盖实际路线、断线和显示错误路径后再判断是否充足。

## 软件验证

使用本机可运行的 CLion GCC 执行本次迁移专项：

```powershell
& 'E:\Embedded_Development\All-terrain\ALL_terain_For_11\tests\motion\run_tests.ps1' `
    -Gcc 'D:\CoderSpcace\IDE\CLion 2026.2.3\bin\mingw\bin\gcc.exe' -RtosOnly
```

该入口编译真实任务循环和 SysTick/TIM4 回调，以内核 API 替身检查任务创建、通知、恰好 10 ms 截止、DWT 回绕、OLED 重连和 IMU 就绪发布；另以真实 FSM、Route、PID 验证节拍故障、末尾 3 秒停车及中止行为。默认九段、全部段与截止八段分别编译，日志和姿态模块也独立回归。

旧全量测试包含过期的实车调参数值：基线仍断言循迹 PWM 为 400，而源码为 500；旧第九段测试仍断言入口 1000 ms、通行 5000 ms，而当前表项为 1300 ms、3000 ms。本次不回退用户参数来使这些断言通过，也不把专项通过表述为旧全量测试通过。

## 上板验证顺序

先架空车轮，确认任务持续调度、HAL 计时正常和两轮停车行为，再依次验证纯循迹、限时 IMU 直行、原路线、OLED 断线及 IMU 断线。观察最大控制耗时和三个任务的剩余栈，确认存在稳定余量。

本次软件测试不能证明真实调度抖动、软件 I2C 在中断下的稳定性或整车通过能力；这些仍待上板测量。

当前为手动内核移植，`.ioc` 尚未描述 FreeRTOS 接入。若用 CubeMX 重新生成工程，应核对 SVC、PendSV、SysTick、任务文件分组和包含路径，避免重新生成空异常入口覆盖本次接入。

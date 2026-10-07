# 状态机主机测试

本测试直接编译 `App/Mode_FSM/Mode_FSM.c`，并使用真实的 `Mode_FSM.h`。`stubs/` 仅替换硬件及控制接口，不复制状态机判据。

在工程根目录执行：

```powershell
powershell -ExecutionPolicy Bypass -File .\tests\run_mode_fsm_tests.ps1
```

编译器没有加入环境变量时可指定其完整路径：

```powershell
powershell -ExecutionPolicy Bypass -File .\tests\run_mode_fsm_tests.ps1 -CompilerPath 'D:\CoderSpcace\IDE\mingw64\bin\gcc.exe'
```

脚本分别以 `LINE_RAW_VALUE=1` 和 `LINE_RAW_VALUE=0` 编译、运行相同样例，并开启编译警告检查。还会直接链接真实的角度控制和 PID，验证低速纠偏、正负航向扰动下轮子不反转，以及转弯后目标航向保持。构建产物保存在输出所示的系统临时目录，不删除文件、不提交代码。

可用 `-ArmCompilerPath` 指定 `arm-none-eabi-gcc.exe`，同时检查本轮修改的五个源码文件。脚本从 Keil 工程读取真实头文件目录，ARM 检查不使用替身，只做语法检查，不写入工程的 Keil 构建产物。

测试覆盖路口候选确认、噪声取消、两侧先后触发的十字、左右转目标及连续到位确认、旧证据失效、驶离后重新解锁、独立路口计数、失线及转弯超时、IMU 失效当帧停车、循迹 PID 清理、日志队列与中断屏蔽状态恢复。

主机测试验证状态转换和输出命令。传感器阈值、实际线宽、车辆移动距离、左右转 yaw 符号以及中断最长耗时仍需在车辆上测量。

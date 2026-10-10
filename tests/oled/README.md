# OLED 主机测试

在工程根目录运行：

```powershell
./tests/oled/run_tests.ps1
```

默认使用本机 `D:/CoderSpcace/IDE/mingw64/bin/gcc.exe`，也可以传入 `-Gcc` 指定 C99 编译器。编译启用 `-Wall -Wextra -Werror -pedantic`，产物保存到临时目录，不自动删除文件。

`test_oled_core.c` 编译真实绘制与字库代码，以模拟 I2C 解析页地址和数据传输。测试通过公开 API 检查屏幕像素，不读取生产代码的内部显存。覆盖初始化、未初始化保护、批量刷新、脏列区间、通信错误后的页级重试、完整 ASCII 字库、非法字符、负数和小数格式化、整行替换、图形及位图裁剪，以及坐标和尺寸极值。

`test_oled_port.c` 编译真实 STM32 平台适配层，仅替换 HAL，检查七位地址转换、控制字、批量长度、超时设置及 HAL 错误映射。脚本分别检查默认配置、地址与超时宏覆盖，以及非法七位地址配置。

这些测试验证软件行为；实际控制器型号、模组寻址兼容性、I2C 时序、上电效果和刷新耗时仍需板上验收。

本机 Keil 构建验证：

```powershell
./tests/oled/build_keil.ps1
```

脚本从当前工程生成无 BOM 的 XML 验证副本，源码及编译选项与原工程一致，输出位于 `MDK-ARM/.vscode/oled-validation/build/`。只构建、不下载固件，不覆盖原工程已跟踪的产物，也不自动删除文件。可用 `-Uv4` 指定其他 Keil 安装路径。

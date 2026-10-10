# OLED 驱动使用与移植

当前支持 128×64 单色 I2C 屏幕、单屏、单调用者。业务代码只包含 `Int_OLED.h`；绘制操作只修改显存，完成一帧后调用 `OLED_Update()`。

## 最常用的调用

```c
#include "Int_OLED.h"

/* 初始化及通信只在主循环或同一个显示任务中调用。 */
OLED_Status status = OLED_Init();
if (status == OLED_OK)
{
    OLED_PrintLine(0, "Jeason FSM");
    OLED_PrintLine(2, "State:%s", "TURN_RIGHT");
    OLED_PrintLine(4, "Target:%.1f", -90.0);
    status = OLED_Update();
}
```

`OLED_PrintLine()` 使用 6×8 字体，行号为 `0`～`7`，对应像素纵坐标 `row * 8`。每行最多显示 21 个 ASCII 字符；超过部分截断，右侧剩余两个像素清黑。每次调用替换整行，长文字变短时不会留下旧字符；非法行号和空格式指针不绘制。

格式化使用标准 `vsnprintf`，支持 `%s`、`%d`、`%u`、`%.1f` 等 C 格式，遵循格式与参数类型匹配规则。`float` 可变参数会自动提升为 `double`；使用 `%lu` 输出 `uint32_t` 时应转换为 `unsigned long`，例如：

```c
OLED_PrintLine(6, "Frames:%lu", (unsigned long)frame_count);
```

浮点格式化会增加标准库代码及栈开销；移植到其他编译器时需要启用其浮点 `printf` 支持，并检查目标工程链接结果与栈预算。格式字符串应是开发者提供的字符串常量，格式宽度和精度使用屏幕实际需要的有限值。

## 像素坐标、字体与绘图

原点在屏幕左上角，`x` 向右、`y` 向下。所有带 `x/y` 的接口统一使用像素，不使用页号。坐标可以为负数，屏幕范围外的像素被裁剪；字符与位图可绘制在不按八像素对齐的位置。

```c
/* 清屏也只修改显存，最后统一刷新。 */
OLED_Clear();
OLED_DrawString(0, 0, "Large", OLED_FONT_8X16);
OLED_DrawString(0, 20, "Small", OLED_FONT_6X8);
OLED_DrawPixel(127, 63, OLED_WHITE);
OLED_DrawLine(0, 31, 127, 31, OLED_WHITE);
OLED_DrawRect(0, 34, 40, 20, OLED_WHITE);
OLED_FillRect(45, 34, 20, 20, OLED_WHITE);
OLED_DrawCircle(82, 44, 9, OLED_WHITE);
OLED_FillCircle(108, 44, 9, OLED_WHITE);
OLED_ClearRect(48, 37, 4, 4);
status = OLED_Update();
```

- `OLED_WHITE` 点亮像素，`OLED_BLACK` 清黑像素；矩形参数 `width/height` 是像素数量。
- 字体通过 `OLED_FONT_6X8` 和 `OLED_FONT_8X16` 选择。字符背景覆盖为黑色，字符串不自动换行，越过右边界的部分不显示。
- 可打印 ASCII `32`～`126` 均可显示，其他字节显示为 `?`；第一版不包含中文和 UTF-8 解码。无效字体和空文本指针不绘制。
- 矩形、圆及其填充只修改显存，`OLED_ClearRect()` 用于清除指定矩形区域。
- `OLED_SetDisplay(0/1)` 和 `OLED_SetContrast(0～255)` 直接发送控制命令并返回通信状态。

## 单色位图

`OLED_DrawBitmap(x, y, width, height, data, data_length)` 使用纵向八像素一页、页内按列排列的格式：

```text
data[page * width + column]
bit0 -> page * 8 + 0 行像素
bit7 -> page * 8 + 7 行像素
```

`data_length` 必须至少为 `width * ((height + 7) / 8)` 字节；尾页超出图片高度的位被忽略。位图矩形内黑白均覆盖，图片在屏幕外的部分裁剪。参数或数据长度无效时返回 `OLED_INVALID_ARGUMENT`，不修改显存。

```c
/* 8×8 图片，每个字节是一列；bit0 对应最上方像素。 */
static const uint8_t icon[] = {
    0x3C, 0x42, 0xA5, 0x81, 0xA5, 0x99, 0x42, 0x3C
};
status = OLED_DrawBitmap(110, 0, 8, 8, icon, sizeof(icon));
if (status == OLED_OK)
{
    status = OLED_Update();
}
```

## 刷新、错误与调用边界

显存为 1024 字节静态数组，驱动不使用动态内存；字库及固定图片使用 `const`。更新按页批量发送发生变化的列区间，无变化时不发送。同样内容重复写入同一行后刷新不会产生 I2C 通信。

| 返回值 | 含义 |
| --- | --- |
| `OLED_OK` | 操作成功，或刷新时没有待发送内容 |
| `OLED_IO_ERROR` | 平台通信失败，如设备未应答 |
| `OLED_TIMEOUT` | 平台通信超时 |
| `OLED_BUSY` | 平台总线忙 |
| `OLED_INVALID_ARGUMENT` | 参数无效 |
| `OLED_NOT_INITIALIZED` | 尚未成功初始化 |

一次刷新发生首次通信失败后立即返回，未完成脏区保留，后续 `OLED_Update()` 可继续重试。初始化失败后应重试 `OLED_Init()`；刷新失败不必每次重新初始化。

OLED 使用阻塞 I2C，绘制、格式化、刷新以及开关屏操作应由主循环或同一个显示任务统一负责，不在控制中断里调用，也不由多个任务同时调用。读取中断维护的多个显示字段时，先在短暂关中断区间取得一致快照，并恢复进入前的中断状态；格式化和通信放在快照之后。

本工程主程序每 100 ms 显示 `State` 和目标角 `Target`，初始化失败每 1 s 重试，刷新失败下个显示周期重试。`g_oled_status` 可在调试器中观察。屏幕故障只记录状态，控制系统仍继续初始化和运行。

## 移植步骤

驱动分为显示绘制、屏幕控制、平台适配三层。移植到其他 MCU 时保留 `Int_OLED.c/.h` 和 `oledfont.c/.h`，重写 `OLED_Port.c/.h` 的以下两项：

```c
OLED_Status OLED_Port_Write(uint8_t control,
                            const uint8_t *data, uint16_t length);
void OLED_Port_DelayMs(uint32_t milliseconds);
```

`control` 为 `0x00` 时发送命令，为 `0x40` 时发送显存数据；`length` 是有效负载长度。平台层应一次批量发送，保证函数返回前负载发送完成，并把平台的成功、失败、超时和忙映射到对应 `OLED_Status`。

当前适配层使用 STM32 HAL 的 I2C1，保持工程现有 PB8/PB9、100 kHz 配置。`OLED_I2C_ADDRESS` 配置为七位地址 `0x3C`，HAL 所需的左移仅在 `OLED_Port.c` 中处理；`OLED_I2C_TIMEOUT_MS` 默认 100 ms。其他平台应按其 API 的地址规则处理，不在业务代码中保存 HAL 地址。

将 `Int_OLED.c`、`OLED_Port.c`、`oledfont.c` 加入目标工程编译，并加入本目录的头文件搜索路径。公共入口 `Int_OLED.h` 不依赖 HAL，可以用于主机测试。当前控制层沿用现有模组的电源、扫描方向、时钟和对比度配置，修正寻址命令为 `0x20, 0x02`，使其与按页刷新一致。[SSD1306 原厂数据手册第 9 节和第 10.1.3 节](https://www.orientdisplay.com/wp-content/uploads/2020/07/SSD1306.pdf)规定页寻址的低两位为二进制 `10`；旧参数 `0x10` 的低两位为 `00`。实际模组控制器仍需板上确认，更换屏幕控制器需调整 `Int_OLED.c` 中的初始化命令和页/列寻址，不能只替换 I2C 发送函数。

移植完成后在板上检查四条屏幕边界、偶数列位置、两种字体、负角度和小数显示、刷新耗时，以及断开 OLED 后的错误返回与恢复。主机测试与 Keil 构建不替代这些板上验收。

本次实际执行结果与资源占用见 [VERIFICATION.md](VERIFICATION.md)。

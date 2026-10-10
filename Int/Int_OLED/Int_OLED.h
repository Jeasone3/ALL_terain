#ifndef INT_OLED_H
#define INT_OLED_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OLED_WIDTH       128U
#define OLED_HEIGHT       64U
#define OLED_PAGE_COUNT    8U
#define OLED_LINE_CHARS   21U

typedef enum {
    OLED_OK = 0,    /*操作成功，或刷新时没有待发送内容 */
    OLED_IO_ERROR,  /*  平台通信失败，如设备未应答 */
    OLED_TIMEOUT,   /*  平台通信超时 */
    OLED_BUSY,  /*  平台通信忙，未能完成传输 */
    OLED_INVALID_ARGUMENT,  /*  参数无效，如坐标超出屏幕范围 */
    OLED_NOT_INITIALIZED   /*  未初始化 */
} OLED_Status;

typedef enum {
    OLED_FONT_6X8 = 0,
    OLED_FONT_8X16
} OLED_Font;

typedef enum {
    OLED_BLACK = 0,
    OLED_WHITE = 1
} OLED_Color;

/* 单屏、单调用者：全部函数在主循环或同一个显示任务中调用，不在中断中调用。 */
/* 初始化会延时、发送命令并清屏；返回失败时可再次调用初始化。 */
OLED_Status OLED_Init(void);
OLED_Status OLED_Update(void);
OLED_Status OLED_SetDisplay(uint8_t enabled);
OLED_Status OLED_SetContrast(uint8_t contrast);

/* 下列绘制操作只修改静态显存，最后调用 OLED_Update 才显示。 */
void OLED_Clear(void);
void OLED_DrawPixel(int16_t x, int16_t y, OLED_Color color);
void OLED_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                   OLED_Color color);
void OLED_DrawRect(int16_t x, int16_t y, uint16_t width, uint16_t height,
                   OLED_Color color);
void OLED_FillRect(int16_t x, int16_t y, uint16_t width, uint16_t height,
                   OLED_Color color);
void OLED_ClearRect(int16_t x, int16_t y, uint16_t width, uint16_t height);
void OLED_DrawCircle(int16_t x, int16_t y, uint16_t radius, OLED_Color color);
void OLED_FillCircle(int16_t x, int16_t y, uint16_t radius, OLED_Color color);

/* 坐标统一为像素；字符背景覆盖为黑色；不自动换行，屏幕外部分裁剪。 */
/* ASCII 32～126 可显示，其他字节替换为 '?'；无效字体和空指针不绘制。 */
void OLED_DrawChar(int16_t x, int16_t y, char character, OLED_Font font);
void OLED_DrawString(int16_t x, int16_t y, const char *text, OLED_Font font);

/* 行号 0～7，固定 6×8 字体；覆盖整行，格式化后最多显示 21 个字符。 */
/* 使用标准 printf 格式，包括 %d、%u、%s、%.1f；格式与参数类型需匹配。 */
void OLED_PrintLine(uint8_t row, const char *format, ...);

/* 位图按八行一页、页内按列排列，bit0 是该页最上方像素。 */
/* data_length 至少为 width * ((height + 7) / 8)，尾页屏幕外位忽略。 */
/* 图像矩形内黑白均覆盖；参数无效时返回错误且不修改显存。 */
OLED_Status OLED_DrawBitmap(int16_t x, int16_t y,
                            uint16_t width, uint16_t height,
                            const uint8_t *data, size_t data_length);

#ifdef __cplusplus
}
#endif

#endif /* INT_OLED_H */

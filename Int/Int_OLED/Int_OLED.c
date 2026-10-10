/**
 * @file Int_OLED.c
 * @author Jeason
 * @brief 
 * @version 0.1
 * @date 2026-10-10
 * 
 * @copyright Copyright (c) 2026、
 * /
//使用说明：
//#include "Int_OLED.h"
// /* 初始化及通信只在主循环或同一个显示任务中调用。 */
// OLED_Status status = OLED_Init();
// if (status == OLED_OK)
// {
//     OLED_PrintLine(0, "Jeason FSM");
//     OLED_PrintLine(2, "State:%s", "TURN_RIGHT");
//     OLED_PrintLine(4, "Target:%.1f", -90.0);
//     status = OLED_Update();
//}

/*
主要API：
`OLED_PrintLine()`:
    在指定行打印格式化字符串，覆盖整行，最多显示 21 个字符。
    使用标准 printf 格式，包括 %d、%u、%s、%.1f；格式与参数类型需匹配。
    行号 0～7，固定 6×8 字体。行号 8～15，固定 8×16 字体。
 

    
 */


#include "Int_OLED.h"
#include "OLED_Port.h"
#include "oledfont.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define OLED_COMMAND 0x00U
#define OLED_DATA    0x40U
#define OLED_CLEAN_COLUMN OLED_WIDTH

/* 显存按页排列，每个字节是一列中的八个像素，bit0 在上方。 */
static uint8_t s_framebuffer[OLED_PAGE_COUNT][OLED_WIDTH];
static uint8_t s_dirty_min[OLED_PAGE_COUNT];
static uint8_t s_dirty_max[OLED_PAGE_COUNT];
static uint8_t s_initialized;

/* 保持现有模组的电源、扫描方向、时钟、对比度等初始化配置。 */
/* 0x20 的参数用 0x02 选择页寻址，匹配下面的 B0～B7 页刷新命令。 */
static const uint8_t s_init_commands[] = {
    0xAE, 0x20, 0x02, 0xB0, 0xC8, 0x00, 0x10, 0x40,
    0x81, 0xFF, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3,
    0x00, 0xD5, 0xF0, 0xD9, 0x22, 0xDA, 0x12, 0xDB,
    0x20, 0x8D, 0x14, 0xAF, 0x8D, 0x14, 0xAF
};

static uint8_t valid_color(OLED_Color color)
{
    return (uint8_t)(color == OLED_BLACK || color == OLED_WHITE);
}

static void write_buffer_byte(uint8_t page, uint8_t column, uint8_t value)
{
    if (s_framebuffer[page][column] == value)
    {
        return;
    }

    s_framebuffer[page][column] = value;
    if (column < s_dirty_min[page])
    {
        s_dirty_min[page] = column;
    }
    if (column > s_dirty_max[page])
    {
        s_dirty_max[page] = column;
    }
}


static void draw_pixel(int32_t x, int32_t y, OLED_Color color)
{
    uint8_t page;
    uint8_t mask;
    uint8_t value;

    if (x < 0 || x >= (int32_t)OLED_WIDTH ||
        y < 0 || y >= (int32_t)OLED_HEIGHT)
    {
        return;
    }

    page = (uint8_t)(y / 8);
    mask = (uint8_t)(1U << (uint32_t)(y % 8));
    value = s_framebuffer[page][x];
    if (color == OLED_WHITE)
    {
        value = (uint8_t)(value | mask);
    }
    else
    {
        value = (uint8_t)(value & (uint8_t)~mask);
    }
    write_buffer_byte(page, (uint8_t)x, value);
}

static void draw_horizontal(int32_t x0, int32_t x1, int32_t y,
                            OLED_Color color)
{
    int32_t x;

    if (y < 0 || y >= (int32_t)OLED_HEIGHT ||
        x1 < 0 || x0 >= (int32_t)OLED_WIDTH)
    {
        return;
    }
    if (x0 < 0) { x0 = 0; }
    if (x1 >= (int32_t)OLED_WIDTH) { x1 = (int32_t)OLED_WIDTH - 1; }
    for (x = x0; x <= x1; ++x)
    {
        draw_pixel(x, y, color);
    }
}

static void draw_vertical(int32_t x, int32_t y0, int32_t y1,
                          OLED_Color color)
{
    int32_t y;

    if (x < 0 || x >= (int32_t)OLED_WIDTH ||
        y1 < 0 || y0 >= (int32_t)OLED_HEIGHT)
    {
        return;
    }
    if (y0 < 0) { y0 = 0; }
    if (y1 >= (int32_t)OLED_HEIGHT) { y1 = (int32_t)OLED_HEIGHT - 1; }
    for (y = y0; y <= y1; ++y)
    {
        draw_pixel(x, y, color);
    }
}

static uint8_t font_width(OLED_Font font)
{
    if (font == OLED_FONT_6X8) { return 6U; }
    if (font == OLED_FONT_8X16) { return 8U; }
    return 0U;
}

static uint8_t glyph_index(unsigned char character)
{
    if (character < 32U || character > 126U)
    {
        character = (unsigned char)'?';
    }
    return (uint8_t)(character - 32U);
}

static void draw_character(int32_t x, int32_t y, unsigned char character,
                           OLED_Font font)
{
    uint8_t width = font_width(font);
    uint8_t height = (font == OLED_FONT_8X16) ? 16U : 8U;
    uint8_t index = glyph_index(character);
    const uint8_t *glyph;
    uint8_t column;
    uint8_t row;

    if (width == 0U || x >= (int32_t)OLED_WIDTH ||
        x + width <= 0 || y >= (int32_t)OLED_HEIGHT || y + height <= 0)
    {
        return;
    }

    glyph = (font == OLED_FONT_6X8) ? OLED_Font6x8[index] :
                                    OLED_Font8x16[index];
    for (column = 0; column < width; ++column)
    {
        for (row = 0; row < height; ++row)
        {
            uint8_t bits = glyph[(row / 8U) * width + column];
            OLED_Color color = ((bits >> (row % 8U)) & 1U) != 0U ?
                               OLED_WHITE : OLED_BLACK;
            draw_pixel(x + column, y + row, color);
        }
    }
}

/* 无浮点的整数平方根，最多十六轮；圆的计算量与可见屏幕大小相关。 */
static uint32_t integer_sqrt(uint32_t value)
{
    uint32_t root = 0U;
    uint32_t bit = (uint32_t)1U << 30U;

    while (bit > value)
    {
        bit >>= 2U;
    }
    while (bit != 0U)
    {
        if (value >= root + bit)
        {
            value -= root + bit;
            root = (root >> 1U) + bit;
        }
        else
        {
            root >>= 1U;
        }
        bit >>= 2U;
    }
    return root;
}

static uint8_t circle_visible(int32_t x, int32_t y, uint16_t radius)
{
    return (uint8_t)(x + radius >= 0 && x - radius < (int32_t)OLED_WIDTH &&
                     y + radius >= 0 && y - radius < (int32_t)OLED_HEIGHT);
}

/**
 * @brief 初始化OLED显示屏
 * 
 * @return 返回OLED状态
 */
OLED_Status OLED_Init(void)
{
    OLED_Status status;
    uint8_t page;

    s_initialized = 0U;
    memset(s_framebuffer, 0, sizeof(s_framebuffer));
    for (page = 0; page < OLED_PAGE_COUNT; ++page)
    {
        /* 即使本地显存为零，也必须覆盖上电后的未知屏幕内容。 */
        s_dirty_min[page] = 0U;
        s_dirty_max[page] = (uint8_t)(OLED_WIDTH - 1U);
    }

    OLED_Port_DelayMs(100U);
    status = OLED_Port_Write(OLED_COMMAND, s_init_commands,
                             (uint16_t)sizeof(s_init_commands));
    if (status != OLED_OK)
    {
        return status;
    }

    s_initialized = 1U;
    status = OLED_Update();
    if (status != OLED_OK)
    {
        s_initialized = 0U;
    }
    return status;
}

/**
 * @brief 更新OLED显示屏，将显存内容写入屏幕
 * 
 * @return OLED_Status 
 */
OLED_Status OLED_Update(void)
{
    uint8_t page;

    if (s_initialized == 0U)
    {
        return OLED_NOT_INITIALIZED;
    }

    for (page = 0; page < OLED_PAGE_COUNT; ++page)
    {
        uint8_t first = s_dirty_min[page];
        uint8_t commands[3];
        uint16_t length;
        OLED_Status status;

        if (first == OLED_CLEAN_COLUMN)
        {
            continue;
        }

        /* 低列地址直接取低四位，不强制 bit0，也不引入隐藏的列偏移。 */
        commands[0] = (uint8_t)(0xB0U | page);
        commands[1] = (uint8_t)(first & 0x0FU);
        commands[2] = (uint8_t)(0x10U | (first >> 4U));
        status = OLED_Port_Write(OLED_COMMAND, commands, sizeof(commands));
        if (status != OLED_OK)
        {
            return status;
        }

        length = (uint16_t)(s_dirty_max[page] - first + 1U);
        status = OLED_Port_Write(OLED_DATA, &s_framebuffer[page][first], length);
        if (status != OLED_OK)
        {
            return status;
        }

        /* 命令和整段数据均成功后才能清除脏标记。 */
        s_dirty_min[page] = (uint8_t)OLED_CLEAN_COLUMN;
        s_dirty_max[page] = 0U;
    }
    return OLED_OK;
}

OLED_Status OLED_SetDisplay(uint8_t enabled)
{
    const uint8_t on[] = {0x8D, 0x14, 0xAF};
    const uint8_t off[] = {0x8D, 0x10, 0xAE};

    if (enabled > 1U)
    {
        return OLED_INVALID_ARGUMENT;
    }
    if (s_initialized == 0U)
    {
        return OLED_NOT_INITIALIZED;
    }
    return OLED_Port_Write(OLED_COMMAND, enabled != 0U ? on : off, sizeof(on));
}

OLED_Status OLED_SetContrast(uint8_t contrast)
{
    uint8_t commands[2] = {0x81, 0};

    if (s_initialized == 0U)
    {
        return OLED_NOT_INITIALIZED;
    }
    commands[1] = contrast;
    return OLED_Port_Write(OLED_COMMAND, commands, sizeof(commands));
}

void OLED_Clear(void)
{
    uint8_t page;
    uint8_t column;

    for (page = 0; page < OLED_PAGE_COUNT; ++page)
    {
        for (column = 0; column < OLED_WIDTH; ++column)
        {
            write_buffer_byte(page, column, 0U);
        }
    }
}

/**
 * @brief 画点
 * 
 * @param x 
 * @param y 
 * @param color 
 */
void OLED_DrawPixel(int16_t x, int16_t y, OLED_Color color)
{
    if (valid_color(color) != 0U)
    {
        draw_pixel(x, y, color);
    }
}


/**
 * @brief 画线
 * 
 * @param x0 起点
 * @param y0 
 * @param x1 终点
 * @param y1 
 * @param color 颜色
 */
void OLED_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                   OLED_Color color)
{
    int32_t x = x0;
    int32_t y = y0;
    int32_t dx = (int32_t)x1 - x0;
    int32_t dy = (int32_t)y1 - y0;
    int32_t sx = (dx >= 0) ? 1 : -1;
    int32_t sy = (dy >= 0) ? 1 : -1;
    int32_t error;

    if (valid_color(color) == 0U ||
        (x0 < 0 && x1 < 0) || (x0 >= (int32_t)OLED_WIDTH && x1 >= (int32_t)OLED_WIDTH) ||
        (y0 < 0 && y1 < 0) || (y0 >= (int32_t)OLED_HEIGHT && y1 >= (int32_t)OLED_HEIGHT))
    {
        return;
    }
    if (dx < 0) { dx = -dx; }
    if (dy > 0) { dy = -dy; }
    error = dx + dy;
    for (;;)
    {
        int32_t twice_error;
        draw_pixel(x, y, color);
        if (x == x1 && y == y1)
        {
            break;
        }
        twice_error = 2 * error;
        if (twice_error >= dy) { error += dy; x += sx; }
        if (twice_error <= dx) { error += dx; y += sy; }
    }
}

/**
 * @brief 空心矩形
 * 
 * @param x 
 * @param y 
 * @param width 
 * @param height 
 * @param color 
 */
void OLED_DrawRect(int16_t x, int16_t y, uint16_t width, uint16_t height,
                   OLED_Color color)
{
    int32_t right;
    int32_t bottom;

    if (width == 0U || height == 0U || valid_color(color) == 0U)
    {
        return;
    }
    right = (int32_t)x + width - 1;
    bottom = (int32_t)y + height - 1;
    draw_horizontal(x, right, y, color);
    draw_horizontal(x, right, bottom, color);
    draw_vertical(x, y, bottom, color);
    draw_vertical(right, y, bottom, color);
}

/**
 * @brief 实心矩形
 * 
 * @param x 
 * @param y 
 * @param width 
 * @param height 
 * @param color 
 */
void OLED_FillRect(int16_t x, int16_t y, uint16_t width, uint16_t height,
                   OLED_Color color)
{
    int32_t left = x;
    int32_t top = y;
    int32_t right = (int32_t)x + width;
    int32_t bottom = (int32_t)y + height;
    int32_t column;
    int32_t page;

    if (width == 0U || height == 0U || valid_color(color) == 0U)
    {
        return;
    }
    if (left < 0) { left = 0; }
    if (top < 0) { top = 0; }
    if (right > (int32_t)OLED_WIDTH) { right = (int32_t)OLED_WIDTH; }
    if (bottom > (int32_t)OLED_HEIGHT) { bottom = (int32_t)OLED_HEIGHT; }
    if (left >= right || top >= bottom)
    {
        return;
    }

    for (page = top / 8; page <= (bottom - 1) / 8; ++page)
    {
        uint32_t low = (top > page * 8) ? (uint32_t)(top - page * 8) : 0U;
        uint32_t high = (bottom < page * 8 + 8) ?
                         (uint32_t)(bottom - page * 8) : 8U;
        uint8_t mask = (uint8_t)((0xFFU << low) & (0xFFU >> (8U - high)));
        for (column = left; column < right; ++column)
        {
            uint8_t value = s_framebuffer[page][column];
            value = (color == OLED_WHITE) ? (uint8_t)(value | mask) :
                                          (uint8_t)(value & (uint8_t)~mask);
            write_buffer_byte((uint8_t)page, (uint8_t)column, value);
        }
    }
}

/**
 * @brief  清空矩形区域
 * 
 * @param x 
 * @param y 
 * @param width 
 * @param height 
 */
void OLED_ClearRect(int16_t x, int16_t y, uint16_t width, uint16_t height)
{
    OLED_FillRect(x, y, width, height, OLED_BLACK);
}

/**
 * @brief 空心圆
 * 
 * @param x 
 * @param y 
 * @param radius 
 * @param color 
 */
void OLED_DrawCircle(int16_t x, int16_t y, uint16_t radius, OLED_Color color)
{
    uint32_t square = (uint32_t)radius * radius;
    int32_t row;
    int32_t column;

    if (valid_color(color) == 0U || circle_visible(x, y, radius) == 0U)
    {
        return;
    }

    /* 分别按行和列取圆周端点，避免只按行采样造成陡峭圆弧缺口。 */
    for (row = 0; row < (int32_t)OLED_HEIGHT; ++row)
    {
        int32_t delta = row - y;
        uint32_t span;
        if (delta < 0) { delta = -delta; }
        if ((uint32_t)delta > radius) { continue; }
        span = integer_sqrt(square - (uint32_t)delta * (uint32_t)delta);
        draw_pixel((int32_t)x - (int32_t)span, row, color);
        draw_pixel((int32_t)x + (int32_t)span, row, color);
    }
    for (column = 0; column < (int32_t)OLED_WIDTH; ++column)
    {
        int32_t delta = column - x;
        uint32_t span;
        if (delta < 0) { delta = -delta; }
        if ((uint32_t)delta > radius) { continue; }
        span = integer_sqrt(square - (uint32_t)delta * (uint32_t)delta);
        draw_pixel(column, (int32_t)y - (int32_t)span, color);
        draw_pixel(column, (int32_t)y + (int32_t)span, color);
    }
}

/**
 * @brief 实心圆
 * 
 * @param x 
 * @param y 
 * @param radius 
 * @param color 
 */
void OLED_FillCircle(int16_t x, int16_t y, uint16_t radius, OLED_Color color)
{
    uint32_t square = (uint32_t)radius * radius;
    int32_t row;

    if (valid_color(color) == 0U || circle_visible(x, y, radius) == 0U)
    {
        return;
    }
    for (row = 0; row < (int32_t)OLED_HEIGHT; ++row)
    {
        int32_t delta = row - y;
        uint32_t span;
        if (delta < 0) { delta = -delta; }
        if ((uint32_t)delta > radius) { continue; }
        span = integer_sqrt(square - (uint32_t)delta * (uint32_t)delta);
        draw_horizontal((int32_t)x - (int32_t)span,
                         (int32_t)x + (int32_t)span, row, color);
    }
}

void OLED_DrawChar(int16_t x, int16_t y, char character, OLED_Font font)
{
    draw_character(x, y, (unsigned char)character, font);
}

void OLED_DrawString(int16_t x, int16_t y, const char *text, OLED_Font font)
{
    uint8_t width = font_width(font);
    int32_t cursor = x;

    if (text == NULL || width == 0U ||
        y >= (int32_t)OLED_HEIGHT || (int32_t)y + (width == 6U ? 8 : 16) <= 0)
    {
        return;
    }
    while (*text != 0 && cursor < (int32_t)OLED_WIDTH)
    {
        draw_character(cursor, y, (unsigned char)*text, font);
        cursor += width;
        ++text;
    }
}

void OLED_PrintLine(uint8_t row, const char *format, ...)
{
    char text[OLED_LINE_CHARS + 1U];
    uint8_t pixels[OLED_WIDTH];
    uint8_t character;
    uint8_t column;
    va_list arguments;

    if (row >= OLED_PAGE_COUNT || format == NULL)
    {
        return;
    }
    /* 兼容截断时返回负数的标准库；始终保留安全、有界的字符串前缀。 */
    memset(text, 0, sizeof(text));
    va_start(arguments, format);
    (void)vsnprintf(text, sizeof(text), format, arguments);
    va_end(arguments);
    text[OLED_LINE_CHARS] = 0;

    /* 先合成整行再比对，避免清除旧内容时产生无意义的脏标记。 */
    memset(pixels, 0, sizeof(pixels));
    for (character = 0; character < OLED_LINE_CHARS && text[character] != 0;
         ++character)
    {
        const uint8_t *glyph = OLED_Font6x8[glyph_index((unsigned char)text[character])];
        memcpy(&pixels[character * 6U], glyph, 6U);
    }
    for (column = 0; column < OLED_WIDTH; ++column)
    {
        write_buffer_byte(row, column, pixels[column]);
    }
}

OLED_Status OLED_DrawBitmap(int16_t x, int16_t y,
                            uint16_t width, uint16_t height,
                            const uint8_t *data, size_t data_length)
{
    size_t required = (size_t)width * (((size_t)height + 7U) / 8U);
    int32_t left = x;
    int32_t top = y;
    int32_t right = (int32_t)x + width;
    int32_t bottom = (int32_t)y + height;
    int32_t column;
    int32_t row;

    if (data == NULL || width == 0U || height == 0U || data_length < required)
    {
        return OLED_INVALID_ARGUMENT;
    }
    if (left < 0) { left = 0; }
    if (top < 0) { top = 0; }
    if (right > (int32_t)OLED_WIDTH) { right = (int32_t)OLED_WIDTH; }
    if (bottom > (int32_t)OLED_HEIGHT) { bottom = (int32_t)OLED_HEIGHT; }
    for (row = top; row < bottom; ++row)
    {
        size_t source_row = (size_t)(row - y);
        for (column = left; column < right; ++column)
        {
            size_t offset = (source_row / 8U) * width + (size_t)(column - x);
            OLED_Color color = ((data[offset] >> (source_row % 8U)) & 1U) != 0U ?
                               OLED_WHITE : OLED_BLACK;
            draw_pixel(column, row, color);
        }
    }
    return OLED_OK;
}

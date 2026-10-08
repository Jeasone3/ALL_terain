#include "oled.h"
#include "oled_font.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

static uint8_t valid(const OLED_Device *d)
{
    return d != NULL && d->started != 0u;
}

OLED_Result OLED_Init(OLED_Device *d, const OLED_Config *c,
                      const OLED_Port *p, uint32_t now)
{
    if (d == NULL || c == NULL || p == NULL || p->write == NULL ||
        c->width == 0u || c->width > OLED_MAX_WIDTH || c->height == 0u ||
        c->height > OLED_MAX_HEIGHT || (c->height % 8u) != 0u ||
        c->address7 > 0x7fu || (uint16_t)c->column_offset + c->width > 256u ||
        c->init_commands == NULL || c->init_length == 0u ||
        c->timeout_ms == 0u || c->retry_ms == 0u ||
        c->retry_ms > 0x7fffffffu || c->power_wait_ms > 0x7fffffffu)
        return OLED_INVALID;
    memset(d, 0, sizeof(*d));
    d->config = *c;
    d->port = *p;
    d->started = 1u;
    d->start_ms = now;
    d->dirty = (uint8_t)((1u << (c->height / 8u)) - 1u);
    return OLED_OK;
}

void OLED_DrawPixel(OLED_Device *d, int16_t x, int16_t y, uint8_t on)
{
    uint16_t index;
    uint8_t old, value, bit;
    if (!valid(d) || x < 0 || y < 0 || x >= d->config.width || y >= d->config.height) return;
    index = (uint16_t)((y / 8) * d->config.width + x);
    old = d->canvas[index];
    bit = (uint8_t)(1u << (y % 8));
    value = on ? (uint8_t)(old | bit) : (uint8_t)(old & (uint8_t)~bit);
    if (old != value) {
        d->canvas[index] = value;
        d->dirty |= (uint8_t)(1u << (y / 8));
    }
}

void OLED_Clear(OLED_Device *d, uint8_t on)
{
    uint16_t i, size;
    uint8_t value = on ? 0xffu : 0u;
    if (!valid(d)) return;
    size = (uint16_t)(d->config.width * (d->config.height / 8u));
    for (i = 0u; i < size; ++i) {
        if (d->canvas[i] != value) {
            d->canvas[i] = value;
            d->dirty |= (uint8_t)(1u << (i / d->config.width));
        }
    }
}

void OLED_DrawLine(OLED_Device *d, int16_t x0, int16_t y0,
                   int16_t x1, int16_t y1, uint8_t on)
{
    int32_t x = x0, y = y0, dx = (int32_t)x1 - x0, dy = (int32_t)y1 - y0;
    int32_t sx = dx < 0 ? -1 : 1, sy = dy < 0 ? -1 : 1, error, twice;
    if (!valid(d)) return;
    if (dx < 0) dx = -dx;
    if (dy < 0) dy = -dy;
    dy = -dy;
    error = dx + dy;
    for (;;) {
        OLED_DrawPixel(d, (int16_t)x, (int16_t)y, on);
        if (x == x1 && y == y1) break;
        twice = error * 2;
        if (twice >= dy) { error += dy; x += sx; }
        if (twice <= dx) { error += dx; y += sy; }
    }
}

void OLED_DrawRect(OLED_Device *d, int16_t x, int16_t y, uint16_t w,
                   uint16_t h, uint8_t filled, uint8_t on)
{
    int32_t px, py, right = (int32_t)x + w - 1, bottom = (int32_t)y + h - 1;
    if (!valid(d) || w == 0u || h == 0u) return;
    /* 只遍历屏幕上的交集，巨大或负起点的矩形也不会拖慢前台。 */
    for (py = y < 0 ? 0 : y; py <= bottom && py < d->config.height; ++py)
        for (px = x < 0 ? 0 : x; px <= right && px < d->config.width; ++px)
            if (filled || px == x || px == right || py == y || py == bottom)
                OLED_DrawPixel(d, (int16_t)px, (int16_t)py, on);
}

OLED_Result OLED_DrawText(OLED_Device *d, int16_t x, int16_t y,
                          const char *text, OLED_Font font)
{
    int32_t px = x, py = y;
    uint8_t width, height, col, row, c, bits;
    if (!valid(d) || text == NULL || (font != OLED_FONT_6X8 && font != OLED_FONT_8X16))
        return OLED_INVALID;
    width = font == OLED_FONT_6X8 ? 6u : 8u;
    height = font == OLED_FONT_6X8 ? 8u : 16u;
    while (*text != '\0' && py < d->config.height) {
        c = (uint8_t)*text++;
        if (c == '\n') { px = x; py += height; continue; }
        if (px >= d->config.width) break; /* 不自动换行，布局由应用明确决定。 */
        if (c < 32u || c > 126u) c = '?';
        c = (uint8_t)(c - 32u);
        for (col = 0u; col < width; ++col)
            for (row = 0u; row < height; ++row) {
                bits = font == OLED_FONT_6X8 ? OLED_Font6x8[c][col] :
                       OLED_Font8x16[c * 16u + (row / 8u) * 8u + col];
                OLED_DrawPixel(d, (int16_t)(px + col), (int16_t)(py + row),
                               (uint8_t)((bits >> (row % 8u)) & 1u));
            }
        px += width;
    }
    return OLED_OK;
}

OLED_Result OLED_Printf(OLED_Device *d, int16_t x, int16_t y,
                        OLED_Font font, const char *format, ...)
{
    char text[96];
    int count;
    va_list args;
    if (!valid(d) || format == NULL) return OLED_INVALID;
    va_start(args, format);
    count = vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    if (count < 0 || count >= (int)sizeof(text)) return OLED_INVALID;
    return OLED_DrawText(d, x, y, text, font);
}

OLED_Result OLED_DrawBitmap(OLED_Device *d, int16_t x, int16_t y,
                            uint16_t w, uint16_t h, const uint8_t *bmp, size_t len)
{
    int32_t px, py, sx, sy;
    size_t needed = (size_t)w * (((size_t)h + 7u) / 8u);
    if (!valid(d) || bmp == NULL || w == 0u || h == 0u || len < needed) return OLED_INVALID;
    for (py = y < 0 ? 0 : y; py < (int32_t)y + h && py < d->config.height; ++py)
        for (px = x < 0 ? 0 : x; px < (int32_t)x + w && px < d->config.width; ++px) {
            sx = px - x; sy = py - y;
            OLED_DrawPixel(d, (int16_t)px, (int16_t)py,
                (uint8_t)((bmp[(size_t)(sy / 8) * w + (size_t)sx] >> (sy % 8)) & 1u));
        }
    return OLED_OK;
}

OLED_Result OLED_Present(OLED_Device *d)
{
    uint8_t page, changed = 0u;
    if (!valid(d)) return OLED_INVALID;
    if (d->pending != 0u) return OLED_BUSY;
    /* Clear 后重画相同内容也无需传输；与上一提交逐页比较实际像素。 */
    for (page = 0u; page < d->config.height / 8u; ++page)
        if ((d->dirty & (1u << page)) != 0u &&
            memcmp(&d->canvas[page * d->config.width], &d->sending[page * d->config.width],
                   d->config.width) != 0) changed |= (uint8_t)(1u << page);
    memcpy(d->sending, d->canvas, (size_t)d->config.width * (d->config.height / 8u));
    d->pending = changed;
    d->dirty = 0u;
    d->status.busy = d->pending != 0u;
    d->page = 0u;
    return OLED_OK;
}

static OLED_Result write_bytes(OLED_Device *d, uint8_t control,
                               const uint8_t *bytes, uint16_t count, uint32_t now)
{
    OLED_Result result = d->port.write(d->port.user, d->config.address7, control,
                                       bytes, count, d->config.timeout_ms);
    if (result != OLED_OK) {
        d->status.last_error = OLED_IO_ERROR;
        ++d->status.error_count;
        d->status.ready = 0u;
        d->last_attempt_ms = now;
        return OLED_IO_ERROR;
    }
    return OLED_OK;
}

OLED_Result OLED_Service(OLED_Device *d, uint32_t now)
{
    uint8_t command[3], column, page;
    if (!valid(d)) return OLED_INVALID;
    if (!d->status.ready) {
        if ((uint32_t)(now - d->start_ms) < d->config.power_wait_ms ||
            (d->status.error_count != 0u &&
             (uint32_t)(now - d->last_attempt_ms) < d->config.retry_ms)) return OLED_BUSY;
        if (write_bytes(d, 0x00u, d->config.init_commands, d->config.init_length, now) != OLED_OK)
            return OLED_IO_ERROR;
        d->status.ready = 1u;
        /* 恢复通信时先完整补发已提交画面；未提交画布仍由应用负责提交。 */
        d->pending = (uint8_t)((1u << (d->config.height / 8u)) - 1u);
        d->page = 0u;
        d->status.busy = 1u;
        return OLED_BUSY;
    }
    if (d->pending == 0u) return OLED_OK;
    for (page = d->page; page < d->config.height / 8u; ++page)
        if ((d->pending & (uint8_t)(1u << page)) != 0u) break;
    column = d->config.column_offset;
    command[0] = (uint8_t)(0xb0u | page);
    command[1] = (uint8_t)(column & 0x0fu);
    command[2] = (uint8_t)(0x10u | (column >> 4));
    if (write_bytes(d, 0x00u, command, 3u, now) != OLED_OK) return OLED_IO_ERROR;
    if (write_bytes(d, 0x40u, &d->sending[page * d->config.width], d->config.width, now) != OLED_OK)
        return OLED_IO_ERROR;
    d->pending &= (uint8_t)~(1u << page);
    d->page = (uint8_t)(page + 1u);
    d->status.busy = d->pending != 0u;
    d->status.last_error = OLED_OK;
    return d->status.busy ? OLED_BUSY : OLED_OK;
}

OLED_Status OLED_GetStatus(const OLED_Device *d)
{
    OLED_Status empty = {0u, OLED_INVALID, 0u, 0u};
    return valid(d) ? d->status : empty;
}

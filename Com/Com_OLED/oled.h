#ifndef OLED_H
#define OLED_H

#include <stdint.h>
#include <stddef.h>

/* 通用层不依赖 HAL。每个对象约占 2KiB，可由应用静态分配。 */
#define OLED_MAX_WIDTH 128u
#define OLED_MAX_HEIGHT 64u
#define OLED_BUFFER_SIZE 1024u

typedef enum { OLED_OK = 0, OLED_BUSY, OLED_INVALID, OLED_IO_ERROR } OLED_Result;
typedef enum { OLED_FONT_6X8 = 0, OLED_FONT_8X16 } OLED_Font;

/* address7 是未移位的七位地址；control=0x00 表示命令，0x40 表示数据。
 * 回调必须同步完成，不能保存 data 指针用于稍后的异步传输。 */
typedef OLED_Result (*OLED_Write)(void *user, uint8_t address7, uint8_t control,
                                const uint8_t *data, uint16_t length, uint32_t timeout_ms);
typedef struct { OLED_Write write; void *user; } OLED_Port;
typedef struct {
    uint8_t width, height, column_offset, address7;
    const uint8_t *init_commands; /* 命令表必须在设备使用期间始终有效。 */
    uint16_t init_length;
    uint32_t timeout_ms, retry_ms, power_wait_ms;
} OLED_Config;
typedef struct {
    uint32_t error_count;
    OLED_Result last_error;
    uint8_t ready, busy;
} OLED_Status;
typedef struct {
    OLED_Config config;
    OLED_Port port;
    uint8_t canvas[OLED_BUFFER_SIZE], sending[OLED_BUFFER_SIZE];
    uint8_t dirty, pending, page, started;
    uint32_t start_ms, last_attempt_ms;
    OLED_Status status;
} OLED_Device;

/* 初始化只配置对象，通信及上电等待由 Service 推进；OK 不代表屏幕已经在线。 */
OLED_Result OLED_Init(OLED_Device *device, const OLED_Config *config,
                      const OLED_Port *port, uint32_t now_ms);
void OLED_Clear(OLED_Device *device, uint8_t on);
void OLED_DrawPixel(OLED_Device *device, int16_t x, int16_t y, uint8_t on);
void OLED_DrawLine(OLED_Device *device, int16_t x0, int16_t y0,
                   int16_t x1, int16_t y1, uint8_t on);
void OLED_DrawRect(OLED_Device *device, int16_t x, int16_t y,
                   uint16_t width, uint16_t height, uint8_t filled, uint8_t on);
OLED_Result OLED_DrawText(OLED_Device *device, int16_t x, int16_t y,
                          const char *text, OLED_Font font);
/* 使用有界临时字符串；浮点格式是否可用取决于目标 C 库。 */
OLED_Result OLED_Printf(OLED_Device *device, int16_t x, int16_t y,
                        OLED_Font font, const char *format, ...);
/* 位图按页排列：先第一组 8 行的各列，再下一组；每字节 bit0 对应上方像素。 */
OLED_Result OLED_DrawBitmap(OLED_Device *device, int16_t x, int16_t y,
                            uint16_t width, uint16_t height,
                            const uint8_t *bitmap, size_t length);
OLED_Result OLED_Present(OLED_Device *device);
/* 一次最多发送一页，成功时仍可能有剩余页面；必须持续在前台调用。 */
OLED_Result OLED_Service(OLED_Device *device, uint32_t now_ms);
OLED_Status OLED_GetStatus(const OLED_Device *device);

#endif

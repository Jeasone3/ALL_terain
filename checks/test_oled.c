#include "oled.h"
#include "oled_font.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

/* 用真实驱动和模拟总线检查传输内容，无 STM32 头文件。 */
typedef struct {
    unsigned commands, pages;
    uint8_t fail, fail_data, page, address, image[1024];
} Bus;
static OLED_Result send(void *user, uint8_t address, uint8_t control,
                         const uint8_t *data, uint16_t len, uint32_t timeout)
{
    Bus *b = user;
    assert(timeout == 20u);
    b->address = address;
    if (b->fail || (b->fail_data && control == 0x40u)) return OLED_IO_ERROR;
    if (control == 0u) {
        ++b->commands;
        if (len == 3u) b->page = (uint8_t)(data[0] & 7u);
    } else {
        assert(control == 0x40u);
        ++b->pages;
        memcpy(&b->image[b->page * 128u], data, len);
    }
    return OLED_OK;
}
static const uint8_t init[] = {0xae, 0xaf};
static OLED_Config config = {128, 64, 0, 0x3c, init, sizeof(init), 20, 500, 100};
static void setup(OLED_Device *d, Bus *b, uint32_t now)
{
    OLED_Port p = {send, b};
    memset(b, 0, sizeof(*b));
    assert(OLED_Init(d, &config, &p, now) == OLED_OK);
}
static void flush(OLED_Device *d, uint32_t now)
{
    unsigned i;
    for (i = 0u; i < 12u; ++i) (void)OLED_Service(d, now);
    assert(!OLED_GetStatus(d).busy);
}
int main(void)
{
    OLED_Device d, other, expected;
    Bus b, other_bus, expected_bus;
    uint8_t bmp[] = {0x01, 0x80};
    unsigned before, i;
    OLED_Port p = {send, &b};
    OLED_Config bad = config;
    bad.height = 63;
    assert(OLED_Init(&d, &bad, &p, 0) == OLED_INVALID);
    assert(OLED_Init(NULL, &config, &p, 0) == OLED_INVALID);
    setup(&d, &b, 0);
    assert(OLED_Service(&d, 99) == OLED_BUSY && b.commands == 0u);
    flush(&d, 100);
    assert(b.pages == 8u && b.address == 0x3c && OLED_GetStatus(&d).ready);
    puts("PASS configuration, power wait and seven-bit address");

    before = b.pages;
    OLED_DrawPixel(&d, 127, 63, 1);
    OLED_DrawPixel(&d, -1, 5, 1);
    OLED_DrawPixel(&d, 128, 64, 1);
    assert(d.canvas[1023] == 0x80);
    assert(OLED_Present(&d) == OLED_OK);
    assert(OLED_Service(&d, 101) == OLED_OK);
    assert(b.pages == before + 1u && b.image[1023] == 0x80);
    puts("PASS clipping and one dirty page");

    OLED_Clear(&d, 0);
    assert(OLED_DrawText(&d, 0, 3, "A", OLED_FONT_6X8) == OLED_OK);
    for (i = 0; i < 6; ++i) {
        assert(d.canvas[i] == (uint8_t)(OLED_Font6x8['A' - 32][i] << 3));
        assert(d.canvas[128 + i] == (uint8_t)(OLED_Font6x8['A' - 32][i] >> 5));
    }
    assert(OLED_DrawText(&d, 0, 0, "X", (OLED_Font)9) == OLED_INVALID);
    setup(&expected, &expected_bus, 0);
    OLED_Clear(&d, 0);
    OLED_DrawText(&d, 0, 0, "\001", OLED_FONT_6X8);
    OLED_DrawText(&expected, 0, 0, "?", OLED_FONT_6X8);
    assert(memcmp(d.canvas, expected.canvas, 1024) == 0);
    OLED_Clear(&d, 0); OLED_Clear(&expected, 0);
    assert(OLED_Printf(&d, 0, 0, OLED_FONT_6X8, "%d", -123) == OLED_OK);
    OLED_DrawText(&expected, 0, 0, "-123", OLED_FONT_6X8);
    assert(memcmp(d.canvas, expected.canvas, 1024) == 0);
    assert(OLED_Printf(&d, 0, 0, OLED_FONT_6X8, "%100d", 1) == OLED_INVALID);
    puts("PASS unaligned fonts, invalid characters and signed numbers");

    OLED_Clear(&d, 0);
    OLED_DrawText(&d, 0, 7, "A", OLED_FONT_8X16);
    for (i = 0; i < 8; ++i) {
        uint8_t top = OLED_Font8x16[('A' - 32) * 16u + i];
        uint8_t bottom = OLED_Font8x16[('A' - 32) * 16u + i + 8];
        assert(d.canvas[i] == (uint8_t)(top << 7));
        assert(d.canvas[128 + i] == (uint8_t)((top >> 1) | (bottom << 7)));
        assert(d.canvas[256 + i] == (uint8_t)(bottom >> 1));
    }
    puts("PASS 8x16 font across three memory pages");

    OLED_Clear(&d, 0);
    assert(OLED_DrawBitmap(&d, 1, 1, 2, 8, bmp, sizeof(bmp)) == OLED_OK);
    assert(d.canvas[1] == 2 && d.canvas[130] == 1);
    assert(OLED_DrawBitmap(&d, 0, 0, 2, 8, bmp, 1) == OLED_INVALID);
    OLED_Clear(&d, 0);
    OLED_DrawRect(&d, -2, -2, 4, 4, 1, 1);
    assert(d.canvas[0] == 3 && d.canvas[1] == 3 && d.canvas[2] == 0);
    OLED_DrawLine(&d, 0, 0, 127, 63, 1);
    assert(d.canvas[1023] == 0x80);
    puts("PASS bitmap format, rectangles and lines");

    OLED_Clear(&d, 0); OLED_DrawPixel(&d, 0, 0, 1);
    assert(OLED_Present(&d) == OLED_OK);
    OLED_DrawPixel(&d, 1, 0, 1);
    assert(OLED_Present(&d) == OLED_BUSY);
    flush(&d, 200);
    assert(b.image[0] == 1 && b.image[1] == 0);
    assert(OLED_Present(&d) == OLED_OK);
    flush(&d, 201);
    assert(b.image[1] == 1);
    before = b.pages;
    OLED_Clear(&d, 0); OLED_DrawPixel(&d, 0, 0, 1); OLED_DrawPixel(&d, 1, 0, 1);
    assert(OLED_Present(&d) == OLED_OK);
    flush(&d, 202);
    assert(b.pages == before);
    puts("PASS immutable submitted image and unchanged redraw suppression");

    setup(&other, &other_bus, 0);
    flush(&other, 100);
    assert(other_bus.image[0] == 0 && b.image[0] == 1);
    OLED_DrawPixel(&d, 2, 0, 1); OLED_Present(&d);
    b.fail_data = 1;
    assert(OLED_Service(&d, 300) == OLED_IO_ERROR);
    assert(!OLED_GetStatus(&d).ready && OLED_GetStatus(&d).error_count == 1);
    before = b.commands;
    assert(OLED_Service(&d, 799) == OLED_BUSY && b.commands == before);
    b.fail_data = 0;
    flush(&d, 800);
    assert(b.image[2] == 1 && OLED_GetStatus(&d).ready && OLED_GetStatus(&d).error_count == 1);
    puts("PASS device isolation, data failure, retry and complete recovery");

    setup(&d, &b, 0xfffffff0u);
    assert(OLED_Service(&d, 0x40u) == OLED_BUSY && b.commands == 0);
    flush(&d, 0x54u);
    b.fail = 1;
    OLED_DrawPixel(&d, 0, 0, 1); OLED_Present(&d);
    assert(OLED_Service(&d, 0xfffffff0u) == OLED_IO_ERROR);
    assert(OLED_Service(&d, 0x100u) == OLED_BUSY);
    b.fail = 0;
    flush(&d, 0x1e4u);
    puts("PASS millisecond counter wraparound");
    {
        OLED_Config smaller = config;
        OLED_Port small_port = {send, &b};
        smaller.width = 64; smaller.height = 32;
        memset(&b, 0, sizeof(b));
        assert(OLED_Init(&d, &smaller, &small_port, 0) == OLED_OK);
        OLED_DrawPixel(&d, 63, 31, 1);
        assert(d.canvas[255] == 0x80);
        OLED_DrawPixel(&d, 64, 32, 1);
        assert(d.canvas[256] == 0);
        OLED_Present(&d); flush(&d, 100);
        assert(b.pages == 4u && b.image[3 * 128 + 63] == 0x80);
    }
    puts("PASS configured 64x32 geometry");
    printf("OLED_Device size: %lu bytes\n", (unsigned long)sizeof(OLED_Device));
    puts("All OLED driver checks passed");
    return 0;
}

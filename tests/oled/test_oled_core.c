#include "Int_OLED.h"
#include "OLED_Port.h"
#include "oledfont.h"

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 黑盒测试：仅通过公开绘制接口和模拟 I2C 屏幕检查结果，不访问驱动内部显存。 */
typedef struct {
    uint8_t control;
    uint16_t length;
    uint8_t data[128];
} Transfer;

static uint8_t screen[8][128];
static uint8_t expected[8][128];
static Transfer transfers[64];
static unsigned transfer_count;
static unsigned checks;
static unsigned failures;
static unsigned fail_at;
static OLED_Status fail_status;
static unsigned current_page;
static unsigned current_column;
static uint32_t delay_ms;
static const char *current_case = "startup";

#define CHECK(condition) check((condition), #condition, __LINE__)

static void check(int condition, const char *expression, unsigned line)
{
    ++checks;
    if (!condition) {
        ++failures;
        fprintf(stderr, "FAIL %s line %u: %s\n", current_case, line, expression);
    }
}

static void reset_log(void)
{
    transfer_count = 0U;
    fail_at = 0U;
    fail_status = OLED_IO_ERROR;
    memset(transfers, 0, sizeof(transfers));
}

OLED_Status OLED_Port_Write(uint8_t control, const uint8_t *data, uint16_t length)
{
    unsigned i;
    if (data == NULL || length == 0U || length > 128U || transfer_count >= 64U) {
        fprintf(stderr, "Invalid mock transfer\n");
        exit(2);
    }
    transfers[transfer_count].control = control;
    transfers[transfer_count].length = length;
    memcpy(transfers[transfer_count].data, data, length);
    ++transfer_count;
    if (fail_at == transfer_count) {
        return fail_status;
    }
    /* 只解析页地址批次，初始化命令和功能命令无需模拟控制器的模拟电路。 */
    if (control == 0x00U && length == 3U &&
        data[0] >= 0xB0U && data[0] <= 0xB7U) {
        current_page = data[0] & 7U;
        current_column = (data[1] & 15U) | ((data[2] & 15U) << 4U);
    } else if (control == 0x40U) {
        if (current_column + length > OLED_WIDTH) {
            fprintf(stderr, "Mock transfer crosses page boundary\n");
            exit(2);
        }
        for (i = 0U; i < length; ++i) {
            screen[current_page][current_column + i] = data[i];
        }
        current_column += length;
    }
    return OLED_OK;
}

void OLED_Port_DelayMs(uint32_t milliseconds)
{
    delay_ms += milliseconds;
}

static void ref_pixel(int x, int y, unsigned color)
{
    uint8_t mask;
    if (x < 0 || x >= 128 || y < 0 || y >= 64) {
        return;
    }
    mask = (uint8_t)(1U << ((unsigned)y & 7U));
    if (color != 0U) {
        expected[(unsigned)y / 8U][(unsigned)x] |= mask;
    } else {
        expected[(unsigned)y / 8U][(unsigned)x] &= (uint8_t)~mask;
    }
}

static void verify_screen(void)
{
    int matches = memcmp(screen, expected, sizeof(screen)) == 0;
    unsigned page;
    unsigned column;
    unsigned differences = 0U;
    CHECK(matches);
    if (!matches) {
        for (page = 0U; page < 8U; ++page) {
            for (column = 0U; column < 128U; ++column) {
                if (screen[page][column] != expected[page][column]) {
                    if (differences < 8U) {
                        fprintf(stderr, "  page %u, column %u: got 0x%02X, expected 0x%02X\n",
                            page, column, (unsigned)screen[page][column],
                            (unsigned)expected[page][column]);
                    }
                    ++differences;
                }
            }
        }
        fprintf(stderr, "  %u differing columns\n", differences);
    }
}

static void clear_screen(void)
{
    OLED_Clear();
    CHECK(OLED_Update() == OLED_OK);
    memset(expected, 0, sizeof(expected));
    verify_screen();
    reset_log();
}

static void ref_character(int x, int y, unsigned character, OLED_Font font)
{
    unsigned dx;
    unsigned dy;
    unsigned width = font == OLED_FONT_6X8 ? 6U : 8U;
    unsigned height = font == OLED_FONT_6X8 ? 8U : 16U;
    unsigned index = character >= 32U && character <= 126U ? character - 32U : '?' - 32U;
    for (dy = 0U; dy < height; ++dy) {
        for (dx = 0U; dx < width; ++dx) {
            uint8_t column = font == OLED_FONT_6X8 ? OLED_Font6x8[index][dx] :
                OLED_Font8x16[index][(dy / 8U) * 8U + dx];
            ref_pixel(x + (int)dx, y + (int)dy, (column >> (dy & 7U)) & 1U);
        }
    }
}

static void ref_text(int x, int y, const char *text, OLED_Font font)
{
    unsigned width = font == OLED_FONT_6X8 ? 6U : 8U;
    while (*text != '\0') {
        ref_character(x, y, (unsigned char)*text, font);
        x += (int)width;
        ++text;
    }
}

static void test_initialization(void)
{
    /* 面板参数保持原配置，寻址参数必须选择与批量刷新一致的页模式。 */
    static const uint8_t expected_init[28] = {
        0xAE, 0x20, 0x02, 0xB0, 0xC8, 0x00, 0x10, 0x40,
        0x81, 0xFF, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3,
        0x00, 0xD5, 0xF0, 0xD9, 0x22, 0xDA, 0x12, 0xDB,
        0x20, 0x8D, 0x14, 0xAF
    };
    unsigned i;
    unsigned data_count = 0U;
    current_case = "initialization";
    CHECK(OLED_Update() == OLED_NOT_INITIALIZED);
    CHECK(OLED_SetDisplay(1U) == OLED_NOT_INITIALIZED);
    CHECK(OLED_SetContrast(128U) == OLED_NOT_INITIALIZED);
    CHECK(transfer_count == 0U);

    fail_at = 1U;
    fail_status = OLED_TIMEOUT;
    CHECK(OLED_Init() == OLED_TIMEOUT);
    CHECK(transfer_count == 1U);
    CHECK(OLED_Update() == OLED_NOT_INITIALIZED);

    reset_log();
    memset(screen, 0xFF, sizeof(screen));
    CHECK(OLED_Init() == OLED_OK);
    CHECK(delay_ms >= 200U);
    CHECK(transfers[0].control == 0U);
    CHECK(transfers[0].length >= sizeof(expected_init));
    CHECK(memcmp(transfers[0].data, expected_init, sizeof(expected_init)) == 0);
    for (i = 0U; i < transfer_count; ++i) {
        if (transfers[i].control == 0x40U) {
            ++data_count;
            CHECK(transfers[i].length == 128U);
            CHECK(i > 0U);
            CHECK(transfers[i - 1U].length == 3U);
            CHECK(transfers[i - 1U].data[0] == (uint8_t)(0xB0U + data_count - 1U));
            CHECK(transfers[i - 1U].data[1] == 0U);
            CHECK(transfers[i - 1U].data[2] == 0x10U);
        }
    }
    CHECK(data_count == 8U);
    verify_screen();
    reset_log();
    CHECK(OLED_Update() == OLED_OK);
    CHECK(transfer_count == 0U);
}

static void test_pixels_and_dirty_ranges(void)
{
    current_case = "pixels and dirty ranges";
    clear_screen();
    OLED_DrawPixel(0, 0, OLED_WHITE);
    OLED_DrawPixel(127, 63, OLED_WHITE);
    OLED_DrawPixel(2, 7, OLED_WHITE);
    OLED_DrawPixel(100, 7, OLED_WHITE);
    OLED_DrawPixel(-1, 0, OLED_WHITE);
    OLED_DrawPixel(128, 0, OLED_WHITE);
    OLED_DrawPixel(0, -1, OLED_WHITE);
    OLED_DrawPixel(0, 64, OLED_WHITE);
    OLED_DrawPixel(INT16_MIN, INT16_MAX, OLED_WHITE);
    CHECK(transfer_count == 0U);
    ref_pixel(0, 0, 1U);
    ref_pixel(127, 63, 1U);
    ref_pixel(2, 7, 1U);
    ref_pixel(100, 7, 1U);
    CHECK(OLED_Update() == OLED_OK);
    CHECK(transfer_count == 4U);
    CHECK(transfers[0].data[1] == 0U);
    CHECK(transfers[1].length == 101U);
    CHECK(transfers[2].data[0] == 0xB7U);
    CHECK(transfers[2].data[1] == 0x0FU);
    CHECK(transfers[2].data[2] == 0x17U);
    CHECK(transfers[3].length == 1U);
    verify_screen();
    reset_log();
    OLED_DrawPixel(2, 7, OLED_WHITE);
    CHECK(OLED_Update() == OLED_OK);
    CHECK(transfer_count == 0U);
    OLED_DrawPixel(2, 7, OLED_BLACK);
    ref_pixel(2, 7, 0U);
    CHECK(OLED_Update() == OLED_OK);
    CHECK(transfer_count == 2U);
    CHECK(transfers[0].data[1] == 2U);
    CHECK(transfers[0].data[2] == 0x10U);
    CHECK(transfers[1].length == 1U);
    verify_screen();
}

static void test_update_failures(void)
{
    static const OLED_Status errors[] = {OLED_IO_ERROR, OLED_TIMEOUT, OLED_BUSY};
    unsigned error;
    unsigned failing_call;
    current_case = "update failures";
    for (error = 0U; error < sizeof(errors) / sizeof(errors[0]); ++error) {
        for (failing_call = 3U; failing_call <= 4U; ++failing_call) {
            clear_screen();
            OLED_DrawPixel(2, 0, OLED_WHITE);
            OLED_DrawPixel(10, 8, OLED_WHITE);
            OLED_DrawPixel(20, 16, OLED_WHITE);
            ref_pixel(2, 0, 1U);
            ref_pixel(10, 8, 1U);
            ref_pixel(20, 16, 1U);
            fail_at = failing_call;
            fail_status = errors[error];
            CHECK(OLED_Update() == errors[error]);
            CHECK(transfer_count == failing_call);
            CHECK(screen[0][2] == 1U);
            CHECK(screen[1][10] == 0U);
            CHECK(screen[2][20] == 0U);
            reset_log();
            CHECK(OLED_Update() == OLED_OK);
            CHECK(transfer_count == 4U);
            CHECK(transfers[0].data[0] == 0xB1U);
            CHECK(transfers[2].data[0] == 0xB2U);
            verify_screen();
            reset_log();
            CHECK(OLED_Update() == OLED_OK);
            CHECK(transfer_count == 0U);
        }
    }
    /* 初始化清屏过程中失败后，不能把部分清屏当成完整初始化。 */
    reset_log();
    fail_at = 5U;
    fail_status = OLED_IO_ERROR;
    CHECK(OLED_Init() == OLED_IO_ERROR);
    CHECK(transfer_count == 5U);
    CHECK(OLED_Update() == OLED_NOT_INITIALIZED);
    reset_log();
    CHECK(OLED_Init() == OLED_OK);
    memset(expected, 0, sizeof(expected));
    verify_screen();
}

static void test_control_commands(void)
{
    current_case = "control commands";
    reset_log();
    CHECK(OLED_SetDisplay(2U) == OLED_INVALID_ARGUMENT);
    CHECK(transfer_count == 0U);
    CHECK(OLED_SetDisplay(0U) == OLED_OK);
    CHECK(transfer_count == 1U);
    CHECK(transfers[0].control == 0x00U);
    CHECK(transfers[0].data[transfers[0].length - 1U] == 0xAEU);
    CHECK(OLED_SetDisplay(1U) == OLED_OK);
    CHECK(transfer_count == 2U);
    CHECK(transfers[1].data[transfers[1].length - 1U] == 0xAFU);
    CHECK(OLED_SetContrast(0x35U) == OLED_OK);
    CHECK(transfer_count == 3U);
    CHECK(transfers[2].length == 2U);
    CHECK(transfers[2].data[0] == 0x81U);
    CHECK(transfers[2].data[1] == 0x35U);
    reset_log();
    fail_at = 1U;
    fail_status = OLED_BUSY;
    CHECK(OLED_SetContrast(0U) == OLED_BUSY);
    CHECK(transfer_count == 1U);
    reset_log();
    fail_at = 1U;
    fail_status = OLED_TIMEOUT;
    CHECK(OLED_SetDisplay(0U) == OLED_TIMEOUT);
    CHECK(transfer_count == 1U);
    reset_log();
}

static void test_fonts_and_text(void)
{
    static const uint8_t reference_a6[6] = {0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C};
    static char distant_text[6000];
    unsigned character;
    unsigned font;
    unsigned column;
    current_case = "fonts and text";
    CHECK(memcmp(OLED_Font6x8['A' - 32], reference_a6, 6U) == 0);
    CHECK(OLED_Font8x16['|' - 32][4] == 0xFFU);
    CHECK(OLED_Font8x16['|' - 32][12] == 0xFFU);
    for (font = 0U; font < 2U; ++font) {
        for (character = 32U; character <= 126U; ++character) {
            unsigned nonempty = 0U;
            unsigned length = font == 0U ? 6U : 16U;
            const uint8_t *glyph = font == 0U ? OLED_Font6x8[character - 32U] :
                OLED_Font8x16[character - 32U];
            for (column = 0U; column < length; ++column) {
                nonempty |= glyph[column];
            }
            CHECK(character == 32U ? nonempty == 0U : nonempty != 0U);
            clear_screen();
            OLED_DrawChar(3, 5, (char)character, (OLED_Font)font);
            ref_character(3, 5, character, (OLED_Font)font);
            CHECK(OLED_Update() == OLED_OK);
            verify_screen();
        }
    }
    for (character = 0U; character <= 255U; ++character) {
        if (character >= 32U && character <= 126U) {
            continue;
        }
        clear_screen();
        OLED_DrawChar(0, 0, (char)character, OLED_FONT_6X8);
        ref_character(0, 0, '?', OLED_FONT_6X8);
        CHECK(OLED_Update() == OLED_OK);
        verify_screen();
    }

    clear_screen();
    OLED_FillRect(0, 0, 128U, 64U, OLED_WHITE);
    CHECK(OLED_Update() == OLED_OK);
    memset(expected, 0xFF, sizeof(expected));
    OLED_DrawString(-2, -3, "AB ~", OLED_FONT_6X8);
    ref_text(-2, -3, "AB ~", OLED_FONT_6X8);
    OLED_DrawString(121, 53, "AZ", OLED_FONT_8X16);
    ref_text(121, 53, "AZ", OLED_FONT_8X16);
    CHECK(OLED_Update() == OLED_OK);
    verify_screen();

    clear_screen();
    OLED_DrawString(120, 0, "AAAAAAAAAAAA", OLED_FONT_6X8);
    ref_text(120, 0, "AAAAAAAAAAAA", OLED_FONT_6X8);
    CHECK(OLED_Update() == OLED_OK);
    verify_screen();
    reset_log();
    OLED_DrawString(0, 0, NULL, OLED_FONT_6X8);
    OLED_DrawString(0, 0, "A", (OLED_Font)99);
    OLED_DrawChar(0, 0, 'A', (OLED_Font)99);
    CHECK(OLED_Update() == OLED_OK);
    CHECK(transfer_count == 0U);

    clear_screen();
    memset(distant_text, 'A', sizeof(distant_text));
    distant_text[sizeof(distant_text) - 1U] = '\0';
    OLED_DrawString(INT16_MAX, 0, distant_text, OLED_FONT_6X8);
    CHECK(OLED_Update() == OLED_OK);
    CHECK(transfer_count == 0U);
    OLED_DrawString(INT16_MIN, 0, distant_text, OLED_FONT_6X8);
    ref_text(INT16_MIN, 0, distant_text, OLED_FONT_6X8);
    CHECK(OLED_Update() == OLED_OK);
    verify_screen();
}

static void test_formatted_lines(void)
{
    char long_text[300];
    unsigned row;
    current_case = "formatted lines";
    memset(long_text, 'M', sizeof(long_text));
    long_text[sizeof(long_text) - 1U] = '\0';
    clear_screen();
    for (row = 0U; row < OLED_PAGE_COUNT; ++row) {
        OLED_PrintLine((uint8_t)row, "Target:%.1f/%d/%u", -12.34, -9, 7U);
        ref_text(0, (int)row * 8, "Target:-12.3/-9/7", OLED_FONT_6X8);
    }
    CHECK(transfer_count == 0U);
    CHECK(OLED_Update() == OLED_OK);
    verify_screen();
    reset_log();
    for (row = 0U; row < OLED_PAGE_COUNT; ++row) {
        OLED_PrintLine((uint8_t)row, "Target:%.1f/%d/%u", -12.34, -9, 7U);
    }
    CHECK(OLED_Update() == OLED_OK);
    CHECK(transfer_count == 0U);

    OLED_PrintLine(0U, "%s", long_text);
    memset(expected[0], 0, 128U);
    ref_text(0, 0, "MMMMMMMMMMMMMMMMMMMMM", OLED_FONT_6X8);
    CHECK(OLED_Update() == OLED_OK);
    verify_screen();
    CHECK(screen[0][126] == 0U && screen[0][127] == 0U);
    OLED_PrintLine(0U, "%s", "A");
    memset(expected[0], 0, 128U);
    ref_text(0, 0, "A", OLED_FONT_6X8);
    CHECK(OLED_Update() == OLED_OK);
    verify_screen();
    OLED_PrintLine(0U, "");
    memset(expected[0], 0, 128U);
    CHECK(OLED_Update() == OLED_OK);
    verify_screen();
    reset_log();
    OLED_PrintLine(8U, "A");
    OLED_PrintLine(255U, "A");
    OLED_PrintLine(0U, NULL);
    CHECK(OLED_Update() == OLED_OK);
    CHECK(transfer_count == 0U);
}

static void test_rectangles_and_lines(void)
{
    int x;
    int y;
    current_case = "rectangles and lines";
    clear_screen();
    OLED_DrawLine(-5, 12, 140, 12, OLED_WHITE);
    OLED_DrawLine(15, -10, 15, 100, OLED_WHITE);
    OLED_DrawLine(-7, -7, 70, 70, OLED_WHITE);
    OLED_DrawLine(100, 20, 60, 60, OLED_WHITE);
    OLED_DrawLine(45, 45, 45, 45, OLED_WHITE);
    for (x = 0; x < 128; ++x) {
        ref_pixel(x, 12, 1U);
    }
    for (y = 0; y < 64; ++y) {
        ref_pixel(15, y, 1U);
        ref_pixel(y, y, 1U);
    }
    for (x = 60; x <= 100; ++x) {
        ref_pixel(x, 120 - x, 1U);
    }
    ref_pixel(45, 45, 1U);
    CHECK(OLED_Update() == OLED_OK);
    verify_screen();

    clear_screen();
    OLED_DrawRect(-2, 3, 8U, 5U, OLED_WHITE);
    OLED_DrawRect(127, 63, 1U, 1U, OLED_WHITE);
    OLED_FillRect(125, 61, 65535U, 65535U, OLED_WHITE);
    for (y = 3; y < 8; ++y) {
        for (x = 0; x < 6; ++x) {
            if (x == 5 || y == 3 || y == 7) {
                ref_pixel(x, y, 1U);
            }
        }
    }
    for (y = 61; y < 64; ++y) {
        for (x = 125; x < 128; ++x) {
            ref_pixel(x, y, 1U);
        }
    }
    CHECK(OLED_Update() == OLED_OK);
    verify_screen();
    OLED_ClearRect(126, 62, 65535U, 65535U);
    for (y = 62; y < 64; ++y) {
        for (x = 126; x < 128; ++x) {
            ref_pixel(x, y, 0U);
        }
    }
    CHECK(OLED_Update() == OLED_OK);
    verify_screen();
    reset_log();
    OLED_DrawRect(0, 0, 0U, 10U, OLED_WHITE);
    OLED_DrawRect(0, 0, 10U, 0U, OLED_WHITE);
    OLED_FillRect(0, 0, 0U, 10U, OLED_WHITE);
    OLED_ClearRect(0, 0, 10U, 0U);
    OLED_DrawRect(INT16_MIN, INT16_MIN, 1U, 1U, OLED_WHITE);
    CHECK(OLED_Update() == OLED_OK);
    CHECK(transfer_count == 0U);

    clear_screen();
    OLED_FillRect(INT16_MIN, INT16_MIN, 65535U, 65535U, OLED_WHITE);
    memset(expected, 0xFF, sizeof(expected));
    CHECK(OLED_Update() == OLED_OK);
    verify_screen();
    OLED_ClearRect(-1, -1, 65535U, 65535U);
    memset(expected, 0, sizeof(expected));
    CHECK(OLED_Update() == OLED_OK);
    verify_screen();
    reset_log();
    OLED_DrawRect(-1, -1, 65535U, 65535U, OLED_WHITE);
    CHECK(OLED_Update() == OLED_OK);
    CHECK(transfer_count == 0U);
    OLED_DrawLine(INT16_MIN, INT16_MIN, INT16_MAX, INT16_MAX, OLED_WHITE);
    for (y = 0; y < 64; ++y) {
        ref_pixel(y, y, 1U);
    }
    CHECK(OLED_Update() == OLED_OK);
    verify_screen();
}

static unsigned pixel_at(int x, int y)
{
    return (screen[(unsigned)y / 8U][(unsigned)x] >> ((unsigned)y & 7U)) & 1U;
}

static void test_circles(void)
{
    int dx;
    int dy;
    current_case = "circles";
    clear_screen();
    OLED_DrawCircle(0, 0, 0U, OLED_WHITE);
    OLED_FillCircle(127, 63, 0U, OLED_WHITE);
    ref_pixel(0, 0, 1U);
    ref_pixel(127, 63, 1U);
    CHECK(OLED_Update() == OLED_OK);
    verify_screen();
    clear_screen();
    OLED_DrawCircle(20, 20, 3U, OLED_WHITE);
    CHECK(OLED_Update() == OLED_OK);
    CHECK(pixel_at(20, 17) == 1U);
    CHECK(pixel_at(20, 23) == 1U);
    CHECK(pixel_at(17, 20) == 1U);
    CHECK(pixel_at(23, 20) == 1U);
    CHECK(pixel_at(20, 20) == 0U);
    for (dy = -3; dy <= 3; ++dy) {
        for (dx = -3; dx <= 3; ++dx) {
            CHECK(pixel_at(20 + dx, 20 + dy) == pixel_at(20 - dx, 20 + dy));
            CHECK(pixel_at(20 + dx, 20 + dy) == pixel_at(20 + dy, 20 + dx));
        }
    }
    clear_screen();
    OLED_FillCircle(0, 0, 3U, OLED_WHITE);
    CHECK(OLED_Update() == OLED_OK);
    CHECK(pixel_at(0, 0) == 1U);
    CHECK(pixel_at(3, 0) == 1U);
    CHECK(pixel_at(0, 3) == 1U);
    CHECK(pixel_at(3, 3) == 0U);
    CHECK(pixel_at(4, 0) == 0U);
    clear_screen();
    OLED_DrawCircle(INT16_MIN, INT16_MIN, 65535U, OLED_WHITE);
    OLED_DrawCircle(0, 0, 65535U, OLED_WHITE);
    CHECK(OLED_Update() == OLED_OK);
    verify_screen();
    OLED_FillCircle(0, 0, 65535U, OLED_WHITE);
    memset(expected, 0xFF, sizeof(expected));
    CHECK(OLED_Update() == OLED_OK);
    verify_screen();
    OLED_FillCircle(0, 0, 65535U, OLED_BLACK);
    memset(expected, 0, sizeof(expected));
    CHECK(OLED_Update() == OLED_OK);
    verify_screen();
}

static void ref_bitmap(int x, int y, unsigned width, unsigned height, const uint8_t *data)
{
    unsigned dx;
    unsigned dy;
    for (dy = 0U; dy < height; ++dy) {
        for (dx = 0U; dx < width; ++dx) {
            ref_pixel(x + (int)dx, y + (int)dy,
                (data[(dy / 8U) * width + dx] >> (dy & 7U)) & 1U);
        }
    }
}

static void test_bitmaps(void)
{
    static const uint8_t bitmap[10] = {0x81, 0x42, 0x24, 0x18, 0xFF, 0xFD, 0xFE, 0xFF, 0xFC, 0xFD};
    static uint8_t wide_bitmap[65535];
    unsigned column;
    current_case = "bitmaps";
    clear_screen();
    OLED_FillRect(0, 0, 128U, 64U, OLED_WHITE);
    CHECK(OLED_Update() == OLED_OK);
    memset(expected, 0xFF, sizeof(expected));
    CHECK(OLED_DrawBitmap(3, 5, 5U, 10U, bitmap, sizeof(bitmap)) == OLED_OK);
    ref_bitmap(3, 5, 5U, 10U, bitmap);
    CHECK(OLED_Update() == OLED_OK);
    verify_screen();
    reset_log();
    CHECK(OLED_DrawBitmap(3, 5, 5U, 10U, bitmap, sizeof(bitmap)) == OLED_OK);
    CHECK(OLED_Update() == OLED_OK);
    CHECK(transfer_count == 0U);
    CHECK(OLED_DrawBitmap(-2, -3, 5U, 10U, bitmap, sizeof(bitmap)) == OLED_OK);
    ref_bitmap(-2, -3, 5U, 10U, bitmap);
    CHECK(OLED_DrawBitmap(126, 60, 5U, 10U, bitmap, sizeof(bitmap)) == OLED_OK);
    ref_bitmap(126, 60, 5U, 10U, bitmap);
    CHECK(OLED_Update() == OLED_OK);
    verify_screen();
    reset_log();
    CHECK(OLED_DrawBitmap(0, 0, 5U, 10U, bitmap, sizeof(bitmap) - 1U) == OLED_INVALID_ARGUMENT);
    CHECK(OLED_DrawBitmap(0, 0, 5U, 10U, NULL, sizeof(bitmap)) == OLED_INVALID_ARGUMENT);
    CHECK(OLED_DrawBitmap(0, 0, 0U, 10U, bitmap, sizeof(bitmap)) == OLED_INVALID_ARGUMENT);
    CHECK(OLED_DrawBitmap(0, 0, 5U, 0U, bitmap, sizeof(bitmap)) == OLED_INVALID_ARGUMENT);
    CHECK(OLED_DrawBitmap(0, 0, 65535U, 65535U, bitmap, sizeof(bitmap)) == OLED_INVALID_ARGUMENT);
    CHECK(OLED_Update() == OLED_OK);
    CHECK(transfer_count == 0U);
    verify_screen();
    clear_screen();
    CHECK(OLED_DrawBitmap(0, 0, 128U, 64U, OLED_BitmapDemo, 1024U) == OLED_OK);
    memcpy(expected, OLED_BitmapDemo, sizeof(expected));
    CHECK(OLED_Update() == OLED_OK);
    CHECK(transfer_count == 16U);
    verify_screen();

    clear_screen();
    for (column = 0U; column < sizeof(wide_bitmap); ++column) {
        wide_bitmap[column] = (uint8_t)column;
    }
    CHECK(OLED_DrawBitmap(INT16_MIN, 0, 65535U, 1U,
                          wide_bitmap, sizeof(wide_bitmap)) == OLED_OK);
    ref_bitmap(INT16_MIN, 0, 65535U, 1U, wide_bitmap);
    CHECK(OLED_Update() == OLED_OK);
    verify_screen();
}

int main(void)
{
    test_initialization();
    test_pixels_and_dirty_ranges();
    test_update_failures();
    test_control_commands();
    test_fonts_and_text();
    test_formatted_lines();
    test_rectangles_and_lines();
    test_circles();
    test_bitmaps();
    printf("OLED core: %u checks, %u failures\n", checks, failures);
    return failures == 0U ? EXIT_SUCCESS : EXIT_FAILURE;
}

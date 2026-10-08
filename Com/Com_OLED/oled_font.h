#ifndef OLED_FONT_H
#define OLED_FONT_H
#include <stdint.h>
/* ASCII 0x20..0x7E；定义仅在 oled_font.c 出现一次。 */
extern const uint8_t OLED_Font6x8[95][6];
extern const uint8_t OLED_Font8x16[95 * 16];
#endif

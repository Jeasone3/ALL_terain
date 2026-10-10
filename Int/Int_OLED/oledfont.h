#ifndef OLED_FONT_H
#define OLED_FONT_H

#include <stdint.h>

/* 完整可打印 ASCII 字库，字符索引为字符编码减 0x20。 */
extern const uint8_t OLED_Font6x8[95][6];
extern const uint8_t OLED_Font8x16[95][16];
extern const uint8_t OLED_BitmapDemo[1024];

#endif /* OLED_FONT_H */

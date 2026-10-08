#ifndef __INT_OLED_H__
#define __INT_OLED_H__

#include "oled.h"

/* 当前板子的 HAL 适配层；移植通用层时不需要复制此文件。
 * 由主循环调用一次初始化，后续向返回对象绘图并调用 OLED_Service。 */
OLED_Result IntOLED_Init(uint32_t now_ms);
OLED_Device *IntOLED_GetDevice(void);




#endif /* __INT_OLED_H__ */

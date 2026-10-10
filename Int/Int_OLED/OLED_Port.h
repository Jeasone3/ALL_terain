#ifndef OLED_PORT_H
#define OLED_PORT_H

#include "Int_OLED.h"

/* 七位地址；STM32 HAL 需要的左移由平台层完成。 */
#ifndef OLED_I2C_ADDRESS
#define OLED_I2C_ADDRESS 0x3CU
#endif

/* 单次批量写入的超时上限，单位为毫秒。 */
#ifndef OLED_I2C_TIMEOUT_MS
#define OLED_I2C_TIMEOUT_MS 100U
#endif

/* control 只能为 0x00（命令）或 0x40（显存数据）。 */
OLED_Status OLED_Port_Write(uint8_t control, const uint8_t *data, uint16_t length);
void OLED_Port_DelayMs(uint32_t milliseconds);

#endif /* OLED_PORT_H */

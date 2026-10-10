#ifndef OLED_TEST_STM32F1XX_HAL_H
#define OLED_TEST_STM32F1XX_HAL_H

#include <stdint.h>

typedef enum {
    HAL_OK = 0,
    HAL_ERROR = 1,
    HAL_BUSY = 2,
    HAL_TIMEOUT = 3
} HAL_StatusTypeDef;

typedef struct {
    unsigned marker;
} I2C_HandleTypeDef;

#define I2C_MEMADD_SIZE_8BIT 1U

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *handle, uint16_t address,
                                  uint16_t memory_address, uint16_t memory_size,
                                  uint8_t *data, uint16_t length, uint32_t timeout);
void HAL_Delay(uint32_t milliseconds);

#endif

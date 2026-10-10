#include "OLED_Port.h"
#include "i2c.h"

#include <stdio.h>
#include <stdlib.h>

/* 编译真实平台适配文件，只有 HAL 本身由测试替换。 */
I2C_HandleTypeDef hi2c1;
static unsigned checks;
static unsigned failures;
static unsigned calls;
static HAL_StatusTypeDef hal_result;
static I2C_HandleTypeDef *seen_handle;
static uint16_t seen_address;
static uint16_t seen_control;
static uint16_t seen_memory_size;
static uint8_t *seen_data;
static uint16_t seen_length;
static uint32_t seen_timeout;
static uint32_t seen_delay;

#define CHECK(condition) check((condition), #condition, __LINE__)

static void check(int condition, const char *expression, unsigned line)
{
    ++checks;
    if (!condition) {
        ++failures;
        fprintf(stderr, "FAIL line %u: %s\n", line, expression);
    }
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *handle, uint16_t address,
                                  uint16_t memory_address, uint16_t memory_size,
                                  uint8_t *data, uint16_t length, uint32_t timeout)
{
    ++calls;
    seen_handle = handle;
    seen_address = address;
    seen_control = memory_address;
    seen_memory_size = memory_size;
    seen_data = data;
    seen_length = length;
    seen_timeout = timeout;
    return hal_result;
}

void HAL_Delay(uint32_t milliseconds)
{
    seen_delay = milliseconds;
}

int main(void)
{
    const uint8_t data[128] = {0xA5U};
    if (OLED_I2C_ADDRESS > 0x7FU) {
        CHECK(OLED_Port_Write(0x00U, data, 128U) == OLED_INVALID_ARGUMENT);
        CHECK(calls == 0U);
    } else {
        CHECK(OLED_Port_Write(0x00U, data, 128U) == OLED_OK);
        CHECK(calls == 1U);
        CHECK(seen_handle == &hi2c1);
        CHECK(seen_address == (OLED_I2C_ADDRESS << 1U));
        CHECK(seen_control == 0x00U);
        CHECK(seen_memory_size == I2C_MEMADD_SIZE_8BIT);
        CHECK(seen_data == data);
        CHECK(seen_length == 128U);
        CHECK(seen_timeout == OLED_I2C_TIMEOUT_MS);
        CHECK(OLED_Port_Write(0x40U, data, 3U) == OLED_OK);
        CHECK(calls == 2U);
        CHECK(seen_control == 0x40U);
        CHECK(seen_length == 3U);
        hal_result = HAL_ERROR;
        CHECK(OLED_Port_Write(0x00U, data, 1U) == OLED_IO_ERROR);
        hal_result = HAL_TIMEOUT;
        CHECK(OLED_Port_Write(0x00U, data, 1U) == OLED_TIMEOUT);
        hal_result = HAL_BUSY;
        CHECK(OLED_Port_Write(0x00U, data, 1U) == OLED_BUSY);
        hal_result = (HAL_StatusTypeDef)99;
        CHECK(OLED_Port_Write(0x00U, data, 1U) == OLED_IO_ERROR);
        calls = 0U;
        CHECK(OLED_Port_Write(0x80U, data, 1U) == OLED_INVALID_ARGUMENT);
        CHECK(OLED_Port_Write(0x00U, NULL, 1U) == OLED_INVALID_ARGUMENT);
        CHECK(OLED_Port_Write(0x00U, data, 0U) == OLED_INVALID_ARGUMENT);
        CHECK(calls == 0U);
    }
    OLED_Port_DelayMs(123U);
    CHECK(seen_delay == 123U);
    printf("OLED port (address 0x%02X): %u checks, %u failures\n",
           (unsigned)OLED_I2C_ADDRESS, checks, failures);
    return failures == 0U ? EXIT_SUCCESS : EXIT_FAILURE;
}

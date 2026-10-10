#include "OLED_Port.h"

#include "i2c.h"
#include "stm32f1xx_hal.h"

OLED_Status OLED_Port_Write(uint8_t control, const uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef result;

    if ((control != 0x00U && control != 0x40U) || data == 0 || length == 0U ||
        OLED_I2C_ADDRESS > 0x7FU)
    {
        return OLED_INVALID_ARGUMENT;
    }

    /* HAL 的旧接口未声明 const；同步传输只读取调用方数据。 */
    result = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(OLED_I2C_ADDRESS << 1U),
                               control, I2C_MEMADD_SIZE_8BIT,
                               (uint8_t *)data, length, OLED_I2C_TIMEOUT_MS);

    switch (result)
    {
    case HAL_OK:
        return OLED_OK;
    case HAL_TIMEOUT:
        return OLED_TIMEOUT;
    case HAL_BUSY:
        return OLED_BUSY;
    case HAL_ERROR:
    default:
        return OLED_IO_ERROR;
    }
}

void OLED_Port_DelayMs(uint32_t milliseconds)
{
    HAL_Delay(milliseconds);
}

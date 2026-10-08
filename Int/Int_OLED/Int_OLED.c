#include "Int_OLED.h"
#include "i2c.h"

/* 当前屏幕芯片尚未确认，沿用原电气参数和扫描方向。
 * 按页传输必须选择页寻址：原 0x20,0x10 的低两位为 00，会选水平模式。
 * 修正为 SSD1306 页模式 0x20,0x02，匹配 B0..B7 和高低列命令。
 * SH1106 需要独立初始化表和列偏移，不能只靠改 I2C 地址实现兼容。 */
static const uint8_t s_panel_init[] = {
    0xae, 0x20, 0x02, 0xb0, 0xc8, 0x00, 0x10, 0x40,
    0x81, 0xff, 0xa1, 0xa6, 0xa8, 0x3f, 0xa4, 0xd3, 0x00,
    0xd5, 0xf0, 0xd9, 0x22, 0xda, 0x12, 0xdb, 0x20,
    0x8d, 0x14, 0xaf
};
static OLED_Device s_display;

static OLED_Result hal_write(void *user, uint8_t address7, uint8_t control,
                             const uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
    /* HAL 地址参数左移一次。Mem_Write 的寄存器字节在这里就是 OLED 控制字节。
     * HAL 接口缺少 const，但同步发送不会修改调用者的缓冲区。 */
    HAL_StatusTypeDef result = HAL_I2C_Mem_Write((I2C_HandleTypeDef *)user,
        (uint16_t)(address7 << 1), control, I2C_MEMADD_SIZE_8BIT,
        (uint8_t *)data, length, timeout_ms);
    return result == HAL_OK ? OLED_OK : OLED_IO_ERROR;
}

OLED_Result IntOLED_Init(uint32_t now_ms)
{
    const OLED_Config config = {
        128u, 64u, 0u, 0x3cu, s_panel_init, sizeof(s_panel_init), 20u, 500u, 100u
    };
    const OLED_Port port = {hal_write, &hi2c1};
    return OLED_Init(&s_display, &config, &port, now_ms);
}

OLED_Device *IntOLED_GetDevice(void)
{
    return &s_display;
}

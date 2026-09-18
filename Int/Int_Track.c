#include "Int_Track.h"


//数组存放8路灰度传感器传回的数值
uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];

//灰度传感器 -> MCU 引脚映射表(数据驱动:调整通道顺序只改此表)
typedef struct {
    GPIO_TypeDef *Port;
    uint16_t      Pin;
} GrayscalePin_t;

static const GrayscalePin_t s_grayscalePins[GRAYSCALE_SENSOR_CHANNELS] = {
    { IN1_GPIO_Port, IN1_Pin },
    { IN2_GPIO_Port, IN2_Pin },
    { IN3_GPIO_Port, IN3_Pin },
    { IN4_GPIO_Port, IN4_Pin },
    { IN5_GPIO_Port, IN5_Pin },
    { IN6_GPIO_Port, IN6_Pin },
    { IN7_GPIO_Port, IN7_Pin },
    { IN8_GPIO_Port, IN8_Pin },
};

void Read_All_Track(uint16_t *sensorValue)
{
    for (uint8_t i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++)
    {
        sensorValue[i] = (uint16_t)HAL_GPIO_ReadPin(s_grayscalePins[i].Port,
                                                    s_grayscalePins[i].Pin);
    }
}

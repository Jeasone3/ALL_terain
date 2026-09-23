#ifndef __INT_TRACK_H__
#define __INT_TRACK_H__

#include "main.h"

#define GRAYSCALE_SENSOR_CHANNELS 8

//读取8路灰度传感器,数值(0/1)写入 sensorValue[0..7],对应 IN1 -> IN8
void Read_All_Track(uint16_t *sensorValue);

#endif /* __INT_TRACK_H__ */

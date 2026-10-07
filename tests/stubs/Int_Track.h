#ifndef TEST_INT_TRACK_H
#define TEST_INT_TRACK_H

#include "main.h"

#define GRAYSCALE_SENSOR_CHANNELS 8
extern uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];
void Read_All_Track(uint16_t *sensor_values);

#endif

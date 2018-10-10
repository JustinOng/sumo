#ifndef PCNT_SPEED_TASK
#define PCNT_SPEED_TASK

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/pcnt.h"

#define SAMPLE_PERIOD_TICKS 50

#define SPEED_CHANNELS_NUM 1

extern const uint8_t SPEED_GPIOS[SPEED_CHANNELS_NUM];
extern const uint8_t SPEED_UNITS[SPEED_CHANNELS_NUM];

extern int16_t SpeedInputs[SPEED_CHANNELS_NUM];

void pcnt_speed_task(void *pvParameter);

#endif
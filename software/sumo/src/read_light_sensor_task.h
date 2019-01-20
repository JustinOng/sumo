#ifndef READ_LIGHT_SENSOR_TASK
#define READ_LIGHT_SENSOR_TASK

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/adc.h>
#include "esp_adc_cal.h"
#include "esp_log.h"

#define IR_CHANNELS_NUM 4

// how many samples to average an reading over
#define IR_SAMPLE_NUM 64
// decrease to be considered white
#define IR_THRESHOLD 500

void read_light_sensor_task(void *pvParameter);
void update_light_sensors();

// 1 for white seen, 0 for black
extern uint8_t Line_Seen[IR_CHANNELS_NUM];
extern uint16_t IR_sensors_values[IR_CHANNELS_NUM];

enum IR_sensors {
    FRONT_LEFT = 0,
    FRONT_RIGHT,
    REAR_LEFT,
    REAR_RIGHT
};

#endif 
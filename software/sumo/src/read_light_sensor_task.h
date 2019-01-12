#ifndef READ_LIGHT_SENSOR_TASK
#define READ_LIGHT_SENSOR_TASK

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/adc.h>
#include "esp_adc_cal.h"
#include "esp_log.h"

#define SAMPLE_NUM 16

void read_light_sensor_task(void *pvParameter);

#endif 
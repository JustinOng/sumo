#ifndef READ_VL53L0x_TASK
#define READ_VL53L0x_TASK

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "vl53l0x_api.h"

#define I2C_SDA GPIO_NUM_23
#define I2C_SCL GPIO_NUM_22
#define I2C_NUM I2C_NUM_0
#define I2C_FREQ 100000

#define SENSORS_NUM 2

#define POWER_CONTROL_PIN GPIO_NUM_2

void read_vl53l0x_task(void *pvParamter);

#endif
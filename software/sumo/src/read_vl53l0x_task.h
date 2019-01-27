#ifndef READ_VL53L0x_TASK
#define READ_VL53L0x_TASK

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "vl53l0x_api.h"

#define PROXIMITY_SENSOR_MAX 500
#define PROXIMITY_SENSOR_SAMPLES 32

#define I2C_SDA GPIO_NUM_21
#define I2C_SCL GPIO_NUM_19
#define I2C_NUM I2C_NUM_0
#define I2C_FREQ 100000

#define SENSORS_NUM 4

#define POWER_CONTROL_PIN GPIO_NUM_22

// averaged values (PROXIMITY_SENSOR_SAMPLES)
extern uint16_t Proximity_Sensors[SENSORS_NUM];
// raw values read by the sensors
extern uint16_t Proximity_Sensors_Raw[SENSORS_NUM];
extern uint8_t Sensor_Ok[SENSORS_NUM];

void read_vl53l0x_task(void *pvParamter);

#endif
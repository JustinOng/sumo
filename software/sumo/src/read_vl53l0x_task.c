#include "read_vl53l0x_task.h"

static const char* TAG = "VL53L0X_READ";
uint16_t Proximity_Sensors[SENSORS_NUM] = {0};
uint16_t Proximity_Sensors_Raw[SENSORS_NUM] = {0};
uint8_t Sensor_Ok[SENSORS_NUM] = {0};

uint32_t sum_distance[SENSORS_NUM] = {0};
uint16_t samples[SENSORS_NUM] = {0};

void read_vl53l0x_task(void *pvParamter) {
    struct VL53L0X_Data sensor_config[SENSORS_NUM] = {
        {
            .port = I2C_NUM,
            .address = 0x29
        },
        {
            .port = I2C_NUM,
            .address = 0x29
        }
    };

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_SCL;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;

	ESP_ERROR_CHECK(i2c_param_config(sensor_config[0].port, &conf));

    ESP_ERROR_CHECK(i2c_driver_install(sensor_config[0].port, I2C_MODE_MASTER, 0, 0, 0));

    gpio_pad_select_gpio(POWER_CONTROL_PIN);
    gpio_set_direction(POWER_CONTROL_PIN, GPIO_MODE_OUTPUT);

    uint8_t sensor_count = 0;
    uint8_t init_successful = 0;

    while(init_successful == 0) {
        sensor_count = 0;
        for(uint8_t u = 0; u < SENSORS_NUM; u++) {
            sensor_config[u].address = 0x29;
            Sensor_Ok[u] = 0;
        }

        ESP_LOGI(TAG, "Initialising sensors...");
        gpio_set_level(POWER_CONTROL_PIN, 0);
        vTaskDelay(500/portTICK_PERIOD_MS);

        gpio_set_level(POWER_CONTROL_PIN, 1);
        vTaskDelay(10/portTICK_PERIOD_MS);

        TickType_t start_ticks = xTaskGetTickCount();
        while(1) {
            if (setAddress(&sensor_config[sensor_count], 0x10+sensor_count) == ESP_OK) {
                ESP_LOGI(TAG, "Found sensor, readdressed to %.2x", 0x10+sensor_count);
                sensor_count++;
            }
            
            if ((xTaskGetTickCount() - start_ticks) * portTICK_PERIOD_MS > 1000 || sensor_count >= SENSORS_NUM) {
                ESP_LOGI(TAG, "Identified %d VL53L0X sensors", sensor_count);
                break;
            }
        }

        if (sensor_count < SENSORS_NUM) {
            continue;
        }

        esp_err_t ok;
        for (uint8_t i = 0; i < sensor_count; i++) {
            ok = vl53l0x_init(&sensor_config[i]);
            if (ok != ESP_OK) {
                ESP_LOGI(TAG, "Failed to initialise %.2x", sensor_config[i].address);
                break;
            }

            Sensor_Ok[i] = 1;
            startContinuous(&sensor_config[i]);
        }

        if (ok != ESP_OK) continue;

        init_successful = 1;
        break;
    }
    
    uint16_t range = 0;
    while(1) {
        for (uint8_t i = 0; i < sensor_count; i++) {
            if (readRangeContinuousMillimeters(&sensor_config[i], &range) == ESP_OK) {
                if (range < PROXIMITY_SENSOR_MAX) {
                    sum_distance[i] += range;
                    samples[i] ++;

                    while(samples[i] > PROXIMITY_SENSOR_SAMPLES) {
                        sum_distance[i] -= sum_distance[i] / (samples[i] > 0 ? samples[i] : 1);
                        samples[i] --;
                    }
                    Proximity_Sensors[i] = sum_distance[i] / (samples[i] > 0 ? samples[i] : 1);
                } else {
                    sum_distance[i] -= sum_distance[i] / (samples[i] > 0 ? samples[i] : 1);
                    samples[i] --;
                }

                Proximity_Sensors_Raw[i] = range;
            }
        }

        taskYIELD();
    }
}
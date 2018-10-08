#include "read_vl53l0x_task.h"

static const char* TAG = "VL53L0X_READ";

void read_vl53l0x_task(void *pvParamter) {
    struct VL53L0X_Data c[SENSORS_NUM] = {
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

	ESP_ERROR_CHECK(i2c_param_config(c[0].port, &conf));

    ESP_ERROR_CHECK(i2c_driver_install(c[0].port, I2C_MODE_MASTER, 0, 0, 0));

    gpio_pad_select_gpio(POWER_CONTROL_PIN);
    gpio_set_direction(POWER_CONTROL_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(POWER_CONTROL_PIN, 1);

    vTaskDelay(10/portTICK_PERIOD_MS);

    uint8_t sensor_count = 0;

    TickType_t start_ticks = xTaskGetTickCount();
    while(1) {
        if (setAddress(&c[sensor_count], 0x10+sensor_count) == ESP_OK) {
            ESP_LOGI(TAG, "Found sensor, readdressed to %.2x", 0x10+sensor_count);
            sensor_count++;
        }
        
        if ((xTaskGetTickCount() - start_ticks) * portTICK_PERIOD_MS > 1000) {
            ESP_LOGI(TAG, "Timed out");
            break;
        }
    }

    for (uint8_t i = 0; i < sensor_count; i++) {
        if (vl53l0x_init(&c[i]) != ESP_OK) {
            ESP_LOGI(TAG, "Failed to initialise %.2x", c[i].address);
            continue;
        }

        startContinuous(&c[i]);
    }

    uint16_t r0 = 0, r1 = 0;
    while(1) {
        if (
            readRangeContinuousMillimeters(&c[0], &r0) == ESP_OK &&
            readRangeContinuousMillimeters(&c[1], &r1) == ESP_OK
        ) {
            ESP_LOGI(TAG, "%d %d", r0, r1);
        }
        vTaskDelay(50/portTICK_PERIOD_MS);
    }
}
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

    bool ok = vl53l0x_init(&c[0]);
    if (!ok) {
        ESP_LOGI(TAG, "Failed to initialise!");
        vTaskDelete(NULL);
    }

    setAddress(&c[0], 0x10);

    TickType_t start_ticks = xTaskGetTickCount();
    while(1) {
        if (vl53l0x_init(&c[1])) {
            setAddress(&c[0], 0x11);
            ESP_LOGI(TAG, "Initialised second sensor");
            break;
        }
        if ((xTaskGetTickCount() - start_ticks) * portTICK_PERIOD_MS > 100) {
            ESP_LOGI(TAG, "Timed out");
            break;
        }
    }

    vTaskDelete(NULL);

    startContinuous(&c[0]);

    while(1) {
        ESP_LOGI(TAG, "%d", readRangeContinuousMillimeters(&c[0]));
        vTaskDelay(50/portTICK_PERIOD_MS);
    }
}
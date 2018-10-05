#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "config.h"
#include "rmt_listen_rx_task.h"
#include "ledc_pwm_task.h"

#include "vl53l0x_api.h"

static const char* TAG = "main";

uint16_t scaleReceiver(uint16_t input) {
    // takes in a value from RECEIVER_CH_MIN to RECEIVER_CH_MAX centered on RECEIVER_CH_DEADZONE and returns
    // a uint16_t where the highest bit denotes direction and the lower 10 bits denote speed
    // map function taken from https://www.arduino.cc/reference/en/language/functions/math/map/
    const uint16_t out_min = 0;
    const uint16_t out_max = 1023;

    // if input out of range force it to be within
    if (input < RECEIVER_CH_MIN) {
        input = RECEIVER_CH_MIN;
    } else if (input > RECEIVER_CH_MAX) {
        input = RECEIVER_CH_MAX;
    }
    
    // if input within deadzone, return 0
    if (input > (RECEIVER_CH_CENTER-RECEIVER_CH_DEADZONE) && input < (RECEIVER_CH_CENTER+RECEIVER_CH_DEADZONE)) {
        return 0;
    }

    // scale a value from [RECEIVER_CH_MIN, RECEIVER_CH_CENTER] to [out_min, out_max]
    if (input < RECEIVER_CH_CENTER) {
        return 0x8000 | ((RECEIVER_CH_CENTER - input) * (out_max - out_min) / (RECEIVER_CH_CENTER - RECEIVER_CH_MIN) + out_min);
    } else
    // scale a value from [RECEIVER_CH_CENTER, RECEIVER_CH_MAX] to [out_min, out_max]
    {
        return (input - RECEIVER_CH_CENTER) * (out_max - out_min) / (RECEIVER_CH_MAX - RECEIVER_CH_CENTER) + out_min;
    }
}

void write_motor_task(void *pvParameter) {
    while(1) {
        MotorControl[0] = scaleReceiver(ReceiverChannels[0]);
        vTaskDelay(1);
    }
}

void dump_task(void *pvParameter) {
    while(1) {
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

void test(void *pvParameter) {
    struct VL53L0X_Data c = {
        .port = I2C_NUM_0,
        .address = 0x29
    };

    /*i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_NUM_23;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = GPIO_NUM_22;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config(c.port, &conf);
    i2c_driver_install(c.port, conf.mode, 0, 0, 0);*/

	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = GPIO_NUM_23;
	conf.scl_io_num = GPIO_NUM_22;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 100000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));

    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    bool ok = vl53l0x_init(&c);
    if (!ok) {
        ESP_LOGI(TAG, "Failed to initialise!");
        vTaskDelete(NULL);
    }

    startContinuous(&c);

    while(1) {
        ESP_LOGI(TAG, "%d", readRangeContinuousMillimeters(&c));
        vTaskDelay(50/portTICK_PERIOD_MS);
    }
}

void test1(void *ignore) {
	ESP_LOGD(TAG, ">> i2cScanner");
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = GPIO_NUM_23;
	conf.scl_io_num = GPIO_NUM_22;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 100000;
	i2c_param_config(I2C_NUM_0, &conf);

	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

	int i;
	esp_err_t espRc;
	printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
	printf("00:         ");
	for (i=3; i< 0x78; i++) {
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
		i2c_master_stop(cmd);

		espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
		if (i%16 == 0) {
			printf("\n%.2x:", i);
		}
		if (espRc == 0) {
			printf(" %.2x", i);
		} else {
			printf(" --");
		}
		ESP_LOGD(TAG, "i=%d, rc=%d (0x%x)", i, espRc, espRc);
		i2c_cmd_link_delete(cmd);
	}
	printf("\n");
	vTaskDelete(NULL);
}

void app_main()
{
    ESP_LOGI(TAG, "Started");
    /*xTaskCreate(&dump_task, "dump_task", 2048, NULL, 1, NULL);
    xTaskCreate(&write_motor_task, "write_motor_task", 2048, NULL, 10, NULL);
    xTaskCreate(&ledc_pwm_task, "ledc_pwm_task", 2048, NULL, 5, NULL);
    xTaskCreate(&rmt_listen_rx_task, "rmt_listen_rx_task", 2048, NULL, 5, NULL);*/
    xTaskCreate(&test, "test", 2048, NULL, 5, NULL);
}
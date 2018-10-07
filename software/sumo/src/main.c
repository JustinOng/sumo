#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "config.h"
#include "rmt_listen_rx_task.h"
#include "ledc_pwm_task.h"
#include "read_vl53l0x_task.h"

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

void app_main()
{
    ESP_LOGI(TAG, "Started");
    /*xTaskCreate(&dump_task, "dump_task", 2048, NULL, 1, NULL);
    xTaskCreate(&write_motor_task, "write_motor_task", 2048, NULL, 10, NULL);
    xTaskCreate(&ledc_pwm_task, "ledc_pwm_task", 2048, NULL, 5, NULL);
    xTaskCreate(&rmt_listen_rx_task, "rmt_listen_rx_task", 2048, NULL, 5, NULL);*/
    xTaskCreate(&read_vl53l0x_task, "read_vl53l0x_task", 2048, NULL, 5, NULL);
}
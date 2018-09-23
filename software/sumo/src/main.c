#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "rmt_listen_rx_task.h"
#include "ledc_pwm_task.h"

static const char* TAG = "main";

uint16_t scaleReceiver(uint16_t x) {
    uint16_t in_min = 8000;
    uint16_t in_max = 16000;
    uint16_t out_min = 0;
    uint16_t out_max = 1023;
    // scales a input from [in_min, in_max] to [out_min, out_max]

    if (x < in_min) return out_min;
    if (x > in_max) return out_max;

    // taken from https://www.arduino.cc/reference/en/language/functions/math/map/

    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void write_motor_task(void *pvParameter) {
    while(1) {
        MotorControl[0] = scaleReceiver(ReceiverChannels[0]);
        vTaskDelay(1);
    }
}

void dump_task(void *pvParameter) {
    while(1) {
        ESP_LOGI(TAG, "%d", MotorControl[0]);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

void app_main()
{
    ESP_LOGI(TAG, "Started");
    xTaskCreate(&dump_task, "dump_task", 2048, NULL, 1, NULL);
    xTaskCreate(&write_motor_task, "write_motor_task", 2048, NULL, 10, NULL);
    xTaskCreate(&ledc_pwm_task, "ledc_pwm_task", 2048, NULL, 5, NULL);
    xTaskCreate(&rmt_listen_rx_task, "rmt_listen_rx_task", 2048, NULL, 5, NULL);
}
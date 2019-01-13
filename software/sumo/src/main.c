#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "config.h"
#include "configure_rmt.h"
#include "motor_control_task.h"
#include "read_vl53l0x_task.h"
#include "read_light_sensor_task.h"

static const char* TAG = "main";

int16_t scaleReceiver(uint16_t input) {
    // takes in a value from RECEIVER_CH_MIN to RECEIVER_CH_MAX centered on RECEIVER_CH_CENTER
    // and returns direction and speed
    // map function taken from https://www.arduino.cc/reference/en/language/functions/math/map/
    const int16_t out_min = -MAX_MOTOR_SPEED;
    const int16_t out_max = MAX_MOTOR_SPEED;

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

    return (input - RECEIVER_CH_MIN) * (out_max - out_min) / (RECEIVER_CH_MAX - RECEIVER_CH_MIN) + out_min;
}

void write_motor_task(void *pvParameter) {
    while(1) {
        int16_t forward = scaleReceiver(ReceiverChannels[1]) * SCALE_FORWARD;
        int16_t turn = scaleReceiver(ReceiverChannels[0]) * SCALE_TURN;

        int16_t left_state = forward + turn;
        int16_t right_state = forward - turn;

        //ESP_LOGI(TAG, "l: %d, r: %d", left_state, right_state);

        set_motor_dir(0, left_state < 0 ? 1 : 0);
        set_motor_speed(0, abs(left_state));

        set_motor_dir(1, right_state < 0 ? 1 : 0);
        set_motor_speed(1, abs(right_state));

        vTaskDelay(1);
    }
}

void logging_task(void *pvParameter) {
    while(1) {
        ESP_LOGI(TAG, "%llu", last_pulse_length);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    ESP_LOGI(TAG, "Started");
    rmt_init();
    //xTaskCreate(&logging_task, "logging_task", 2048, NULL, 5, NULL);
    xTaskCreate(&motor_control_task, "motor_control_task", 2048, NULL, 5, NULL);
    xTaskCreate(&read_vl53l0x_task, "read_vl53l0x_task", 2048, NULL, 5, NULL);
    //xTaskCreate(&read_light_sensor_task, "read_light_sensor_task", 2048, NULL, 5, NULL);
    xTaskCreate(&write_motor_task, "write_motor_task", 2048, NULL, 5, NULL);
}
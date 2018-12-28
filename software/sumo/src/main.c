#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "config.h"
#include "rmt_listen_rx_task.h"
#include "ledc_pwm_task.h"
#include "read_vl53l0x_task.h"

#define ESP_INTR_FLAG_DEFAULT 0

static const char* TAG = "main";

uint16_t scaleReceiver(uint16_t input) {
    // takes in a value from RECEIVER_CH_MIN to RECEIVER_CH_MAX centered on RECEIVER_CH_DEADZONE and returns
    // a uint16_t where the highest bit denotes direction and the lower 10 bits denote speed
    // map function taken from https://www.arduino.cc/reference/en/language/functions/math/map/
    const uint16_t out_min = 50;
    const uint16_t out_max = 150;

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
    uint32_t millis = 0;
    // tracks when the motor was last powered
    uint32_t last_powered = 0;
    // tracks when the last throttle reset occured to limit how often it resets
    uint32_t last_reset = 0;

    while(1) {
        millis = esp_timer_get_time() / 1000;
        uint16_t speed = scaleReceiver(ReceiverChannels[1]);
        
        if ((0x3FF & speed) > 50) {
            if (last_powered == 0) {
                last_powered = millis;
            }
        } else {
            last_powered = 0;
        }

        if ((millis - last_reset) > 1000) {
            last_reset = millis;
            if ((millis - last_pulse) > 5 && (last_powered > 0 && (millis - last_powered) > 10)) {
                ESP_LOGI(TAG, "Resetting");
                MotorControl[0] = speed & 0x8000;
                vTaskDelay(50 / portTICK_PERIOD_MS);
                speed = scaleReceiver(ReceiverChannels[1]);
            }
        }

        MotorControl[0] = speed;

        vTaskDelay(1);
    }
}

void logging_task(void *pvParameter) {
    while(1) {
        ESP_LOGI(TAG, "%d %d %d", scaleReceiver(ReceiverChannels[0]), scaleReceiver(ReceiverChannels[1]), scaleReceiver(ReceiverChannels[2]));
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    ESP_LOGI(TAG, "Started");
    //xTaskCreate(&logging_task, "logging_task", 2048, NULL, 5, NULL);
    xTaskCreate(&ledc_pwm_task, "ledc_pwm_task", 2048, NULL, 5, NULL);
    xTaskCreate(&rmt_listen_rx_task, "rmt_listen_rx_task", 2048, NULL, 5, NULL);
    xTaskCreate(&read_vl53l0x_task, "read_vl53l0x_task", 2048, NULL, 5, NULL);
    xTaskCreate(&write_motor_task, "write_motor_task", 2048, NULL, 5, NULL);
    //xTaskCreate(&pcnt_speed_task, "pcnt_speed_task", 2048, NULL, 5, NULL);
}
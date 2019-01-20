#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "math.h"

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
        int16_t forward = scaleReceiver(ReceiverChannels[1]);
        int16_t turn = scaleReceiver(ReceiverChannels[0]) * SCALE_TURN;

        // solve for k in y=0.1*2^(kx)-0.1 where y=40, x=200
        forward = (forward < 0 ? -1 : 1) * (pow(2, 0.0432 * abs(forward)) - 1) * SCALE_FORWARD;

        int16_t left_state = forward + turn;
        int16_t right_state = forward - turn;

        //ESP_LOGI(TAG, "l: %d, r: %d", left_state, right_state);

        update_light_sensors();

        set_motor_dir(0, left_state < 0 ? 1 : 0);
        // if line sensors see the boundary and we're trying to move in that direction,
        // zero out the speed
        if (
            (Line_Seen[FRONT_LEFT] && left_state < 0) ||
            (Line_Seen[REAR_LEFT] && left_state > 0)
        ) {
            set_motor_speed(0, 0);
            set_motor_brake(0, 1);
        } else {
            set_motor_speed(0, abs(left_state));
            set_motor_brake(0, 0);
        }

        set_motor_dir(1, right_state < 0 ? 1 : 0);
        if (
            (Line_Seen[FRONT_RIGHT] && right_state < 0) ||
            (Line_Seen[REAR_RIGHT] && right_state > 0)
        ) {
            set_motor_speed(1, 0);
            set_motor_brake(1, 1);
        } else {
            set_motor_speed(1, abs(right_state));
            set_motor_brake(1, 0);
        }

        taskYIELD();
    }
}

void logging_task(void *pvParameter) {
    while(1) {
        /*ESP_LOGI(TAG, "%d(%d) %d(%d) %d(%d) %d(%d)",
            IR_sensors_values[0], Line_Seen[0],
            IR_sensors_values[1], Line_Seen[1],
            IR_sensors_values[2], Line_Seen[2],
            IR_sensors_values[3], Line_Seen[3]
        );*/
        ESP_LOGI(TAG, "%d %d", Proximity_Sensors[0], Proximity_Sensors[1]);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    ESP_LOGI(TAG, "Started");
    rmt_init();
    xTaskCreate(&logging_task, "logging_task", 2048, NULL, 5, NULL);
    xTaskCreate(&motor_control_task, "motor_control_task", 2048, NULL, 10, NULL);
    xTaskCreate(&read_vl53l0x_task, "read_vl53l0x_task", 2048, NULL, 5, NULL);
    xTaskCreate(&read_light_sensor_task, "read_light_sensor_task", 2048, NULL, 5, NULL);
    xTaskCreate(&write_motor_task, "write_motor_task", 2048, NULL, 5, NULL);
}
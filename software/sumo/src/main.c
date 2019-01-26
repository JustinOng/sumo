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
#include "driver/gpio.h"
#include "ws2812.h"

static const char* TAG = "main";

#define FORCE_RANGE(val, range) (fmin(range, abs(val)) * (val < 0 ? -1 : 1))

int16_t scaleReceiver(uint16_t input, int16_t out_min, int16_t out_max) {
    // takes in a value from RECEIVER_CH_MIN to RECEIVER_CH_MAX centered on RECEIVER_CH_CENTER
    // and returns direction and speed
    // map function taken from https://www.arduino.cc/reference/en/language/functions/math/map/

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
    uint8_t rc_mode_entered = 0;
    uint8_t p_rc_mode = 0, rc_mode = 0;

    uint32_t sum_distance = 0;
    uint16_t samples = 0;
    while(1) {
        rc_mode = ReceiverChannels[2] > (RECEIVER_CH_CENTER+RECEIVER_CH_DEADZONE);

        if (rc_mode != p_rc_mode) {
            ESP_LOGI(TAG, "RC mode: %d", rc_mode)
            p_rc_mode = rc_mode;
        }

        if (rc_mode) {
            rc_mode_entered = 1;
            sum_distance = 0;
            samples = 0;
        }

        if (!rc_mode && rc_mode_entered) {
            int16_t speed_left = 0, speed_right = 0;

            // at least one of the proximity sensors don't see anything
            // if left: turn right
            // if right: turn left
            // if both: turn right
            if (
                (Proximity_Sensors[0] > PROXIMITY_SENSOR_MAX) ||
                (Proximity_Sensors[1] > PROXIMITY_SENSOR_MAX)
            ) {
                if (Proximity_Sensors[1] > PROXIMITY_SENSOR_MAX) {
                    speed_left = -1;
                    speed_right = 2;
                } else {
                    speed_left = 2;
                    speed_right = -1;
                }
            } else {
                // both sensors see something.
                uint16_t ave_distance = sum_distance / (samples > 0 ? samples : 1);
                int16_t base_speed = ave_distance < 50 ? 70 : ave_distance < 70 ? 50 : 1    0;
                speed_left = base_speed + Proximity_Sensors[1] * 0.02;
                speed_right = base_speed + Proximity_Sensors[2] * 0.02;

                sum_distance += Proximity_Sensors[1] + Proximity_Sensors[2];
                samples += 2;

                while(samples > 32) {
                    sum_distance -= sum_distance / samples;
                    samples--;
                }
            }

            speed_left = (int16_t) FORCE_RANGE(speed_left, 100);
            speed_right = (int16_t) FORCE_RANGE(speed_right, 100);

            ESP_LOGI(TAG, "left: %d right: %d distance: %d", speed_left, speed_right, sum_distance / (samples > 0 ? samples : 1));
            
            set_motor_dir(0, speed_left < 0 ? 1 : 0);
            set_motor_speed(0, abs(speed_left));

            set_motor_dir(1, speed_right < 0 ? 1 : 0);
            set_motor_speed(1, abs(speed_right));
        } else {
            int16_t forward = scaleReceiver(ReceiverChannels[1], -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
            int16_t turn = scaleReceiver(ReceiverChannels[0], -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED) * SCALE_TURN;

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

            update_motors();
        }

        vTaskDelay(1);
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

void lighting_task(void *pvParameter) {
    rgbVal pixels[4];
    while(1) {
        pixels[0].r = 50;

        for(uint8_t i = 1; i <= 2; i ++) {
            uint8_t sensor_num = i - 1;
            if (!Sensor_Ok[sensor_num]) {
                pixels[i].r = 128;
                pixels[i].g = 0;
            } else {
                uint16_t val = Proximity_Sensors[sensor_num];

                if (val > PROXIMITY_SENSOR_MAX) val = PROXIMITY_SENSOR_MAX;

                pixels[i].r = 0;
                pixels[i].g = (PROXIMITY_SENSOR_MAX - val) * 255 / PROXIMITY_SENSOR_MAX;
                ESP_LOGI(TAG, "%d %d", pixels[i].g, val);
            }
        }

        pixels[3].r = 50;
        
        ws2812_setColors(4, pixels);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    ESP_LOGI(TAG, "Started");
    rmt_init();
    ws2812_init(0);
    //xTaskCreate(&logging_task, "logging_task", 2048, NULL, 5, NULL);
    xTaskCreate(&lighting_task, "lighting_task", 2048, NULL, 5, NULL);
    xTaskCreate(&motor_control_task, "motor_control_task", 2048, NULL, 5, NULL);
    xTaskCreate(&write_motor_task, "write_motor_task", 2048, NULL, 10, NULL);
    xTaskCreate(&read_vl53l0x_task, "read_vl53l0x_task", 4096, NULL, 5, NULL);
    xTaskCreate(&read_light_sensor_task, "read_light_sensor_task", 2048, NULL, 5, NULL);
}
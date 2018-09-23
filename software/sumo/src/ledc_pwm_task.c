#include "ledc_pwm_task.h"

static const char* TAG = "LEDC";
uint16_t MotorControl[MOTOR_CHANNELS_NUM] = {0};

static ledc_channel_config_t ledc_channel[MOTOR_CHANNELS_NUM] = {
    {
        .channel    = MOTOR_LEFT_CHANNEL,
        .duty       = 0,
        .gpio_num   = MOTOR_LEFT_GPIO_NUM,
        .speed_mode = LEDC_SPEED_MODE,
        .timer_sel  = LEDC_TIMER
    }
};

void ledc_pwm_task(void *pvParameter) {
    uint8_t i;

    ledc_init();

    while(1) {
        for (i = 0; i < MOTOR_CHANNELS_NUM; i++) {
            ledc_set_duty(ledc_channel[i].speed_mode, ledc_channel[i].channel, 0x3FF & MotorControl[i]);
            ledc_update_duty(ledc_channel[i].speed_mode, ledc_channel[i].channel);
        }
        vTaskDelay(1);
    }
}

void ledc_init(void) {
    uint8_t i;

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = LEDC_FREQUENCY,
        .speed_mode = LEDC_SPEED_MODE,
        .timer_num = LEDC_TIMER
    };

    ledc_timer_config(&ledc_timer);

    for (i = 0; i < MOTOR_CHANNELS_NUM; i++) {
        ledc_channel_config(&ledc_channel[i]);
    }
}
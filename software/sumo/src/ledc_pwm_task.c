#include "ledc_pwm_task.h"

static const char* TAG = "LEDC";
uint16_t MotorControl[MOTOR_CHANNELS_NUM] = {0};

static ledc_channel_config_t ledc_channel[MOTOR_CHANNELS_NUM] = {
    {
        .channel    = MOTOR_LEFT_CHANNEL,
        .duty       = 0,
        .gpio_num   = MOTOR_LEFT_THROTTLE_GPIO_NUM,
        .speed_mode = LEDC_SPEED_MODE,
        .timer_sel  = LEDC_TIMER
    }
};

void ledc_pwm_task(void *pvParameter) {
    uint8_t i;

    ledc_init();

    while(1) {
        for (i = 0; i < MOTOR_CHANNELS_NUM; i++) {
            if ((0x3FF & MotorControl[i]) > 80) {
                ledc_set_duty(ledc_channel[i].speed_mode, ledc_channel[i].channel, 0x3FF & MotorControl[i]);
            } else {
                ledc_set_duty(ledc_channel[i].speed_mode, ledc_channel[i].channel, 0);
            }

            ledc_update_duty(ledc_channel[i].speed_mode, ledc_channel[i].channel);
        }
        gpio_set_level(MOTOR_LEFT_DIR_NUM, 0x8000 & MotorControl[0]);
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

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << MOTOR_LEFT_DIR_NUM);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
}
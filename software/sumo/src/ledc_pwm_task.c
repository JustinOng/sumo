#include "ledc_pwm_task.h"

static const char* TAG = "LEDC";
uint16_t MotorControl[MOTOR_CHANNELS_NUM] = {0};
volatile uint32_t last_pulse = 0;

static ledc_channel_config_t ledc_channel[MOTOR_CHANNELS_NUM] = {
    {
        .channel    = MOTOR_LEFT_CHANNEL,
        .duty       = 0,
        .gpio_num   = MOTOR_LEFT_THROTTLE_NUM,
        .speed_mode = LEDC_SPEED_MODE,
        .timer_sel  = LEDC_TIMER
    }, {
        .channel    = MOTOR_RIGHT_CHANNEL,
        .duty       = 0,
        .gpio_num   = MOTOR_RIGHT_THROTTLE_NUM,
        .speed_mode = LEDC_SPEED_MODE,
        .timer_sel  = LEDC_TIMER
    }
};

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t motor_index = (uint32_t) arg;
    int64_t cur_time = esp_timer_get_time();

    last_pulse = (uint64_t) cur_time / 1000;
}

void ledc_pwm_task(void *pvParameter) {
    uint8_t i;
    uint32_t millis = 0;

    ledc_init();

    while(1) {
        millis = (uint64_t) esp_timer_get_time() / 1000;

        for (i = 0; i < MOTOR_CHANNELS_NUM; i++) {
            if ((0x3FF & MotorControl[i]) > 80) {
                ledc_set_duty(ledc_channel[i].speed_mode, ledc_channel[i].channel, 0x3FF & MotorControl[i]);
            } else {
                ledc_set_duty(ledc_channel[i].speed_mode, ledc_channel[i].channel, 0);
            }

            ledc_update_duty(ledc_channel[i].speed_mode, ledc_channel[i].channel);
        }
        gpio_set_level(MOTOR_LEFT_DIR_NUM, 0x8000 & MotorControl[0]);
        /*if ((0x3FF & MotorControl[0]) == 0) {
            gpio_set_level(MOTOR_LEFT_BRAKE_NUM, 1);
        } else {
            gpio_set_level(MOTOR_LEFT_BRAKE_NUM, 0);
        }*/
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
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << MOTOR_LEFT_DIR_NUM) | (1ULL << MOTOR_LEFT_BRAKE_NUM);
    io_conf.pull_down_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
}

void speed_isr_init() {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = (1ULL << MOTOR_LEFT_SPEED_NUM) | (1ULL << MOTOR_RIGHT_SPEED_NUM);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(MOTOR_LEFT_SPEED_NUM, gpio_isr_handler, (void*) 0);
    gpio_isr_handler_add(MOTOR_RIGHT_SPEED_NUM, gpio_isr_handler, (void*) 1);
}
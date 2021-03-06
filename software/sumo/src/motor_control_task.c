#include "motor_control_task.h"

static const char* TAG = "LEDC";

typedef struct {
    // requested direction
    uint8_t dir;
    // time at which dir last changed
    int64_t last_dir_change;
    // requested speed
    uint16_t speed;
} MotorInternalState;

MotorInternalState Motors[MOTOR_CHANNELS_NUM] = {
    {
        .dir   = 0,
        .last_dir_change = 0,
        .speed = 0
    }, {
        .dir   = 0,
        .last_dir_change = 0,
        .speed = 0
    }
};

volatile uint64_t last_pulse_length = 0;
volatile uint64_t last_pulse = 0;

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

    last_pulse_length = (uint64_t) cur_time - last_pulse;
    last_pulse = cur_time;
}

static void motor_control_init(void) {
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
    io_conf.pin_bit_mask = 
        (1ULL << MOTOR_LEFT_DIR_NUM) |
        (1ULL << MOTOR_LEFT_BRAKE_NUM) |
        (1ULL << MOTOR_RIGHT_DIR_NUM) |
        (1ULL << MOTOR_RIGHT_BRAKE_NUM);
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
}

void speed_isr_init() {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = (1ULL << MOTOR_LEFT_SPEED_NUM) | (1ULL << MOTOR_RIGHT_SPEED_NUM);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(MOTOR_LEFT_SPEED_NUM, gpio_isr_handler, (void*) 0);
    gpio_isr_handler_add(MOTOR_RIGHT_SPEED_NUM, gpio_isr_handler, (void*) 1);
}

void update_motors() {    
    for (uint8_t i = 0; i < MOTOR_CHANNELS_NUM; i++) {
        if ((esp_timer_get_time() - Motors[i].last_dir_change) < DIR_CHANGE_DEAD_TIME) {
            ledc_set_duty(ledc_channel[i].speed_mode, ledc_channel[i].channel, 0);
        } else {
            ledc_set_duty(ledc_channel[i].speed_mode, ledc_channel[i].channel, Motors[i].speed);
        }
        ledc_update_duty(ledc_channel[i].speed_mode, ledc_channel[i].channel);
    }
    gpio_set_level(MOTOR_LEFT_DIR_NUM, INVERT_CHANNELS ? !Motors[0].dir : Motors[0].dir);
    gpio_set_level(MOTOR_RIGHT_DIR_NUM, INVERT_CHANNELS ? !Motors[1].dir : Motors[1].dir);
}

void motor_control_task(void *pvParameter) {
    motor_control_init();
    speed_isr_init();

    while(1) {
        update_motors();
        vTaskDelay(1);
    }
}

void set_motor_dir(uint8_t motor, uint8_t dir) {
    if (motor >= MOTOR_CHANNELS_NUM) return;

    if (Motors[motor].dir != dir) {
        if ((esp_timer_get_time() - Motors[motor].last_dir_change) < DIR_CHANGE_DEAD_TIME) {
            Motors[motor].last_dir_change = 0;
        } else {
            Motors[motor].last_dir_change = esp_timer_get_time();
        }
    }

    Motors[motor].dir = dir;
}

void set_motor_speed(uint8_t motor, uint16_t speed) {
    if (motor >= MOTOR_CHANNELS_NUM) return;

    if (speed > 0) {
        speed += MIN_MOTOR_SPEED;
    }

    if (speed > MAX_MOTOR_SPEED) speed = MAX_MOTOR_SPEED;

    Motors[motor].speed = speed;
}


void set_motor_brake(uint8_t motor, uint8_t brake) {
    if (motor >= MOTOR_CHANNELS_NUM) return;

    if (motor == 0) {
        gpio_set_level(MOTOR_LEFT_BRAKE_NUM, brake);
    } else if (motor == 1) {
        gpio_set_level(MOTOR_RIGHT_BRAKE_NUM, brake);
    }
}
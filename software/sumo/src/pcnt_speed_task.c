#include "pcnt_speed_task.h"

const uint8_t SPEED_GPIOS[SPEED_CHANNELS_NUM] = { 25 };
const uint8_t SPEED_UNITS[SPEED_CHANNELS_NUM] = { 0 };

int16_t SpeedInputs[SPEED_CHANNELS_NUM] = {0};

void pcnt_speed_task(void *pvParameter) {
    pcnt_config_t pcnt_config[SPEED_CHANNELS_NUM];

    for (uint8_t i = 0; i < SPEED_CHANNELS_NUM; i++) {
        pcnt_config[i].pulse_gpio_num = GPIO_NUM_25;
        pcnt_config[i].ctrl_gpio_num = PCNT_PIN_NOT_USED;
        pcnt_config[i].channel = PCNT_CHANNEL_0;
        pcnt_config[i].unit = SPEED_UNITS[i];
        pcnt_config[i].pos_mode = PCNT_COUNT_INC;
        pcnt_config[i].neg_mode = PCNT_COUNT_DIS;
        pcnt_config[i].lctrl_mode = PCNT_MODE_KEEP;
        pcnt_config[i].hctrl_mode = PCNT_MODE_KEEP;

        pcnt_unit_config(&pcnt_config[i]);
        pcnt_counter_clear(pcnt_config[i].unit);
    }

    vTaskDelete(NULL);

    while(1) {
        for (uint8_t i = 0; i < SPEED_CHANNELS_NUM; i++) {
            pcnt_get_counter_value(pcnt_config[i].unit, &SpeedInputs[i]);
        }
        vTaskDelay(1);
        if (xTaskGetTickCount() % SAMPLE_PERIOD_TICKS == 0) {
            for (uint8_t i = 0; i < SPEED_CHANNELS_NUM; i++) {
                pcnt_counter_clear(pcnt_config[i].unit);
            }
        }
    }
}
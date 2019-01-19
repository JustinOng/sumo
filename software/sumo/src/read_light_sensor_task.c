#include "read_light_sensor_task.h"

static const char* TAG = "LSENSOR";

uint8_t IR_sensors[IR_CHANNELS_NUM] = { 0 };
uint16_t IR_sensors_values[IR_CHANNELS_NUM] = { 0 };
uint16_t IR_sensors_base_values[IR_CHANNELS_NUM] = { 0 };


const adc1_channel_t channels[IR_CHANNELS_NUM] = {
    ADC1_CHANNEL_0,
    ADC1_CHANNEL_3,
    ADC1_CHANNEL_6,
    ADC1_CHANNEL_7
};

uint16_t read_adc(adc1_channel_t channel, uint8_t samples) {
    uint32_t sum = 0;
    
    for(uint8_t i = 0; i < samples; i++) {
        sum += (uint16_t) adc1_get_raw(channel);
    }

    return sum / samples;
}

void read_light_sensor_task(void *pvParameter) {
    adc1_config_width(ADC_WIDTH_BIT_12);

    for(uint8_t i = 0; i < IR_CHANNELS_NUM; i++) {
        adc1_config_channel_atten(channels[i], ADC_ATTEN_DB_11);
    }

    // read sensors and keep this as the "base" value
    for(uint8_t i = 0; i < IR_CHANNELS_NUM; i++) {
        IR_sensors_base_values[i] = read_adc(channels[i], IR_SAMPLE_NUM);
    }

    while(1) {
        for(uint8_t i = 0; i < IR_CHANNELS_NUM; i++) {
            IR_sensors_values[i] = read_adc(channels[i], IR_SAMPLE_NUM);
            // if the current value is more than IR_THRESHOLD below the base value
            // treat it as white seen
            if (IR_sensors_base_values[i] > (IR_sensors_values[i] + IR_THRESHOLD)) {
                Line_Seen[i] = 1;
            } else {
                Line_Seen[i] = 0;
            }
        }

        vTaskDelay(1);
    }
}
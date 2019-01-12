#include "read_light_sensor_task.h"

static const char* TAG = "LSENSOR";

void read_light_sensor_task(void *pvParameter) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);

    while(1) {
        uint32_t sum = 0;
        
        for(uint8_t i = 0; i < SAMPLE_NUM; i++) {
            sum += adc1_get_raw(ADC1_CHANNEL_0);
        }

        sum /= SAMPLE_NUM;

        ESP_LOGI(TAG, "val: %d", sum);
    }
}
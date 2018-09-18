#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "rmt_listen_rx_task.h"

static const char* TAG = "main";

void print_channels(void *pvParameter) {
    while(1) {
        ESP_LOGI(TAG, "%u", ReceiverChannels.ch1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    ESP_LOGI(TAG, "Started");
    xTaskCreate(&print_channels, "print_channels", 2048, NULL, 5, NULL);
    xTaskCreate(&rmt_listen_rx_task, "rmt_listen_rx_task", 2048, NULL, 5, NULL);
}
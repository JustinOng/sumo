#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "rmt_listen_rx_task.h"

static const char* TAG = "main";

void app_main()
{
    ESP_LOGI(TAG, "Started");
    xTaskCreate(&rmt_listen_rx_task, "rmt_listen_rx_task", 2048, NULL, 5, NULL);
}
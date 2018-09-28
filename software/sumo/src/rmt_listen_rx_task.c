#include "rmt_listen_rx_task.h"

static const char* TAG = "RMT";

uint16_t ReceiverChannels[RECEIVER_CHANNELS_NUM] = {0};
const uint8_t RECEIVER_CHANNELS[RECEIVER_CHANNELS_NUM] = { 0, 1, 2 };
const uint8_t RECEIVER_GPIOS[RECEIVER_CHANNELS_NUM] = { 12, 13, 14 };

void rmt_listen_rx_task(void *pvParameter) {
    // this task listens to the 3 channels of the receiver and outputs to the global struct receiver
    int channel = RECEIVER_CHANNELS[0];
    rmt_init_rx();
    RingbufHandle_t rb = NULL;
    
    rmt_get_ringbuf_handle(channel, &rb);
    rmt_rx_start(channel, 1);
    while(1) {
        size_t rx_size = 0;
        rmt_item32_t* item = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, 1000);
        if(item) {
            ReceiverChannels[0] = (uint16_t) item->duration0;
            vRingbufferReturnItem(rb, (void*) item);
        }
        vTaskDelay(1);
    }
}

void rmt_init_rx(void) {
    uint8_t i;

    rmt_config_t rmt_channels[RECEIVER_CHANNELS_NUM] = {};

    for (i = 0; i < RECEIVER_CHANNELS_NUM; i++) {
        rmt_channels[i].channel = RECEIVER_CHANNELS[i];
        rmt_channels[i].gpio_num = RECEIVER_GPIOS[i];
        rmt_channels[i].clk_div = RMT_RX_CLK_DIV;
        rmt_channels[i].mem_block_num = 1;
        rmt_channels[i].rmt_mode = RMT_MODE_RX;
        rmt_channels[i].rx_config.filter_en = true;
        rmt_channels[i].rx_config.filter_ticks_thresh = 100;
        rmt_channels[i].rx_config.idle_threshold = RMT_RX_MAX_US * RMT_TICK_PER_US;

        rmt_config(&rmt_channels[i]);
        rmt_driver_install(rmt_channels[i].channel, RMT_RB_SIZE, 0);
    }

    ESP_LOGI(TAG, "Init");
}
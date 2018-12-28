#include "listen_rx_task.h"

static const char* TAG = "RMT";

uint16_t ReceiverChannels[RECEIVER_CHANNELS_NUM] = {0};
const uint8_t RECEIVER_CHANNELS[RECEIVER_CHANNELS_NUM] = { 0, 1, 2 };
const uint8_t RECEIVER_GPIOS[RECEIVER_CHANNELS_NUM] = { 18, 17, 5 };

static void rmt_init(void) {
    uint8_t i;

    rmt_config_t rmt_channels[RECEIVER_CHANNELS_NUM] = {};

    for (i = 0; i < RECEIVER_CHANNELS_NUM; i++) {
        ReceiverChannels[i] = RECEIVER_CH_CENTER;

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

void listen_rx_task(void *pvParameter) {
    // this task listens to the 3 channels of the receiver and outputs to the global ReceiverChannels
    uint8_t i;

    rmt_init();
    RingbufHandle_t rb[RECEIVER_CHANNELS_NUM];
    
    for (i = 0; i < RECEIVER_CHANNELS_NUM; i++) {
        rmt_get_ringbuf_handle(RECEIVER_CHANNELS[i], &rb[i]);
        rmt_rx_start(RECEIVER_CHANNELS[i], 1);
    }

    while(1) {
        size_t rx_size = 0;

        for (i = 0; i < RECEIVER_CHANNELS_NUM; i++) {
            rmt_item32_t* item = (rmt_item32_t*) xRingbufferReceive(rb[i], &rx_size, 1000);
            if(item) {
                ReceiverChannels[i] = (uint16_t) item->duration0;
                vRingbufferReturnItem(rb[i], (void*) item);
            }
        }
        vTaskDelay(1);
    }
}
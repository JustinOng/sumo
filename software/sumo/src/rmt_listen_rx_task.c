#include "rmt_listen_rx_task.h"

static const char* TAG = "RMT";

void rmt_listen_rx_task(void *pvParameter) {
    // this task listens to the 3 channels of the receiver and outputs to the global struct receiver
    int channel = RECEIVER_CH1_CHANNEL;
    rmt_init_rx();
    RingbufHandle_t rb = NULL;
    
    rmt_get_ringbuf_handle(channel, &rb);
    rmt_rx_start(channel, 1);
    while(1) {
        size_t rx_size = 0;
        rmt_item32_t* item = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, 1000);
        if(item) {
            ReceiverChannels.ch1 = (uint16_t) item->duration0;
            vRingbufferReturnItem(rb, (void*) item);
        }
    }
}

void rmt_init_rx(void) {
    uint8_t i;

    rmt_config_t rmt_channels[RECEIVER_CHANNELS_NUM] = {
        {
            .channel = RECEIVER_CH1_CHANNEL,
            .gpio_num = RECEIVER_CH1_GPIO_NUM
        },
        {
            .channel = RECEIVER_CH2_CHANNEL,
            .gpio_num = RECEIVER_CH2_GPIO_NUM
        },
        {
            .channel = RECEIVER_CH3_CHANNEL,
            .gpio_num = RECEIVER_CH3_GPIO_NUM
        }
    };

    for (i = 0; i < RECEIVER_CHANNELS_NUM; i++) {
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
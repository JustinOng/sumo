#include "rmt_listen_rx_task.h"

static const char* TAG = "RMT";

void rmt_listen_rx_task(void *pvParameter)
{
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

void rmt_init_rx() {
    rmt_config_t rmt_rx;
    rmt_rx.channel = RECEIVER_CH1_CHANNEL;
    rmt_rx.gpio_num = RECEIVER_CH1_GPIO_NUM;
    rmt_rx.clk_div = RMT_RX_CLK_DIV;
    rmt_rx.mem_block_num = 1;
    rmt_rx.rmt_mode = RMT_MODE_RX;
    rmt_rx.rx_config.filter_en = true;
    rmt_rx.rx_config.filter_ticks_thresh = 100;
    rmt_rx.rx_config.idle_threshold = RMT_RX_MAX_US * RMT_TICK_PER_US;
    rmt_config(&rmt_rx);
    rmt_driver_install(rmt_rx.channel, 1000, 0);

    ESP_LOGI(TAG, "Init");
}
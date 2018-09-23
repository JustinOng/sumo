#ifndef RMT_LISTEN_RX_TASK
#define RMT_LISTEN_RX_TASK

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt.h"
#include "esp_log.h"

#define RMT_TICK_PER_US 8
// determines how many clock cycles one "tick" is
// [1..255], source is generally 80MHz APB clk
#define RMT_RX_CLK_DIV (80000000/RMT_TICK_PER_US/1000000)
// time before receiver goes idle
#define RMT_RX_MAX_US 3500
// size of ring buffer
#define RMT_RB_SIZE 1000

#define RECEIVER_CHANNELS_NUM 3

#define RECEIVER_CH1_CHANNEL 0
#define RECEIVER_CH1_GPIO_NUM 12

#define RECEIVER_CH2_CHANNEL 1
#define RECEIVER_CH2_GPIO_NUM 13

#define RECEIVER_CH3_CHANNEL 2
#define RECEIVER_CH3_GPIO_NUM 14

volatile struct {
    uint16_t ch1;
} ReceiverChannels;

void rmt_listen_rx_task(void *pvParameter);
void rmt_init_rx(void);

#endif
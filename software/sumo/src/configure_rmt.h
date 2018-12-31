#ifndef CONFIGURE_RMT
#define CONFIGURE_RMT

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt.h"
#include "esp_log.h"
#include "config.h"

#define RMT_TICK_PER_US 8
// determines how many clock cycles one "tick" is
// [1..255], source is generally 80MHz APB clk
#define RMT_RX_CLK_DIV (80000000/RMT_TICK_PER_US/1000000)
// time before receiver goes idle
#define RMT_RX_MAX_US 3500

#define RECEIVER_CHANNELS_NUM 3

extern volatile uint16_t ReceiverChannels[RECEIVER_CHANNELS_NUM];

void rmt_init(void);

#endif

#ifndef MOTOR_CONTROL_TASK
#define MOTOR_CONTROL_TASK

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#define MIN_MOTOR_SPEED 150
#define MAX_MOTOR_SPEED 40

#define MOTOR_CHANNELS_NUM 2

#define MOTOR_LEFT_CHANNEL LEDC_CHANNEL_0
#define MOTOR_LEFT_THROTTLE_NUM 27
#define MOTOR_LEFT_BRAKE_NUM 26
#define MOTOR_LEFT_DIR_NUM 14
#define MOTOR_LEFT_SPEED_NUM 25

#define MOTOR_RIGHT_CHANNEL LEDC_CHANNEL_1
#define MOTOR_RIGHT_THROTTLE_NUM 4
#define MOTOR_RIGHT_BRAKE_NUM 13
#define MOTOR_RIGHT_DIR_NUM 15
#define MOTOR_RIGHT_SPEED_NUM 12

#define LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_FREQUENCY 100000
#define LEDC_RESOLUTION LEDC_TIMER_10_BIT

typedef struct {
    uint8_t dir;
    uint16_t speed;
} MotorState;

extern volatile uint64_t last_pulse;
extern volatile uint64_t last_pulse_length;

void motor_control_task(void *pvParameter);
void set_motor_dir(uint8_t motor, uint8_t dir);
void set_motor_speed(uint8_t motor, uint16_t speed);

#endif

#ifndef LEDC_PWM_TASK
#define LEDC_PWM_TASK

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"

#define MOTOR_CHANNELS_NUM 1

#define MOTOR_LEFT_CHANNEL LEDC_CHANNEL_0
#define MOTOR_LEFT_GPIO_NUM 27

#define LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_FREQUENCY 1000
#define LEDC_RESOLUTION LEDC_TIMER_10_BIT

// only lower 10 bits is used because of 10 bit resolution of the module
extern uint16_t MotorControl[MOTOR_CHANNELS_NUM];

void ledc_pwm_task(void *pvParameter);
void ledc_init(void);

#endif
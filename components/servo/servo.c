#include "servo.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define SERVO_MODE LEDC_LOW_SPEED_MODE
#define PWM_FREQUENCY 50
#define PWM_RESOLUTION LEDC_TIMER_14_BIT

#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2500
#define SERVO_MAX_DEGREE 180

static int current_angle = 0;

esp_err_t servo_init(int gpio)
{
    // Configura timer
    ledc_timer_config_t timer_conf = {
        .speed_mode = SERVO_MODE,
        .timer_num = SERVO_TIMER,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_USE_APB_CLK}; // _Usa o mesmo clock dos motores por razões de simplicidade e sincronização
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    // Configura canal
    ledc_channel_config_t channel_conf = {
        .gpio_num = gpio,
        .speed_mode = SERVO_MODE,
        .channel = SERVO_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = SERVO_TIMER,
        .duty = 0,
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));

    // Vai imediatamente para posição “home”
    servo_set_angle(SERVO_HOME_ANGLE);
    printf("Servo init: GPIO=%d, Timer=%d, Canal=%d, Freq=%d\n", gpio, SERVO_TIMER, SERVO_CHANNEL, PWM_FREQUENCY);

    return ESP_OK;
}

static uint32_t angle_to_duty(int angle)
{
    uint32_t pulse_width = SERVO_MIN_PULSEWIDTH_US +
                           (angle * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / SERVO_MAX_DEGREE);

    uint32_t max_duty = (1 << PWM_RESOLUTION) - 1;
    uint32_t period_us = 1000000 / PWM_FREQUENCY;

    return (pulse_width * max_duty) / period_us;
}

esp_err_t servo_set_angle(int angle)
{
    if (angle < 0)
        angle = 0;
    if (angle > 180)
        angle = 180;

    current_angle = angle;
    uint32_t duty = angle_to_duty(angle);

    ESP_ERROR_CHECK(ledc_set_duty(SERVO_MODE, SERVO_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(SERVO_MODE, SERVO_CHANNEL));

    return ESP_OK;
}

int servo_get_angle(void)
{
    return current_angle;
}
// motors_control.c
#include <stdio.h>
#include "driver/gpio.h"  // GPIO_MODE_OUTPUT
#include "driver/ledc.h"  // LEDC PWM
#include "esp_rom_gpio.h" // esp_rom_gpio_pad_select_gpio()
#include "motors_control.h"
#include "tuning.h"

// ----- Motores -----
#define AIN1 17
#define AIN2 16
#define PWM1 15
#define BIN1 40
#define BIN2 41
#define PWM2 42

#define MOTOR_PWM_FREQ 20000 ///< Frequência PWM (Hz)
#define MOTOR_PWM_MODE LEDC_LOW_SPEED_MODE
#define MOTOR_PWM_TIMER LEDC_TIMER_0
#define MOTOR_PWM_RES LEDC_TIMER_10_BIT      ///< Resolução PWM (10-bit)
#define MAX_DUTY_CYCLE 1023                  ///< Ciclo máximo para 10-bit
#define MOTOR_PWM_CHANNEL_ESQ LEDC_CHANNEL_0 ///< Canal PWM do motor esquerdo
#define MOTOR_PWM_CHANNEL_DTA LEDC_CHANNEL_1 ///< Canal PWM do motor direito

void motors_init(void)
{
    // Motor A (esquerdo)
    esp_rom_gpio_pad_select_gpio(AIN1);
    gpio_set_direction(AIN1, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(AIN2);
    gpio_set_direction(AIN2, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(PWM1);
    gpio_set_direction(PWM1, GPIO_MODE_OUTPUT);

    // Motor B (direito)
    esp_rom_gpio_pad_select_gpio(BIN1);
    gpio_set_direction(BIN1, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(BIN2);
    gpio_set_direction(BIN2, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(PWM2);
    gpio_set_direction(PWM2, GPIO_MODE_OUTPUT);

    // -------------------------------
    // 2) Configuração do PWM (LEDC)
    // -------------------------------

    // Timer PWM
    ledc_timer_config_t pwm_timer = {
        .speed_mode = MOTOR_PWM_MODE,
        .duty_resolution = MOTOR_PWM_RES,
        .timer_num = MOTOR_PWM_TIMER,
        .freq_hz = MOTOR_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&pwm_timer);

    // Canal PWM motor esquerdo
    ledc_channel_config_t motorESQ = {
        .gpio_num = PWM1,
        .speed_mode = MOTOR_PWM_MODE,
        .channel = MOTOR_PWM_CHANNEL_ESQ,
        .timer_sel = MOTOR_PWM_TIMER,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&motorESQ);

    // Canal PWM motor direito
    ledc_channel_config_t motorDTA = {
        .gpio_num = PWM2,
        .speed_mode = MOTOR_PWM_MODE,
        .channel = MOTOR_PWM_CHANNEL_DTA,
        .timer_sel = MOTOR_PWM_TIMER,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&motorDTA);
}

static void motor_set(int pwm, int gpio_a, int gpio_b, ledc_channel_t channel)
{
    if (pwm == 0)
    {
        gpio_set_level(gpio_a, 0);
        gpio_set_level(gpio_b, 0);
        ledc_set_duty(MOTOR_PWM_MODE, channel, 0);
        ledc_update_duty(MOTOR_PWM_MODE, channel);
        return;
    }
    if (pwm > MAX_DUTY_CYCLE)
        pwm = MAX_DUTY_CYCLE;
    if (pwm < -MAX_DUTY_CYCLE)
        pwm = -MAX_DUTY_CYCLE;

    if (pwm > 0)
    {
        gpio_set_level(gpio_a, 1);
        gpio_set_level(gpio_b, 0);
    }
    else
    {
        gpio_set_level(gpio_a, 0);
        gpio_set_level(gpio_b, 1);
        pwm = -pwm;
    }
    ledc_set_duty(MOTOR_PWM_MODE, channel, pwm);
    ledc_update_duty(MOTOR_PWM_MODE, channel);
}

void motorControl(float line_position)
{
    float erro = 0.0f - line_position; // queremos linha centrada = 0
    float correcao = KP * erro;

    int left_pwm = BASE_SPEED + correcao;
    int right_pwm = BASE_SPEED - correcao;

    // Limitar o PWM para não ser negativo (coast) - não é necessário.
    if (left_pwm < 0)
        left_pwm = 0;
    if (right_pwm < 0)
        right_pwm = 0;

    motor_set(left_pwm, AIN1, AIN2, MOTOR_PWM_CHANNEL_ESQ);
    motor_set(right_pwm, BIN1, BIN2, MOTOR_PWM_CHANNEL_DTA);
}

void motors_set(int left_pwm, int right_pwm)
{
    motor_set(left_pwm, AIN1, AIN2, MOTOR_PWM_CHANNEL_ESQ);
    motor_set(right_pwm, BIN1, BIN2, MOTOR_PWM_CHANNEL_DTA);
}

void motors_stop(void)
{
    motors_set(0, 0);
}
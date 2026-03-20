// motors_control.c
#include "freertos/FreeRTOS.h" // ← vTaskDelay, pdMS_TO_TICKS
#include "driver/gpio.h"       // ← gpio_set_direction, gpio_set_level
#include "driver/ledc.h"       // ← ledc_set_duty, ledc_channel_config, etc.
#include "esp_rom_gpio.h"      // ← esp_rom_gpio_pad_select_gpio
#include "motors_control.h"    // ← próprio header
#include "tuning.h"            // ← tuning.KP, tuning.BASE_SPEED, MAX_DUTY_CYCLE

static int current_left_pwm = 0;
static int current_right_pwm = 0;

void motors_init(void)
{
    // Motor A (esquerdo)
    esp_rom_gpio_pad_select_gpio(PWM1);
    gpio_set_direction(PWM1, GPIO_MODE_OUTPUT);
    gpio_set_level(PWM1, 1);

    // Motor B (direito)
    esp_rom_gpio_pad_select_gpio(PWM2);
    gpio_set_direction(PWM2, GPIO_MODE_OUTPUT);
    gpio_set_level(PWM2, 1);

    // -------------------------------
    // 2) Configuração do PWM (LEDC)
    // -------------------------------

    // Timer PWM — deve ser configurado ANTES dos canais
    ledc_timer_config_t pwm_timer = {
        .speed_mode = MOTOR_PWM_MODE,     // low speed mode
        .duty_resolution = MOTOR_PWM_RES, // 10-bit: 0-1023
        .timer_num = MOTOR_PWM_TIMER,     // timer 0
        .freq_hz = MOTOR_PWM_FREQ,        // 20kHz
        .clk_cfg = LEDC_AUTO_CLK          // clock automático
    };
    ledc_timer_config(&pwm_timer);

    // Canal RPWM motor esquerdo — AIN2 (GPIO16)
    // RPWM=PWM → motor anda para a frente
    // RPWM=0   → motor parado (em conjunto com LPWM=0 → short brake)
    ledc_channel_config_t motorESQ = {
        .gpio_num = AIN2, // GPIO16 — RPWM esquerdo
        .speed_mode = MOTOR_PWM_MODE,
        .channel = MOTOR_PWM_CHANNEL_ESQ, // canal 0
        .timer_sel = MOTOR_PWM_TIMER,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&motorESQ);

    // Canal LPWM motor esquerdo — AIN1 (GPIO17)
    // LPWM=PWM → motor anda para trás / travagem regenerativa
    // LPWM=0   → sem travagem
    ledc_channel_config_t motorESQ_brake = {
        .gpio_num = AIN1, // GPIO17 — LPWM esquerdo
        .speed_mode = MOTOR_PWM_MODE,
        .channel = MOTOR_PWM_CHANNEL_ESQ_BRAKE, // canal 2
        .timer_sel = MOTOR_PWM_TIMER,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&motorESQ_brake);

    // Canal RPWM motor direito — BIN2 (GPIO41)
    // Equivalente ao AIN2 mas para o motor direito
    ledc_channel_config_t motorDTA = {
        .gpio_num = BIN2, // GPIO41 — RPWM direito
        .speed_mode = MOTOR_PWM_MODE,
        .channel = MOTOR_PWM_CHANNEL_DTA, // canal 1
        .timer_sel = MOTOR_PWM_TIMER,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&motorDTA);

    // Canal LPWM motor direito — BIN1 (GPIO40)
    // Equivalente ao AIN1 mas para o motor direito
    ledc_channel_config_t motorDTA_brake = {
        .gpio_num = BIN1, // GPIO40 — LPWM direito
        .speed_mode = MOTOR_PWM_MODE,
        .channel = MOTOR_PWM_CHANNEL_DTA_BRAKE, // canal 3
        .timer_sel = MOTOR_PWM_TIMER,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&motorDTA_brake);
}

static void motor_set(int pwm, int gpio_a, int gpio_b, ledc_channel_t channel, int *current_pwm)
{
    *current_pwm = pwm; // guarda PWM atual

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
    // Garante travagem desligada
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_ESQ_BRAKE, 0);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_ESQ_BRAKE);
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_DTA_BRAKE, 0);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_DTA_BRAKE);

    float erro = 0.0f - line_position; // queremos linha centrada = 0
    float correcao = tuning.KP * erro;

    int left_pwm = tuning.BASE_SPEED - correcao;
    int right_pwm = tuning.BASE_SPEED + correcao;

    // Limitar o PWM para não ser negativo (coast) - não é necessário.
    if (left_pwm < 0)
        left_pwm = 0;
    if (right_pwm < 0)
        right_pwm = 0;

    motor_set(left_pwm, AIN1, AIN2, MOTOR_PWM_CHANNEL_ESQ, &current_left_pwm);
    motor_set(right_pwm, BIN1, BIN2, MOTOR_PWM_CHANNEL_DTA, &current_right_pwm);
}

void motors_set(int left_pwm, int right_pwm)
{
    motor_set(left_pwm, AIN1, AIN2, MOTOR_PWM_CHANNEL_ESQ, &current_left_pwm);
    motor_set(right_pwm, BIN1, BIN2, MOTOR_PWM_CHANNEL_DTA, &current_right_pwm);
}

void motors_stop_fast(void)
{
    // Corta frente imediatamente
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_ESQ, 0);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_ESQ);
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_DTA, 0);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_DTA);

    gpio_set_level(AIN1, 1);
    gpio_set_level(BIN1, 1);

    // Rampa de inversão progressiva
    // Cada fase: 50ms, aumenta 10% por fase
    int steps[] = {102, 205, 307}; // 10%, 20%, 30% de 1023
    int num_steps = sizeof(steps) / sizeof(steps[0]);

    for (int i = 0; i < num_steps; i++)
    {
        ledc_set_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_ESQ_BRAKE, steps[i]);
        ledc_update_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_ESQ_BRAKE);
        ledc_set_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_DTA_BRAKE, steps[i]);
        ledc_update_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_DTA_BRAKE);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Coast final
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_ESQ_BRAKE, 0);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_ESQ_BRAKE);
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_DTA_BRAKE, 0);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_DTA_BRAKE);

    gpio_set_level(AIN1, 1);
    gpio_set_level(BIN1, 1);
}

void motors_coast(void)
{
    // Desliga PWM e deixa motores em coast
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_ESQ, 0);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_ESQ);
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_DTA, 0);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_DTA);

    gpio_set_level(AIN1, 0);
    gpio_set_level(BIN1, 0);
}
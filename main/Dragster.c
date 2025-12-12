#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "driver/ledc.h"  // For PWM control
#include "esp_rom_gpio.h" // Para esp_rom_gpio_pad_select_gpio()

// Handles do ADC
adc_oneshot_unit_handle_t adc1_handle;

// Motor Pin definitions
#define AIN1 13
#define BIN1 27
#define AIN2 12
#define BIN2 14
#define PWM1 25
#define PWM2 26

#define MOTOR_PWM_FREQ 20000 // Frequency in Hz for PWM
#define MOTOR_PWM_MODE LEDC_LOW_SPEED_MODE
#define MOTOR_PWM_TIMER LEDC_TIMER_0
#define MOTOR_PWM_RES LEDC_TIMER_10_BIT // PWM resolution (10-bit)
#define MAX_DUTY_CYCLE 1023             // Maximum duty cycle for 10-bit resolution

// Motor ESQ
#define MOTOR_PWM_CHANNEL_ESQ LEDC_CHANNEL_0

// Motor DTA
#define MOTOR_PWM_CHANNEL_DTA LEDC_CHANNEL_1

void controlTask(void *pvParameters);
void motorControl(float line_position);

void motor_left_set(int duty);
void motor_right_set(int duty);

float calculaPosicao(int s1, int s2, int s3, int s4); // declaração
float pos;
float KP = 0.5f;
int BASE_SPEED = 80;
const int LIMITE_PRETO = 3000; // depende da calibração

volatile float line_position = 0.0f;

typedef struct
{
    float s1, s2, s3, s4;
} SensorValues;
volatile SensorValues sensors;

volatile bool prova_terminada = false; // Flag de fim de prova

void app_main(void)
{
    // ================================
    // Configurar os GPIOs de direção como saída
    // ================================

    // Motor A direction
    esp_rom_gpio_pad_select_gpio(AIN1);
    gpio_set_direction(AIN1, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(AIN2);
    gpio_set_direction(AIN2, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(PWM1);
    gpio_set_direction(PWM1, GPIO_MODE_OUTPUT);

    // Motor B direction
    esp_rom_gpio_pad_select_gpio(BIN1);
    gpio_set_direction(BIN1, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(BIN2);
    gpio_set_direction(BIN2, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(PWM2);
    gpio_set_direction(PWM2, GPIO_MODE_OUTPUT);

    // ================================
    // Configurar PWM
    // ================================

    // Configure LEDC timer
    ledc_timer_config_t pwm_timer = {
        .speed_mode = MOTOR_PWM_MODE,
        .duty_resolution = MOTOR_PWM_RES,
        .timer_num = MOTOR_PWM_TIMER,
        .freq_hz = MOTOR_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&pwm_timer);

    // Configure Motor ESQuerdo
    ledc_channel_config_t motorESQ = {
        .gpio_num = PWM1,
        .speed_mode = MOTOR_PWM_MODE,
        .channel = MOTOR_PWM_CHANNEL_ESQ,
        .timer_sel = MOTOR_PWM_TIMER,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&motorESQ);

    // Configure Motor Direito
    ledc_channel_config_t motorDTA = {
        .gpio_num = PWM2,
        .speed_mode = MOTOR_PWM_MODE,
        .channel = MOTOR_PWM_CHANNEL_DTA,
        .timer_sel = MOTOR_PWM_TIMER,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&motorDTA);

    // ================================
    // 1) Inicializar o ADC1
    // ================================
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1};
    adc_oneshot_new_unit(&init_config, &adc1_handle);

    // ================================
    // 2) Configurar cada canal (sensor)
    // ================================
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, // Normalmente 12 bits
        .atten = ADC_ATTEN_DB_12          // Para ler até ~3.3V
    };
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config); // Sensor esq - verde/laranja
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_7, &config); // Sensor central esq branco/cinzento
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &config); // Sensor central dir vermelho/castanho
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_5, &config); // Sensor dir verde/amarelo

    // ================================
    // 3) Criar a tarefa que lê os sensores
    // ================================
    xTaskCreate(controlTask, "Control Task", 2048, NULL, 1, NULL);
}

// ================================
// TAREFA QUE LÊ OS 3 SENSORES
// ================================
void controlTask(void *pvParameters)
{
    int s1, s2, s3, s4;

    while (1)
    {
        // Ler valor de cada canal
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &s1);
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &s2);
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &s3);
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &s4);

        line_position = calculaPosicao(s1, s2, s3, s4);

        // Imprimir resultados
        printf("pos:  %f S1: %d  S2: %d S3: %d   S4: %d \n", line_position, s1, s2, s3, s4);

        if (s1 > LIMITE_PRETO && s2 > LIMITE_PRETO)
        {
            // Para os motores
            motor_left_set(0);
            motor_right_set(0);

            printf("Prova terminada!\n");

            // Loop seguro: a task fica “congelada” e não consome CPU
            //while (1)
            //    vTaskDelay(pdMS_TO_TICKS(1000));
        }

        motorControl(line_position);

        // Esperar 100 ms antes da próxima leitura
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ================================
// TAREFA QUE controla os motores
// ================================

void motorControl(float line_position)
{
    float erro = 0.0f - line_position; // queremos linha centrada = 0
    float correcao = KP * erro;

    int left_pwm = BASE_SPEED + correcao;
    int right_pwm = BASE_SPEED - correcao;

    motor_left_set(left_pwm);
    motor_right_set(right_pwm);
}

// ================================
// TAREFA QUE pára os motores
// ================================

void motor_left_set(int pwm)
{

    // STOP total → coast
    if (pwm == 0)
    {
        gpio_set_level(AIN1, 0);
        gpio_set_level(AIN2, 0);
        ledc_set_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_ESQ, pwm);
        ledc_update_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_ESQ);
        return;
    }

    // Saturar
    if (pwm > MAX_DUTY_CYCLE)
        pwm = MAX_DUTY_CYCLE;
    if (pwm < -MAX_DUTY_CYCLE)
        pwm = -MAX_DUTY_CYCLE;

    if (pwm > 0)
    {
        gpio_set_level(AIN1, 1);
        gpio_set_level(AIN2, 0);
        ledc_set_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_ESQ, pwm);
    }
    else
    {
        gpio_set_level(AIN1, 0);
        gpio_set_level(AIN2, 1);
        ledc_set_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_ESQ, -pwm);
    }

    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_ESQ);
}

void motor_right_set(int pwm)
{
    // STOP total → coast
    if (pwm == 0)
    {
        gpio_set_level(AIN1, 0);
        gpio_set_level(AIN2, 0);
        ledc_set_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_DTA, pwm);
        ledc_update_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_DTA);
        return;
    }

    // Saturar
    if (pwm > MAX_DUTY_CYCLE)
        pwm = MAX_DUTY_CYCLE;
    if (pwm < -MAX_DUTY_CYCLE)
        pwm = -MAX_DUTY_CYCLE;

    if (pwm > 0)
    {
        gpio_set_level(BIN1, 1);
        gpio_set_level(BIN2, 0);
        ledc_set_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_DTA, pwm);
    }
    else
    {
        gpio_set_level(BIN1, 0);
        gpio_set_level(BIN2, 1);
        ledc_set_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_DTA, -pwm);
    }

    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL_DTA);
}

float calculaPosicao(int s1, int s2, int s3, int s4)
{
    int soma = s1 + s2 + s3 + s4;
    if (soma < 50)
        return 0; // linha perdida

    float pos = (s1 * -1.0f + s2 * -0.33f + s3 * 0.33f + s4 * 1.0f) / soma;
    return pos;
}
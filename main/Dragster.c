#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "driver/ledc.h"  // For PWM control
#include "hal/ledc_types.h" // Adicione esta linha
#include "esp_rom_gpio.h" // Para esp_rom_gpio_pad_select_gpio()
#include "blinkt.h"

// ===============================
// Definições copiadas do ficheiro de calibração
// ===============================


// Handles do ADC
adc_oneshot_unit_handle_t adc1_handle;

// Motor Pin definitions
#define AIN1 12
#define BIN1 14
#define AIN2 13
#define BIN2 27
#define PWM1 25
#define PWM2 26

#define BTN_CAL XX // Botão de calibração

#define MOTOR_PWM_FREQ 20000 // Frequency in Hz for PWM
#define MOTOR_PWM_MODE LEDC_LOW_SPEED_MODE
#define MOTOR_PWM_TIMER LEDC_TIMER_0
#define MOTOR_PWM_RES LEDC_TIMER_10_BIT // PWM resolution (10-bit)
#define MAX_DUTY_CYCLE 1023             // Maximum duty cycle for 10-bit resolution

// Canal ADC dos sensores
#define S1_CHANNEL ADC_CHANNEL_6 //Amarelo,  S1 - GPIO34 -> Esquerda
#define S2_CHANNEL ADC_CHANNEL_7 //Castanho, S2 - GPIO35
#define S3_CHANNEL ADC_CHANNEL_3 //Cinzento, S3 - GPIO39
#define S4_CHANNEL ADC_CHANNEL_5 //Castanho, S4 - GPIO33 -> Direita

// Motor ESQ
#define MOTOR_PWM_CHANNEL_ESQ LEDC_CHANNEL_0

// Motor DTA
#define MOTOR_PWM_CHANNEL_DTA LEDC_CHANNEL_1

void controlTask(void *pvParameters);
void motorControl(float line_position);
float normalize(int raw, int min, int max);

void motor_left_set(int duty);
void motor_right_set(int duty);

float calculaPosicao(float s1_norm, float s2_norm, float s3_norm, float s4_norm); // declaração
float pos;
float KP = 450.0f;
int BASE_SPEED = 1500;
const int LIMITE_PRETO = 3000; // depende da calibração

volatile float line_position = 0.0f;
#define linelost_threshold 0.05f  // linha perdida se soma dos sensores normalizados < 0.05
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
    adc_oneshot_config_channel(adc1_handle, S1_CHANNEL, &config); // Sensor esq - verde/laranja
    adc_oneshot_config_channel(adc1_handle, S2_CHANNEL, &config); // Sensor central esq branco/cinzento
    adc_oneshot_config_channel(adc1_handle, S3_CHANNEL, &config); // Sensor central dir vermelho/castanho
    adc_oneshot_config_channel(adc1_handle, S4_CHANNEL, &config); // Sensor dir verde/amarelo
    
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
    float s1_norm, s2_norm, s3_norm, s4_norm; // Valores normalizados entre 0.0 e 1.0
    // Valores da calibração

    #define S1_MIN 94
    #define S1_MAX 3046

    #define S2_MIN 101
    #define S2_MAX 3056

    #define S3_MIN 125
    #define S3_MAX 3250

    #define S4_MIN 130
    #define S4_MAX 3287

    // Espera botão de calibração
    wait_for_calibration_button();

     // Calibra
    qtr_calibrate();

    while (1)
    {
        // Ler valor de cada canal
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &s1);
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &s2);
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_3, &s3);
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &s4);

        s1_norm = normalize(s1, S1_MIN, S1_MAX);
        s2_norm = normalize(s2, S2_MIN, S2_MAX);
        s3_norm = normalize(s3, S3_MIN, S3_MAX);
        s4_norm = normalize(s4, S4_MIN, S4_MAX);

        line_position = calculaPosicao(s1_norm, s2_norm, s3_norm, s4_norm);

        // Imprimir resultados
        printf("pos:  %f S1: %.2f  S2: %.2f S3: %.2f   S4: %.2f \n", line_position, s1_norm, s2_norm, s3_norm, s4_norm);

        if (s1 > LIMITE_PRETO && s4 > LIMITE_PRETO)
        {
            // Para os motores
            motor_left_set(0);
            motor_right_set(0);

            printf("Prova terminada!\n");

            // Loop seguro: a task fica “congelada” e não consome CPU
            while (1)
                vTaskDelay(pdMS_TO_TICKS(1000));
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

float calculaPosicao(float s1, float s2, float s3, float s4)
{
    float soma = s1 + s2 + s3 + s4;
    if (soma < linelost_threshold)
        return 0; // linha perdida

    float pos = (s1 * -1.0f + s2 * -0.33f + s3 * 0.33f + s4 * 1.0f) / soma;
    return pos;
}

float normalize(int raw, int min, int max)
{
    // Limita o valor ao intervalo calibrado - devolve um valor entre 0.0 e 1.0
    if (raw < min) raw = min;
    if (raw > max) raw = max;

    // Normaliza para 0.0–1.0
    return (float)(raw - min) / (float)(max - min);
}

void qtr_calibrate(void)
{

     // Inicializa Blinkt!
    blinkt_init();      // Inicializa os pinos
    blinkt_white();     // Acende todos os LEDs a branco
 
    // Variáveis
    int s1_raw, s2_raw, s3_raw, s4_raw;
    int min_s1 = 4095, max_s1 = 0;
    int min_s2 = 4095, max_s2 = 0;
    int min_s3 = 4095, max_s3 = 0;
    int min_s4 = 4095, max_s4 = 0;

    printf("=== CALIBRACAO SENSORES ===\n");
    printf("Move o Dragster lentamente sobre a linha e o fundo...\n");

    uint32_t inicio = xTaskGetTickCount();
    const int DURACAO_CALIB_MS = 10000; // 3 segundos

    // -----------------------
    // CICLO DE CALIBRAÇÃO
    // -----------------------
    while (xTaskGetTickCount() - inicio < pdMS_TO_TICKS(DURACAO_CALIB_MS))
    {
        adc_oneshot_read(adc1_handle, S1_CHANNEL, &s1_raw);
        adc_oneshot_read(adc1_handle, S2_CHANNEL, &s2_raw);
        adc_oneshot_read(adc1_handle, S3_CHANNEL, &s3_raw);
        adc_oneshot_read(adc1_handle, S4_CHANNEL, &s4_raw);

        if (s1_raw < min_s1)
            min_s1 = s1_raw;
        if (s1_raw > max_s1)
            max_s1 = s1_raw;

        if (s2_raw < min_s2)
            min_s2 = s2_raw;
        if (s2_raw > max_s2)
            max_s2 = s2_raw;

        if (s3_raw < min_s3)
            min_s3 = s3_raw;
        if (s3_raw > max_s3)
            max_s3 = s3_raw;

        if (s4_raw < min_s4)
            min_s4 = s4_raw;
        if (s4_raw > max_s4)
            max_s4 = s4_raw;

        printf("S1 raw=%4d min=%4d max=%4d | S2 raw=%4d min=%4d max=%4d\n",
               s1_raw, min_s1, max_s1, s2_raw, min_s2, max_s2);
        printf("S3 raw=%4d min=%4d max=%4d | S4 raw=%4d min=%4d max=%4d\n\n",
               s3_raw, min_s3, max_s3, s4_raw, min_s4, max_s4);

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // -----------------------
    // RESULTADOS FINAIS
    // -----------------------
    printf("\n=== RESULTADOS FINAIS DA CALIBRACAO ===\n");
    printf("S1: min=%4d  max=%4d\n", min_s1, max_s1);
    printf("S2: min=%4d  max=%4d\n", min_s2, max_s2);
    printf("S3: min=%4d  max=%4d\n", min_s3, max_s3);
    printf("S4: min=%4d  max=%4d\n", min_s4, max_s4);
    printf("=======================================\n");

    printf("Calibração concluída! Reinicia para correr o programa.\n");

    // Impede que continue (opcional)
    while (1)
        vTaskDelay(pdMS_TO_TICKS(1000));
}

// -----------------------
// Botão de calibração
// -----------------------
static void wait_for_calibration_button(void)
{
    while (gpio_get_level(BTN_CAL) == 1) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

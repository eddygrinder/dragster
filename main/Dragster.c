// ===============================
// Includes principais
// ===============================
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "driver/ledc.h"    // PWM control
#include "hal/ledc_types.h" // Tipos LEDC
#include "esp_rom_gpio.h"   // esp_rom_gpio_pad_select_gpio()
#include "blinkt.h"

// ===============================
// Definições de hardware
// ===============================

// ----- Sensores QTR-8C -----
#define S1_CHANNEL ADC_CHANNEL_6 ///< Sensor esquerdo (Amarelo, GPIO34)
#define S2_CHANNEL ADC_CHANNEL_7 ///< Sensor central esquerdo (Castanho, GPIO35)
#define S3_CHANNEL ADC_CHANNEL_3 ///< Sensor central direito (Cinzento, GPIO39)
#define S4_CHANNEL ADC_CHANNEL_5 ///< Sensor direito (Castanho, GPIO33)

adc_oneshot_unit_handle_t adc1_handle; ///< Handle do ADC

// ----- Motores -----
#define AIN1 12
#define AIN2 13
#define PWM1 25
#define BIN1 14
#define BIN2 27
#define PWM2 26

#define MOTOR_PWM_FREQ 20000 ///< Frequência PWM (Hz)
#define MOTOR_PWM_MODE LEDC_LOW_SPEED_MODE
#define MOTOR_PWM_TIMER LEDC_TIMER_0
#define MOTOR_PWM_RES LEDC_TIMER_10_BIT      ///< Resolução PWM (10-bit)
#define MAX_DUTY_CYCLE 1023                  ///< Ciclo máximo para 10-bit
#define MOTOR_PWM_CHANNEL_ESQ LEDC_CHANNEL_0 ///< Canal PWM do motor esquerdo
#define MOTOR_PWM_CHANNEL_DTA LEDC_CHANNEL_1 ///< Canal PWM do motor direito

// ----- Botões -----
#define BTN_CAL 22 ///< GPIO do botão de calibração (substituir XX pelo GPIO real)

// ===============================
// Estruturas de dados
// ===============================
typedef struct
{
    float s1, s2, s3, s4; ///< Valores normalizados dos sensores
} SensorValues;

volatile SensorValues sensors;         ///< Valores atuais dos sensores


// ===============================
// Protótipos de funções
// ===============================

/// @brief Task principal de controlo do robô
void controlTask(void *pvParameters);
void run_line_follower(int s1_min, int s1_max, int s2_min, int s2_max, int s3_min, int s3_max, int s4_min, int s4_max); // loop contínuo


/**
 * @brief Ajusta os motores esquerdo e direito com base na posição da linha.
 *
 * @param line_position Posição calculada da linha (-1.0 à esquerda, +1.0 à direita)
 */
void motorControl(float line_position);
void motor_left_set(int duty);
void motor_right_set(int duty);

/// @brief Funções auxiliares
float normalize(int raw, int min, int max);
float calculaPosicao(float s1_norm, float s2_norm, float s3_norm, float s4_norm);
void qtr_calibrate(int *s1_min, int *s1_max, int *s2_min, int *s2_max, int *s3_min, int *s3_max, int *s4_min, int *s4_max, uint32_t duration_ms);
bool calib_button_pressed(void);
bool start_led_detected(void);

/// @brief Estados possíveis do robô durante a prova
///
/// O robô passa por estes estados em sequência:
/// 1. Espera pelo botão de calibração
/// 2. Executa a calibração dos sensores
/// 3. Espera pelo sinal de arranque (LED)
/// 4. Executa o seguimento da linha
typedef enum
{
    WAIT_CALIB,  ///< Estado inicial: espera que o botão de calibração seja pressionado
    CALIBRATING, ///< Estado de calibração: calibra sensores QTR-8C, acontece apenas uma vez
    WAIT_START,  ///< Estado de espera pelo LED de arranque
    RUN          ///< Estado de execução: segue a linha utilizando os valores calibrados
} robot_state_t;

// ===============================
// Função principal
// ===============================

/**
 * @brief Inicializa hardware, ADC, PWM e cria a task de controlo
 */
void app_main(void)
{
    // -------------------------------
    // 1) Configuração dos GPIOs de direção
    // -------------------------------

    esp_rom_gpio_pad_select_gpio(BTN_CAL);
    gpio_set_direction(BTN_CAL, GPIO_MODE_INPUT);

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

    // -------------------------------
    // 3) Inicializar ADC
    // -------------------------------
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1};
    adc_oneshot_new_unit(&init_config, &adc1_handle);

    // -------------------------------
    // 4) Configurar cada canal ADC (sensores)
    // -------------------------------
    adc_oneshot_chan_cfg_t adc_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, // normalmente 12 bits
        .atten = ADC_ATTEN_DB_12          // para ler até ~3.3V
    };

    adc_oneshot_config_channel(adc1_handle, S1_CHANNEL, &adc_config);
    adc_oneshot_config_channel(adc1_handle, S2_CHANNEL, &adc_config);
    adc_oneshot_config_channel(adc1_handle, S3_CHANNEL, &adc_config);
    adc_oneshot_config_channel(adc1_handle, S4_CHANNEL, &adc_config);

    // -------------------------------
    // 5) Criar a task principal de controlo
    // -------------------------------
    xTaskCreate(
        controlTask,    ///< Função da task
        "Control Task", ///< Nome da task
        2048,           ///< Stack size
        NULL,           ///< Parâmetros
        1,              ///< Prioridade
        NULL            ///< Handle da task (não usado)
    );
}

// ================================
// TAREFA QUE LÊ OS 3 SENSORES
// ================================
void controlTask(void *pvParameters)
{
    
    uint32_t duration_ms = 5000; ///< Duração da calibração em milissegundos
    int s1_min = 4095, s1_max = 0;
    int s2_min = 4095, s2_max = 0;
    int s3_min = 4095, s3_max = 0;
    int s4_min = 4095, s4_max = 0;

    /// @brief Espera até que o botão de calibração seja pressionado
    /*
    Liga robô
        ↓
    WAIT_CALIB   → botão pressionado
        ↓
    CALIBRATING  → termina
        ↓
    WAIT_START   → LED OFF
        ↓
    WAIT_START   → LED ON  ← AQUI
        ↓
    RUN          → segue linha até ao fim

    */

    robot_state_t state = WAIT_CALIB;

    // Assegura motores parados no início
    motor_left_set(0);
    motor_right_set(0);

    while (1)
    {
        switch (state)
        {
        case WAIT_CALIB:
            motor_left_set(0);
            motor_right_set(0);
            if (calib_button_pressed())
            {
                state = CALIBRATING;
            }
            break;

        case CALIBRATING:
            motor_left_set(0);
            motor_right_set(0);
            qtr_calibrate(&s1_min, &s1_max, &s2_min, &s2_max, &s3_min, &s3_max, &s4_min, &s4_max, duration_ms); // corre UMA VEZ
            state = WAIT_START;
            break;

        case WAIT_START:
            motor_left_set(0);
            motor_right_set(0);
            if (start_led_detected())
            {
                state = RUN;
            }
            break;

        case RUN:
            run_line_follower(s1_min, s1_max, s2_min, s2_max, s3_min, s3_max, s4_min, s4_max); // loop contínuo
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void run_line_follower(int s1_min, int s1_max, int s2_min, int s2_max, int s3_min, int s3_max, int s4_min, int s4_max)
{
    int s1, s2, s3, s4;
    const int LIMITE_PRETO = 3000; ///< Limite de deteção de preto (deve vir da calibração)
    float line_position = 0.0f;   ///< Posição da linha calculada
    float s1_norm, s2_norm, s3_norm, s4_norm; // Valores normalizados entre 0.0 e 1.0
    while (1)
    {
        // Ler valor de cada canal
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &s1);
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &s2);
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_3, &s3);
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &s4);

        s1_norm = normalize(s1, s1_min, s1_max);
        s2_norm = normalize(s2, s2_min, s2_max);
        s3_norm = normalize(s3, s3_min, s3_max);
        s4_norm = normalize(s4, s4_min, s4_max);

        line_position = calculaPosicao(s1_norm, s2_norm, s3_norm, s4_norm);
       
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

void motorControl(float line_position)
{
    
    float KP = 450.0f;             ///< Ganho proporcional
    int BASE_SPEED = 1500;         ///< Velocidade base dos motores
    float erro = 0.0f - line_position; // queremos linha centrada = 0
    float correcao = KP * erro;

    int left_pwm = BASE_SPEED + correcao;
    int right_pwm = BASE_SPEED - correcao;

    motor_left_set(left_pwm);
    motor_right_set(right_pwm);
}

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
        gpio_set_level(BIN1, 0);
        gpio_set_level(BIN2, 0);
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
    #define linelost_threshold 0.05f ///< Linha considerada perdida se soma dos sensores normalizados < 0.05
    float soma = s1 + s2 + s3 + s4;
    if (soma < linelost_threshold)
        return 0; // linha perdida

    float pos = (s1 * -1.0f + s2 * -0.33f + s3 * 0.33f + s4 * 1.0f) / soma;
    return pos;
}

float normalize(int raw, int min, int max)
{
    // Limita o valor ao intervalo calibrado - devolve um valor entre 0.0 e 1.0
    if (raw < min)
        raw = min;
    if (raw > max)
        raw = max;

    // Normaliza para 0.0–1.0
    return (float)(raw - min) / (float)(max - min);
}

void qtr_calibrate(int *s1_min, int *s1_max, int *s2_min, int *s2_max, int *s3_min, int *s3_max, int *s4_min, int *s4_max, uint32_t duration_ms)
{

    int s1_raw, s2_raw, s3_raw, s4_raw;
    // Inicializa Blinkt!
    blinkt_init();  // Inicializa os pinos
    blinkt_white(); // Acende todos os LEDs a branco

    printf("=== CALIBRACAO SENSORES ===\n");
    printf("Move o Dragster lentamente sobre a linha e o fundo...\n");

    uint32_t inicio = xTaskGetTickCount();

    // -----------------------
    // CICLO DE CALIBRAÇÃO
    // -----------------------
    while (xTaskGetTickCount() - inicio < pdMS_TO_TICKS(duration_ms))
    {
        adc_oneshot_read(adc1_handle, S1_CHANNEL, &s1_raw);
        adc_oneshot_read(adc1_handle, S2_CHANNEL, &s2_raw);
        adc_oneshot_read(adc1_handle, S3_CHANNEL, &s3_raw);
        adc_oneshot_read(adc1_handle, S4_CHANNEL, &s4_raw);

        if (s1_raw < *s1_min)
            *s1_min = s1_raw;
        if (s1_raw > *s1_max)
            *s1_max = s1_raw;

        if (s2_raw < *s2_min)
            *s2_min = s2_raw;
        if (s2_raw > *s2_max)
            *s2_max = s2_raw;

        if (s3_raw < *s3_min)
            *s3_min = s3_raw;
        if (s3_raw > *s3_max)
            *s3_max = s3_raw;

        if (s4_raw < *s4_min)
            *s4_min = s4_raw;
        if (s4_raw > *s4_max)
            *s4_max = s4_raw;

        printf("S1 raw=%4d min=%4d max=%4d | S2 raw=%4d min=%4d max=%4d\n",
               s1_raw, *s1_min, *s1_max, s2_raw, *s2_min, *s2_max);
        printf("S3 raw=%4d min=%4d max=%4d | S4 raw=%4d min=%4d max=%4d\n\n",
               s3_raw, *s3_min, *s3_max, s4_raw, *s4_min, *s4_max);

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // -----------------------
    // RESULTADOS FINAIS
    // -----------------------
    printf("\n=== RESULTADOS FINAIS DA CALIBRACAO ===\n");
    printf("S1: min=%4d  max=%4d\n", *s1_min, *s1_max);
    printf("S2: min=%4d  max=%4d\n", *s2_min, *s2_max);
    printf("S3: min=%4d  max=%4d\n", *s3_min, *s3_max);
    printf("S4: min=%4d  max=%4d\n", *s4_min, *s4_max);
    printf("=======================================\n");

    printf("Calibração concluída! Reinicia para correr o programa.\n");

    // Impede que continue (opcional)
    while (1)
        vTaskDelay(pdMS_TO_TICKS(1000));
}

/**
 * @brief Verifica se o botão de calibração foi pressionado
 *
 * @return true se pressionado, false caso contrário
 */
bool calib_button_pressed(void)
{
    return (gpio_get_level(BTN_CAL) == 1); // Retorna true quando pressionado
}

bool start_led_detected(void)
{
    return false; // ou true, conforme precisares nos testes
}
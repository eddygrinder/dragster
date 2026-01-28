// ===============================
// Includes principais
// ===============================
#include <stdio.h>
#include "tuning.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "driver/ledc.h"    // PWM control
#include "hal/ledc_types.h" // Tipos LEDC
#include "esp_rom_gpio.h"   // esp_rom_gpio_pad_select_gpio()
#include "blinkt.h"

#include "nvs_flash.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "host/ble_gap.h"

#include "led_strip.h"
#include "driver/rmt_tx.h"
#define RGB_LED_GPIO 38
static led_strip_handle_t led;

// Tag para logs

static const char *TAG = "NIMBLE_SCAN";

static uint8_t own_addr_type;

/* Converte endereço BLE em string XX:XX:XX:XX:XX:XX */
static void addr_to_str(const ble_addr_t *addr, char *str, size_t size)
{
    snprintf(str, size,
             "%02X:%02X:%02X:%02X:%02X:%02X",
             addr->val[5], addr->val[4], addr->val[3],
             addr->val[2], addr->val[1], addr->val[0]);
}

/* Callback de anúncio recebido */
static int
gap_event_cb(struct ble_gap_event *event, void *arg)
{
    if (event->type == BLE_GAP_EVENT_DISC)
    {
        const struct ble_gap_disc_desc *d = &event->disc;

        char addr_str[18];
        addr_to_str(&d->addr, addr_str, sizeof(addr_str));

        /* Tenta extrair nome do advertising */
        struct ble_hs_adv_fields fields;
        int rc = ble_hs_adv_parse_fields(&fields, d->data, d->length_data);
        char name[64] = "Unknown";

        if (rc == 0)
        {
            if (fields.name != NULL && fields.name_len > 0)
            {
                int len = fields.name_len < (int)sizeof(name) - 1 ? fields.name_len : (int)sizeof(name) - 1;
                memcpy(name, fields.name, len);
                name[len] = '\0';
            }
        }

        ESP_LOGI(TAG, "Device: %s | MAC: %s | RSSI: %d dBm",
                 name, addr_str, d->rssi);
    }

    return 0;
}

/* Inicia scan contínuo */
static void
start_scan(void)
{
    struct ble_gap_disc_params disc_params;

    memset(&disc_params, 0, sizeof(disc_params));

    disc_params.itvl = 0x50;       // intervalo de scan
    disc_params.window = 0x30;     // janela de scan
    disc_params.passive = 0;       // 0 = active scan
    disc_params.limited = 0;       // general discovery
    disc_params.filter_policy = 0; // BLE_HCI_SCAN_FILT_NO_WL

    /* Algumas versões não têm filter_dup/filter_dups – se não existir, ignora.
    disc_params.filter_dup = 0; */

    int rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER,
                          &disc_params, gap_event_cb, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Error starting discovery; rc=%d", rc);
    }
    else
    {
        ESP_LOGI(TAG, "Scanning for BLE devices...");
    }
}

/* Callback NimBLE quando o host está pronto */
static void ble_on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Address infer failed; rc=%d", rc);
        return;
    }

    // Nome do dispositivo
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    const char *name = "DRAGSTER";
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    ble_gap_adv_set_fields(&fields);

    // Parâmetros de advertising
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    ESP_LOGI(TAG, "Starting advertising...");
    ble_gap_adv_start(
        own_addr_type,
        NULL,
        BLE_HS_FOREVER,
        &adv_params,
        NULL,
        NULL);
}

/* Task principal NimBLE (corre o host) */
static void
host_task(void *param)
{
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run(); // Nunca retorna
    nimble_port_freertos_deinit();
}

/* Inicialização NimBLE + FreeRTOS */
static void ble_init(void)
{
    // NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // A partir do IDF 5.x, o controller + HCI é feito dentro do nimble_port_init()
    nimble_port_init(); // se isto falhar, retorna assert/log

    // Configurar callbacks NimBLE host
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.reset_cb = NULL;

    // Criar task FreeRTOS para o host NimBLE
    nimble_port_freertos_init(host_task);
}

// ==============================

// ------------------------------
// PID / control parameters
// ------------------------------
float KP = 120.0f; // variável atualizável via BLE

// ===============================
// Definições de hardware
// ===============================

// ----- Sensores QTR-8C -----
#define S1_CHANNEL ADC_CHANNEL_3 ///< Sensor esquerdo (Amarelo, GPIO34)
#define S2_CHANNEL ADC_CHANNEL_4 ///< Sensor central esquerdo (Castanho, GPIO35)
#define S3_CHANNEL ADC_CHANNEL_5 ///< Sensor central direito (Cinzento, GPIO39)
#define S4_CHANNEL ADC_CHANNEL_6 ///< Sensor direito (Castanho, GPIO33)

adc_oneshot_unit_handle_t adc1_handle; ///< Handle do ADC

bool calib_done = false; // Flag calibração

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

// ----- Botões -----
#define BTN_CAL 39 ///< GPIO do botão de calibração (substituir XX pelo GPIO real)

// ===============================
// Estruturas de dados
// ===============================
typedef struct
{
    float s1, s2, s3, s4; ///< Valores normalizados dos sensores
} SensorValues;

volatile SensorValues sensors; ///< Valores atuais dos sensores

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
void rgb_off(void);
void rgb_on(void);
void rgb_init(void);

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

    // Inicializar RGB interno
    rgb_init();
    // Inicializar Bluetooth
    ble_init();

    // -------------------------------
    // 1) Configuração dos GPIOs de direção
    // -------------------------------

    esp_rom_gpio_pad_select_gpio(BTN_CAL);
    gpio_set_direction(BTN_CAL, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BTN_CAL, GPIO_PULLDOWN_ONLY);

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
        4096,           ///< Stack size
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

    uint32_t duration_ms = CALIBRATION_TIME_MS; ///< Duração da calibração em milissegundos
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
            if (calib_done)
            {
                // Se já calibrado, pula direto para start
                state = WAIT_START;
            }
            else if (calib_button_pressed())
            {
                state = CALIBRATING;
            }
            break;

        case CALIBRATING:
            motor_left_set(0);
            motor_right_set(0);
            qtr_calibrate(&s1_min, &s1_max, &s2_min, &s2_max, &s3_min, &s3_max, &s4_min, &s4_max, duration_ms); // corre UMA VEZ

            calib_done = true; // Marca calibração feita
            state = WAIT_START;
            break;

        case WAIT_START:
            motor_left_set(0);
            motor_right_set(0);
            if (start_led_detected())
            {
                vTaskDelay(pdMS_TO_TICKS(2000)); // Pequena espera antes de arrancar
                state = RUN;
            }
            break;

        case RUN:
            run_line_follower(s1_min, s1_max, s2_min, s2_max, s3_min, s3_max, s4_min, s4_max); // loop contínuo
            state = WAIT_CALIB;                                                                // reinicia ciclo após prova
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void run_line_follower(int s1_min, int s1_max, int s2_min, int s2_max, int s3_min, int s3_max, int s4_min, int s4_max)
{
    int s1, s2, s3, s4;
    float line_position = 0.0f;               ///< Posição da linha calculada
    float s1_norm, s2_norm, s3_norm, s4_norm; // Valores normalizados entre 0.0 e 1.0

    // Depois do loop de calibração, ao calcular resultados finais
    int LIMITE_PRETO_S1 = s1_min + (s1_max - s1_min) * PRETO_PERCENT / 100;
    int LIMITE_PRETO_S4 = s4_min + ((s4_max - s4_min) * PRETO_PERCENT / 100);

    // printf("LIMITE_PRETO S1 = %d, S4 = %d\n", LIMITE_PRETO_S1, LIMITE_PRETO_S4);

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
        motorControl(line_position);

        if (s1 > LIMITE_PRETO_S1 && s4 > LIMITE_PRETO_S4)
        {
            // Para os motores
            motor_left_set(0);
            motor_right_set(0);

            printf("Prova terminada!\n");
        }

        // Esperar 1 ms antes da próxima leitura
        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }
}

void motorControl(float line_position)
{

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
    float soma = s1 + s2 + s3 + s4;
    if (soma < LINELOST_THRESHOLD)
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

    rgb_on(); // Acende LED de calibração

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
    rgb_off(); // Apaga LED de calibração

    // Impede que continue (opcional)
    // while (1)
    //    vTaskDelay(pdMS_TO_TICKS(1000));
}

/**
 * @brief Verifica se o botão de calibração foi pressionado
 *
 * @return true se pressionado, false caso contrário
 */
bool calib_button_pressed(void)
{
    printf("gpio_get_level(BTN_CAL) = %d\n", gpio_get_level(BTN_CAL));
    return (gpio_get_level(BTN_CAL) == 1); // Retorna true quando pressionado
}

bool start_led_detected(void)
{
    return true; // ou true, conforme precisares nos testes
}

void rgb_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_LED_GPIO,
        .max_leds = 1,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
    };

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
    };

    led_strip_new_rmt_device(&strip_config, &rmt_config, &led);
    led_strip_clear(led); // começa desligado
}

void rgb_on(void)
{
    led_strip_set_pixel(led, 0, 50, 0, 0); // verde fraquinho
    led_strip_refresh(led);
}
void rgb_off(void)
{
    led_strip_clear(led);
}

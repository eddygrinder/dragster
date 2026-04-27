#include "qtr_sensors.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "strip_leds.h"
#include "esp_adc/adc_oneshot.h"
#include "motors_control.h"
// #include "braking.h"
#include "tuning.h"

static adc_oneshot_unit_handle_t adc1_handle; // só visível neste ficheiro

volatile CalibrationData calibData;     // Guarda sMin, sMax e limites
volatile bool prova_finalizada = false; // Flag fim de prova
static const char *TAG = "QTR";         // para ESP_LOGI

volatile bool speed_reduced = false; // já reduziu velocidade
volatile bool brake_done = false;    // já travou

// Array raw[] definido static no ficheiro, acessível apenas aqui
// Armazena leitura bruta dos 4 sensores
static int raw[4];
static const adc_channel_t channels[4] = {S1_CHANNEL, S2_CHANNEL, S3_CHANNEL, S4_CHANNEL};

void qtr_init(void)
{
    // -------------------------------
    // 3) Iniciar ADC
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
    adc_oneshot_config_channel(adc1_handle, LED_CHANNEL, &adc_config); // Configura canal do LED
}

int LED_START(void)
{
    int raw_led;
    adc_oneshot_read(adc1_handle, LED_CHANNEL, &raw_led);
    return raw_led;
}

float calculaPosicao(float s1, float s2, float s3, float s4)
{
    float soma = s1 + s2 + s3 + s4;
    if (soma < 0.01f)
        return 0.0f;
    return (s1 * -1.0f + s2 * -0.5f + s3 * 0.5f + s4 * 1.0f) / soma;
}

float normalize(int raw, int min, int max)
{
    if (raw < min)
        raw = min;
    if (raw > max)
        raw = max;
    return (float)(raw - min) / (float)(max - min);
}

void qtr_calibrate(uint32_t duration_ms)
{
    int raw[4];

    rgb_on();
    strip_set_color();

    // Inicializa limites
    for (int i = 0; i < 4; i++)
    {
        calibData.sMin[i] = 4095;
        calibData.sMax[i] = 0;
    }

    ESP_LOGI(TAG, "=== CALIBRACAO SENSORES ===");

    ESP_LOGI(TAG, "Move o Dragster lentamente sobre a linha e o fundo...");

    uint32_t inicio = xTaskGetTickCount();

    // -----------------------
    // CICLO DE CALIBRAÇÃO
    // -----------------------
    while (xTaskGetTickCount() - inicio < pdMS_TO_TICKS(duration_ms))
    {
        adc_oneshot_read(adc1_handle, S1_CHANNEL, &raw[0]);
        adc_oneshot_read(adc1_handle, S2_CHANNEL, &raw[1]);
        adc_oneshot_read(adc1_handle, S3_CHANNEL, &raw[2]);
        adc_oneshot_read(adc1_handle, S4_CHANNEL, &raw[3]);

        for (int i = 0; i < 4; i++)
        {
            if (raw[i] < calibData.sMin[i])
                calibData.sMin[i] = raw[i];
            if (raw[i] > calibData.sMax[i])
                calibData.sMax[i] = raw[i];
        }

        ESP_LOGI(TAG, "S1 raw=%4d min=%4d max=%4d | S2 raw=%4d min=%4d max=%4d",
                 raw[0], calibData.sMin[0], calibData.sMax[0], raw[1], calibData.sMin[1], calibData.sMax[1]);
        ESP_LOGI(TAG, "S3 raw=%4d min=%4d max=%4d | S4 raw=%4d min=%4d max=%4d",
                 raw[2], calibData.sMin[2], calibData.sMax[2], raw[3], calibData.sMin[3], calibData.sMax[3]);

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Limites de preto laterais
    calibData.limites.limite_s1 = calibData.sMin[0] + (calibData.sMax[0] - calibData.sMin[0]) * PRETO_PERCENT / 100;
    calibData.limites.limite_s4 = calibData.sMin[3] + (calibData.sMax[3] - calibData.sMin[3]) * PRETO_PERCENT / 100;

    ESP_LOGI(TAG, "\n=== Limites Preto ===");
    ESP_LOGI(TAG, "S0 limite=%4d S4 limite=%4d ", calibData.limites.limite_s1, calibData.limites.limite_s4);

    // -----------------------
    // RESULTADOS FINAIS
    // -----------------------
    ESP_LOGI(TAG, "=== Limites Preto ===");
    ESP_LOGI(TAG, "S1 limite=%4d S4 limite=%4d",
             calibData.limites.limite_s1, calibData.limites.limite_s4);

    ESP_LOGI(TAG, "Calibração concluída!");
    rgb_off();
}

// ===============================
// Leitura dos sensores QTR
// ===============================

// ===============================
// Seguimento da linha
// ===============================

/**
 * Executa o controlo de motores com base na posição da linha
 * Calcula a posição normalizada e chama motorControl()
 */
bool run_line_follower()
{
    float s_norm[4];            // valores normalizados
    float line_position = 0.0f; // posição calculada da linha

    for (int i = 0; i < 4; i++)
    {
        adc_oneshot_read(adc1_handle, channels[i], &raw[i]);
    }

    // perdeu a linha — só verifica após 300ms
    float threshold = calibData.sMin[1] +
                      (calibData.sMax[1] - calibData.sMin[1]) * 0.25f;

    if (raw[1] < threshold && raw[2] < threshold)
    {
        motors_coast();
        return true;
    }

    // Normaliza valores brutos para [0,1] usando calibração
    for (int i = 0; i < 4; i++)
    {
        s_norm[i] = normalize(raw[i], calibData.sMin[i], calibData.sMax[i]);
    }

    line_position = calculaPosicao(s_norm[0], s_norm[1], s_norm[2], s_norm[3]);
    motorControl(line_position);

    return false; // prova continua
}

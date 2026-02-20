#include "qtr_sensors.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tuning.h" // PRETO_PERCENT, S1_CHANNEL, etc.
#include "strip_leds.h"
#include "esp_adc/adc_oneshot.h"
#include "motors_control.h"

static adc_oneshot_unit_handle_t adc1_handle; // só visível neste ficheiro

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
}

static const char *TAG = "QTR"; // para ESP_LOGI

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

LimitesPreto qtr_calibrate(int *sMin, int *sMax, uint32_t duration_ms)
{
    int raw[4];
    LimitesPreto limites;

    rgb_on();
    strip_set_color();

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
            if (raw[i] < sMin[i])
                sMin[i] = raw[i];
            if (raw[i] > sMax[i])
                sMax[i] = raw[i];
        }

        ESP_LOGI(TAG, "S1 raw=%4d min=%4d max=%4d | S2 raw=%4d min=%4d max=%4d",
                 raw[0], sMin[0], sMax[0], raw[1], sMin[1], sMax[1]);
        ESP_LOGI(TAG, "S3 raw=%4d min=%4d max=%4d | S4 raw=%4d min=%4d max=%4d",
                 raw[2], sMin[2], sMax[2], raw[3], sMin[3], sMax[3]);

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Depois do loop de calibração, ao calcular resultados finais
    limites.limite_s1 = sMin[0] + (sMax[0] - sMin[0]) * PRETO_PERCENT / 100;
    limites.limite_s4 = sMin[3] + ((sMax[3] - sMin[3]) * PRETO_PERCENT / 100);

    ESP_LOGI(TAG, "\n=== Limites Preto ===");
    ESP_LOGI(TAG, "S0 limite=%4d S4 limite=%4d ", limites.limite_s1, limites.limite_s4);

    // -----------------------
    // RESULTADOS FINAIS
    // -----------------------
    ESP_LOGI(TAG, "\n=== RESULTADOS FINAIS DA CALIBRACAO ===");
    ESP_LOGI(TAG, "S1: min=%4d  max=%4d", sMin[0], sMax[0]);
    ESP_LOGI(TAG, "S2: min=%4d  max=%4d", sMin[1], sMax[1]);
    ESP_LOGI(TAG, "S3: min=%4d  max=%4d", sMin[2], sMax[2]);
    ESP_LOGI(TAG, "S4: min=%4d  max=%4d", sMin[3], sMax[3]);
    ESP_LOGI(TAG, "=======================================");
    ESP_LOGI(TAG, "Calibração concluída! Reinicia para correr o programa.");

    rgb_off(); // Apaga LED de calibração

    return limites;
}

void run_line_follower(int s1_min, int s1_max, int s2_min, int s2_max, int s3_min, int s3_max, int s4_min, int s4_max, int limite_s1, int limite_s4)
{
    int raw[4];
    float line_position = 0.0f; ///< Posição da linha calculada
    while (1)
    {
        // Ler valor de cada canal
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &raw[0]);
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &raw[1]);
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_3, &raw[2]);
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &raw[3]);

        if (raw[0] < 200 && raw[3] < 200)
        {
            // Para os motores
            motors_stop();
            printf("Prova terminada!\n");
            break; // sai do while
        }

        // Valores normalizados entre 0.0 e 1.0
        float s1_norm = normalize(raw[0], s1_min, s1_max);
        float s2_norm = normalize(raw[1], s2_min, s2_max);
        float s3_norm = normalize(raw[2], s3_min, s3_max);
        float s4_norm = normalize(raw[3], s4_min, s4_max);

        line_position = calculaPosicao(s1_norm, s2_norm, s3_norm, s4_norm);
        motorControl(line_position);

        // Esperar 1 ms antes da próxima leitura
        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }
}
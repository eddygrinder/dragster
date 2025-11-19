#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"

// Handles do ADC
adc_oneshot_unit_handle_t adc1_handle;

void sensorTask(void *pvParameters);

void app_main(void)
{
    // ================================
    // 1) Inicializar o ADC1
    // ================================
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1
    };
    adc_oneshot_new_unit(&init_config, &adc1_handle);

    // ================================
    // 2) Configurar cada canal (sensor)
    // ================================
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,   // Normalmente 12 bits
        .atten = ADC_ATTEN_DB_12            // Para ler até ~3.3V
    };
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config); // Sensor esq - verde/laranja
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_7, &config); // Sensor central esq branco/cinzento
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &config); // Sensor central dir vermelho/castanho
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_5, &config); // Sensor dir verde/amarelo

    // ================================
    // 3) Criar a tarefa que lê os sensores
    // ================================
    xTaskCreate(sensorTask, "Sensor Task", 2048, NULL, 1, NULL);
}

// ================================
// TAREFA QUE LÊ OS 3 SENSORES
// ================================
void sensorTask(void *pvParameters)
{
    int s1, s2, s3, s4; // Variáveis para armazenar os valores lidos

    while (1)
    {
        // Ler valor de cada canal
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &s1);
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &s2);
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &s3);
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &s4);

        // Imprimir resultados
        printf("S1: %d  S2: %d S3: %d   S4: %d\n", s1, s2, s3, s4);

        // Esperar 100 ms antes da próxima leitura
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

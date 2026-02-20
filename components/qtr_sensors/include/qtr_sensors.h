#pragma once
#include <stdint.h>
#include "esp_log.h"

// ----- Sensores QTR-8C -----
#define S1_CHANNEL ADC_CHANNEL_4 ///< Sensor esquerdo (Amarelo, GPIO34)
#define S2_CHANNEL ADC_CHANNEL_5 ///< Sensor central esquerdo (Castanho, GPIO35)
#define S3_CHANNEL ADC_CHANNEL_6 ///< Sensor central direito (Cinzento, GPIO39)
#define S4_CHANNEL ADC_CHANNEL_1 ///< Sensor direito (Castanho, GPIO33)

// Estrutura para limites de preto
typedef struct {
    int limite_s1;
    int limite_s4;
} LimitesPreto;

// Funções públicas
void qtr_init(void);
float calculaPosicao(float s1, float s2, float s3, float s4);
float normalize(int raw, int min, int max);
LimitesPreto qtr_calibrate(int *sMin, int *sMax, uint32_t duration_ms);
void run_line_follower(int s1_min, int s1_max, int s2_min, int s2_max, int s3_min, int s3_max, int s4_min, int s4_max, int limite_s1, int limite_s4); // loop contínuo


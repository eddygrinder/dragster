#pragma once
#include <stdint.h>
#include "esp_log.h"

// ----- Sensores QTR-8C -----
#define S1_CHANNEL ADC_CHANNEL_4 ///< Sensor esquerdo (Amarelo, GPIO5)
#define S2_CHANNEL ADC_CHANNEL_5 ///< Sensor central esquerdo (Castanho, GPIO6)
#define S3_CHANNEL ADC_CHANNEL_6 ///< Sensor central direito (Cinzento, GPIO7)
#define S4_CHANNEL ADC_CHANNEL_1 ///< Sensor direito (Castanho, GPIO2)
#define LED_CHANNEL ADC_CHANNEL_7 ///< Sensor do LED (Branco, GPIO8)

extern volatile uint32_t encoder_total_ticks; // declarada noutro ficheiro, visível globalmente

// Limites de preto
typedef struct {
    int limite_s1;
    int limite_s4;
} LimitesPreto;

// --- Estrutura de calibração completa ---
typedef struct {
    int sMin[4];              // mínimos dos 4 sensores
    int sMax[4];              // máximos dos 4 sensores
    LimitesPreto limites;     // limites laterais calculados
} CalibrationData;

typedef enum {
    LINE_NONE,
    LINE_START,
    LINE_FINISH,
    LINE_OFF_TRACK
} line_event_t;

// Funções públicas
void qtr_init(void);
float calculaPosicao(float s1, float s2, float s3, float s4);
float normalize(int raw, int min, int max);
void qtr_calibrate(uint32_t duration_ms);
int LED_START(void);

// Executa o seguidor de linha usando raw[4], retorna true se terminou a prova
bool run_line_follower();

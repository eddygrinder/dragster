#pragma once
#include <stdint.h>
#include <stdbool.h>

// ===============================
// Sensores (constantes)
// ===============================
#define MAX_DUTY_CYCLE 1023      ///< Valor máximo do PWM (10-bit)
#define PRETO_PERCENT 75         ///< Percentagem entre min e max para considerar "preto"
#define LINELOST_THRESHOLD 0.05f ///< Linha perdida se soma dos sensores normalizados < 0.05

// ===============================
// Timing / Loops (constantes)
// ===============================
#define LOOP_DELAY_MS 2          ///< Delay do loop principal em ms
#define CALIBRATION_TIME_MS 5000 ///< Duração da calibração em ms

#define TICKS_REDUCE_SPEED 1364 // ticks correspondentes a 8 m VERIFICAR
#define BREAK_TICKS 1700        // ticks correspondentes À DISTÂNCIA DE TRAVAGEM
#define KP_BRAKE 200             // ganho de travagem - ajustável conforme testes
#define MAX_BRAKE_PWM 1000      // valor máximo de PWM para travagem segura sem queimar drivers - ajustável conforme testes
#define STOP_THRESHOLD 3       // ticks por ciclo abaixo do qual consideramos que o robô parou - ajustável conforme testes
#define IGNORE_LINE_TICKS 5    // ticks para ignorar a linha de partida (ajustar conforme necessário)


// -------------------------------
// Protótipos de funções de tuning - KP e BASE_SPEED
// -------------------------------

void tuning_load();
void tuning_save();
void tuning_print_saved();
void save_final_ticks(uint32_t ticks_final);
uint32_t read_final_ticks(void);

// Estrutura de parâmetros de controlo - KP e BASE_SPEED
typedef struct
{
    int KP;                   // valor literal
    int BASE_SPEED;           // valor literal
    int BRAKE_DISTANCE_TICKS; // ticks desde linha até parar
    int dist_cm;              // distância de travagem em cm, calculada a partir dos ticks e gravada para referência futura

} tuning_t;

extern tuning_t tuning; // apenas declaração
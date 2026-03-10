#pragma once
#include <stdint.h>
#include <stdbool.h>

// tempo do short brake em ms
// Usado para manter os motores em curto (short brake) de forma contínua
// durante a travagem rápida. Ajustar entre 120–500 ms conforme testes.
// Valores menores apenas reduzem parte da velocidade; valores maiores
// param quase totalmente o dragster dentro de 1 metro.
#define SHORT_BRAKE_MS 500

// ===============================
// Sensores (constantes)
// ===============================
#define MAX_DUTY_CYCLE 1023      ///< Valor máximo do PWM (10-bit)
#define PRETO_PERCENT 75         ///< Percentagem entre min e max para considerar "preto"
#define LINELOST_THRESHOLD 0.05f ///< Linha perdida se soma dos sensores normalizados < 0.05

// ===============================
// Timing / Loops (constantes)
// ===============================
#define LOOP_DELAY_MS 10         ///< Delay do loop principal em ms
#define CALIBRATION_TIME_MS 5000 ///< Duração da calibração em ms

// -------------------------------
// Protótipos de funções de tuning - KP e BASE_SPEED
// -------------------------------

void tuning_load();
void tuning_save();
void tuning_print_saved();

// Estrutura de parâmetros de controlo - KP e BASE_SPEED
typedef struct
{
    int KP;                   // valor literal
    int BASE_SPEED;           // valor literal
    int BRAKE_DISTANCE_TICKS; // ticks desde linha até parar

} tuning_t;

extern tuning_t tuning; // apenas declaração
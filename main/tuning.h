#pragma once
#include <stdint.h>
#include <stdbool.h>

// ===============================
// PID (ajustável durante testes)
// ===============================
extern float KP;       ///< Ganho proporcional do controlador

// ===============================
// Motores (ajustável durante testes)
// ===============================
extern int BASE_SPEED; ///< Velocidade base dos motores (0 a 1023)

// ===============================
// Sensores (constantes)
// ===============================
#define MAX_DUTY_CYCLE      1023   ///< Valor máximo do PWM (10-bit)
#define PRETO_PERCENT       75     ///< Percentagem entre min e max para considerar "preto"
#define LINELOST_THRESHOLD  0.05f  ///< Linha perdida se soma dos sensores normalizados < 0.05

// ===============================
// Timing / Loops (constantes)
// ===============================
#define LOOP_DELAY_MS        10          ///< Delay do loop principal em ms
#define CALIBRATION_TIME_MS  5000        ///< Duração da calibração em ms

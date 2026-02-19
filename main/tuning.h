#pragma once
#include <stdint.h>
#include <stdbool.h>

// =======================================================
//                     PARÂMETROS DE AFINAÇÃO
// =======================================================

// ===============================
// PID (ajustável durante testes)
// ===============================
extern float KP;       ///< Ganho proporcional do controlador

// ===============================
// Motores (ajustável durante testes)
// ===============================
int BASE_SPEED = 680;   ///< Velocidade base dos motores (0 a 1023)

// ===============================
// Sensores (constantes)
// ===============================
const int MAX_DUTY_CYCLE = 1023;   ///< Valor máximo do PWM (10-bit)
const int PRETO_PERCENT = 75;      ///< Percentagem entre min e max para considerar "preto"
const float LINELOST_THRESHOLD = 0.05f; ///< Linha considerada perdida se soma dos sensores normalizados < 0.05

// ===============================
// Timing / Loops (constantes)
// ===============================
const int LOOP_DELAY_MS = 10;            ///< Delay do loop principal em ms
const uint32_t CALIBRATION_TIME_MS = 5000; ///< Duração da calibração em ms

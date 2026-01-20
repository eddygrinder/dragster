#pragma once

// ===============================
// Afinação do seguidor de linha
// ===============================

// ---------- PID ----------
float KP = 120.0f; ///< Ganho proporcional

// ---------- Motores ----------
#define BASE_SPEED 950;      ///< Velocidade base dos motores
//#define MAX_CORRECTION 800 // limite de correção do PID

// ---------- Sensores ----------
#define PRETO_PERCENT 75         // percentagem entre min e max
#define LINELOST_THRESHOLD 0.05f // linha perdida

// ---------- Timing ----------
#define LOOP_DELAY_MS 1 // delay do loop principal
#define CALIBRATION_TIME_MS 5000

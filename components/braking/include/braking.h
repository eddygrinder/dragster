#pragma once
#include <stdbool.h>
#include <stdint.h>

// Chamada quando detecta linha de fim de pista
// Retorna true quando o robot parou completamente
bool braking_short_brake(void);   // Opção 1 — SHORT BRAKE (atual, segura)
bool braking_reverse(void);       // Opção 2 — INVERSÃO (futura)

// Grava a distância de paragem medida na NVS
void braking_save_distance(uint32_t ticks);
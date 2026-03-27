#pragma once
#include <stdbool.h>
#include <stdint.h>

// Chamada quando detecta linha de fim de pista
// Retorna true quando o robot parou completamente
void braking_short_brake(void);   // Opção 1 — SHORT BRAKE (atual, segura)

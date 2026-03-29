#pragma once
#include <stdint.h>
#include "esp_err.h"

// Canal e timer do servo — coerente com estilo motores
#define SERVO_CHANNEL LEDC_CHANNEL_4
#define SERVO_TIMER   LEDC_TIMER_3

// Posição “home” padrão (em graus)
#define SERVO_HOME_ANGLE 0

// API do servo
esp_err_t servo_init(int gpio);
esp_err_t servo_set_angle(int angle);
int servo_get_angle(void);
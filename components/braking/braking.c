#include "braking.h"
#include "tuning.h"
#include "encoders.h"
#include "motors_control.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "strip_leds.h"

#define TAG "BRAKING"
#define BRAKE_TIMEOUT_MS 500 // máximo 500ms de travagem

// ================================================
// OPÇÃO 1 — SHORT BRAKE
// Corta potência e aplica curto nos motores
// Mede os ticks desde a linha até parar
// Grava na NVS para uso futuro
// ================================================
void  braking_short_brake(void)
{
    motors_stop_fast();      // Fase 1: corta PWM e aplica short brake
    gpio_set_level(PWM1, 1); // EN=1 para travagem
    gpio_set_level(PWM2, 1);
}

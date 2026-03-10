#include "braking.h"
#include "tuning.h"
#include "encoders.h"
#include "motors_control.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "BRAKING"

// ================================================
// OPÇÃO 1 — SHORT BRAKE
// Corta potência e aplica curto nos motores
// Mede os ticks desde a linha até parar
// Grava na NVS para uso futuro
// ================================================
bool braking_short_brake(void)
{
    uint32_t ticks_start = encoder_get_ticks();

    ESP_LOGI(TAG, "Linha detectada! A travar...");

    // Fase 1: corta PWM e aplica short brake
    motors_stop_fast();

    // Fase 2: mede ticks de paragem
    uint32_t ticks_stop = encoder_get_ticks();
    uint32_t ticks_braking = ticks_stop - ticks_start;

    const float dist_per_tick = 3.1415926f * 0.056f / 15.0f;
    float dist_m = ticks_braking * dist_per_tick;

    ESP_LOGI(TAG, "Ticks travagem: %" PRIu32 " | Distância: %.3f m", ticks_braking, dist_m);

    // Grava apenas se for um valor válido (maior que o anterior ou ainda não medido)
    if (tuning.BRAKE_DISTANCE_TICKS == 0 || ticks_braking > tuning.BRAKE_DISTANCE_TICKS)
    {
        braking_save_distance(ticks_braking);
    }

    return true; // robot parou
}

// ================================================
// OPÇÃO 2 — INVERSÃO SUAVE (esqueleto para futuro)
// Só activa quando tiver dados reais de velocidade segura
// ================================================
bool braking_reverse(void)
{
    // TODO: implementar quando tiver threshold de velocidade segura
    // 1. SHORT BRAKE até vel < SAFE_REVERSE_SPEED
    // 2. Inversão a PWM baixo até parar
    ESP_LOGW(TAG, "braking_reverse() ainda não implementado — a usar short brake");
    return braking_short_brake();
}

// ================================================
// Grava distância de paragem na NVS
// ================================================
void braking_save_distance(uint32_t ticks)
{
    tuning.BRAKE_DISTANCE_TICKS = ticks;
    tuning_save();
    ESP_LOGI(TAG, "BRAKE_DISTANCE_TICKS gravado: %" PRIu32, ticks);
}
#include "tuning.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

// Instância global
tuning_t tuning = {
    .KP = 40,
    .BASE_SPEED = 580,
    .BRAKE_DISTANCE_TICKS = 0 // valor inicial
};

void tuning_load()
{
    nvs_handle_t handle;
    if (nvs_open("tuning", NVS_READWRITE, &handle) == ESP_OK)
    {
        int32_t val;
        if (nvs_get_i32(handle, "kp", &val) == ESP_OK)
            tuning.KP = val;
        if (nvs_get_i32(handle, "speed", &val) == ESP_OK)
            tuning.BASE_SPEED = val;
        if (nvs_get_i32(handle, "brake_ticks", &val) == ESP_OK) // nova leitura para BRAKE_DISTANCE_TICKS
            tuning.BRAKE_DISTANCE_TICKS = val;
        nvs_close(handle);
    }
}

void tuning_save()
{
    nvs_handle_t handle;
    if (nvs_open("tuning", NVS_READWRITE, &handle) == ESP_OK)
    {
        nvs_set_i32(handle, "kp", tuning.KP);
        nvs_set_i32(handle, "speed", tuning.BASE_SPEED);
        nvs_set_i32(handle, "brake_ticks", tuning.BRAKE_DISTANCE_TICKS); // BRAKE_DISTANCE_TICKS
        nvs_commit(handle);
        nvs_close(handle);
    }
}

void tuning_print_saved()
{
    nvs_handle_t handle;
    int32_t kp_saved = 0;
    int32_t speed_saved = 0;

    if (nvs_open("tuning", NVS_READONLY, &handle) == ESP_OK)
    {
        nvs_get_i32(handle, "kp", &kp_saved);
        nvs_get_i32(handle, "speed", &speed_saved);
        nvs_close(handle);

        ESP_LOGI("TUNING", "Valores salvos na NVS -> KP: %d, BASE_SPEED: %d", kp_saved, speed_saved);
    }
    else
    {
        ESP_LOGE("TUNING", "Não foi possível abrir NVS para leitura");
    }
}
#include "tuning.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

/*
250, 42 - 2.85 seg
260, 35 - OK 
270, 35 - Optimo
280, 33 - Optimo - FALSO
280, 27 - VERIFICAR
300, 32/31 - OK
310, 40 - OPTIMO
PROVA: 235, 55
testes: 
245, 76
teste: 250, 94 - APARENTEMENTE, OK
PROXIMO PASSO, 250, ?
260, 98
*/

// Instância global
tuning_t tuning = {
    .KP = 98,
    .BASE_SPEED = 290,
    .KD = 10,
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

void save_final_ticks(uint32_t ticks_final)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // abrir espaço "storage" para escrita
    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        printf("Erro ao abrir NVS\n");
        return;
    }

    // gravar o valor
    err = nvs_set_u32(nvs_handle, "tick_final", ticks_final);
    if (err != ESP_OK)
    {
        printf("Erro ao gravar tick_final\n");
    }
    else
    {
        nvs_commit(nvs_handle); // obrigatório para efetivar
        printf("tick_final gravado: %" PRIu32 "\n", ticks_final);
    }

    nvs_close(nvs_handle);
}

uint32_t read_final_ticks(void)
{
    nvs_handle_t nvs_handle;
    uint32_t ticks = 0;

    if (nvs_open("storage", NVS_READONLY, &nvs_handle) == ESP_OK)
    {
        nvs_get_u32(nvs_handle, "tick_final", &ticks);
        nvs_close(nvs_handle);
        ESP_LOGI("TUNING", "Valores salvos na NVS -> ticks: %" PRIu32, ticks);
    }

    return ticks; // necessário devolver o valor lido
}
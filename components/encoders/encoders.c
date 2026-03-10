// encoder.c
#include "encoders.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

#define ENCODER_GPIO 39 // GPIO do encoder IR
#define TAG_ENC "ENCODER"

static volatile uint32_t tick_count = 0;
volatile uint32_t encoder_ticks;

// ISR — chamada a cada flanco de subida
static void IRAM_ATTR encoder_isr_handler(void *arg)
{
    tick_count++;
    // Temporário para debug — remove depois
    esp_rom_printf("tick %lu\n", tick_count);
}

void encoder_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << ENCODER_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE, // dispara no flanco de subida
    };
    gpio_config(&cfg);

    gpio_install_isr_service(0); // instala serviço de ISR (só 1x no projecto)
    gpio_isr_handler_add(ENCODER_GPIO, encoder_isr_handler, NULL);

    ESP_LOGI(TAG_ENC, "Encoder IR iniciado no GPIO %d", ENCODER_GPIO);
}

uint32_t encoder_get_ticks(void)
{
    return tick_count;
}

void encoder_reset_ticks(void)
{
    tick_count = 0;
}
// blinkt.c
#include "blinkt.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include "esp_rom_sys.h" // Para o ets_delay_us

#define CLK GPIO_NUM_23
#define DAT GPIO_NUM_18
#define NUM_LEDS 8

typedef struct {
    uint8_t r, g, b;
    float   brightness; // 0.0 – 1.0
} led_t;

static led_t leds[NUM_LEDS];

// --- Funções internas ---
static void clock_pulse(void)
{
    gpio_set_level(CLK, 0);
    esp_rom_delay_us(1);  // espera ~1 microsegundo
    gpio_set_level(CLK, 1);
    esp_rom_delay_us(1);  // espera ~1 microsegundo
}

static void send_byte(uint8_t b)
{
    for (int i = 0; i < 8; i++) {
        gpio_set_level(DAT, (b & 0x80) ? 1 : 0);
        clock_pulse();
        b <<= 1;
    }
}

static void show(void)
{
    // Start Frame (32 zeros)
    for (int i = 0; i < 4; i++) send_byte(0x00);

    // LEDs (Global + BGR)
    for (int i = 0; i < NUM_LEDS; i++) {
        // Brilho: 3 bits '1' seguidos de 5 bits de brilho (0-31)
        uint8_t brightness_val = (uint8_t)(leds[i].brightness * 31.0f);
        if (brightness_val > 31) brightness_val = 31;
        
        send_byte(0xE0 | brightness_val); // 0xE0 = 11100000
        send_byte(leds[i].b);
        send_byte(leds[i].g);
        send_byte(leds[i].r);
    }

    // End Frame (Pelo menos 32 bits de 1s para 8 LEDs)
    for (int i = 0; i < 4; i++) send_byte(0xFF);
}

// --- Funções públicas ---
void blinkt_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DAT) | (1ULL << CLK),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Inicializar os níveis a 0
    gpio_set_level(DAT, 0);
    gpio_set_level(CLK, 0);

    // Limpar LEDs no arranque
    for (int i = 0; i < NUM_LEDS; i++) {
        leds[i].r = leds[i].g = leds[i].b = 0;
        leds[i].brightness = 0.0f;
    }
    show();
}

void blinkt_set_all(uint8_t r, uint8_t g, uint8_t b, float brightness)
{
    for (int i = 0; i < NUM_LEDS; i++) {
        leds[i].r = r;
        leds[i].g = g;
        leds[i].b = b;
        leds[i].brightness = brightness;
    }
    show();
}

void blinkt_white(void)
{
    blinkt_set_all(255, 255, 255, 1.0f);
}

void blinkt_clear(void)
{
    blinkt_set_all(0, 0, 0, 0.0f);
}

// blinkt.c
#include "blinkt.h"
#include "driver/gpio.h"

#define DAT GPIO_NUM_18
#define CLK GPIO_NUM_23
#define NUM_LEDS 8

typedef struct {
    uint8_t r, g, b;
    float   brightness; // 0.0 – 1.0
} led_t;

static led_t leds[NUM_LEDS];

// --- Funções internas ---
static void clock_pulse(void)
{
    gpio_set_level(CLK, 1);
    gpio_set_level(CLK, 0);
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
    // start frame (4 bytes de zero)
    for (int i = 0; i < 4; i++) send_byte(0x00);

    // LEDs
    for (int i = 0; i < NUM_LEDS; i++) {
        uint8_t gb = 0b11100000 | (uint8_t)(leds[i].brightness * 31.0f);
        send_byte(gb);
        send_byte(leds[i].b);
        send_byte(leds[i].g);
        send_byte(leds[i].r);
    }

    // end frame: pelo menos (NUM_LEDS + 15)/16 bytes de 0xFF
    int end_bytes = (NUM_LEDS + 15) / 16;
    for (int i = 0; i < end_bytes; i++) send_byte(0xFF);
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

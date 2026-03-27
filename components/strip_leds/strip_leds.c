// stripleds.c

#include "led_strip.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include "esp_rom_sys.h" // Para o

#define LED_STRIP_GPIO 9
#define STRIP_LED_NUM 8
#define RGB_LED_GPIO 38

static led_strip_handle_t led;
static led_strip_handle_t led_strip;

void strip_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO,
        .max_leds = STRIP_LED_NUM,
        .led_model = LED_MODEL_WS2812,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .flags = {
            .invert_out = false,
        },
    };

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags = {
            .with_dma = false,
        },
    };

    led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
}

void strip_set_color(void)
{
    for (int i = 0; i < STRIP_LED_NUM; i++)
    {
        led_strip_set_pixel(led_strip, i, 255, 255, 255); // Red color
    }

    led_strip_refresh(led_strip);
}

void strip_set_red(void)
{
    for (int i = 0; i < STRIP_LED_NUM; i++)
    {
        led_strip_set_pixel(led_strip, i, 255, 0, 0); // Red color
    }

    led_strip_refresh(led_strip);
}

void strip_set_green(void)
{
    for (int i = 0; i < STRIP_LED_NUM; i++)
    {
        led_strip_set_pixel(led_strip, i, 0, 255, 0); // Green color
    }

    led_strip_refresh(led_strip);
}

void strip_set_blue(void)
{
    for (int i = 0; i < STRIP_LED_NUM; i++)
    {
        led_strip_set_pixel(led_strip, i, 0, 0, 255); // Blue color
    }

    led_strip_refresh(led_strip);
}

void strip_set_yellow(void)
{
    for (int i = 0; i < STRIP_LED_NUM; i++)
    {
        led_strip_set_pixel(led_strip, i, 255, 255, 0); // Yellow color
    }

    led_strip_refresh(led_strip);
}


void strip_off(void)

{
    led_strip_clear(led_strip);
}

void rgb_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_LED_GPIO,
        .max_leds = 1,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
    };

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
    };

    led_strip_new_rmt_device(&strip_config, &rmt_config, &led);
    led_strip_clear(led); // começa desligado
}

void rgb_on(void)
{
    led_strip_set_pixel(led, 0, 50, 0, 0); // verde fraquinho
    led_strip_refresh(led);
}

void rgb_off(void)
{
    led_strip_clear(led);
}
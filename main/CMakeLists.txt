idf_component_register(
    SRCS "Dragster.c" "tuning.c"
    INCLUDE_DIRS "."
    REQUIRES bt nvs_flash
    PRIV_REQUIRES qtr_sensors motors_control strip_leds esp_adc driver spi_flash freertos
)
idf_component_register(
    SRCS "Dragster.c"
    INCLUDE_DIRS "."
    REQUIRES servo bt motors_control tuning ble
    PRIV_REQUIRES qtr_sensors encoders strip_leds esp_adc driver spi_flash freertos
)
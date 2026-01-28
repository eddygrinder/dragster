idf_component_register(
    SRCS "Dragster.c"
    INCLUDE_DIRS "."
    REQUIRES bt nvs_flash
    PRIV_REQUIRES blinkt esp_adc driver spi_flash freertos
)
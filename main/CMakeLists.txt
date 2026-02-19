idf_component_register(
    SRCS "Dragster.c"
    INCLUDE_DIRS "."
    REQUIRES bt nvs_flash
    PRIV_REQUIRES stripleds esp_adc driver spi_flash freertos
)
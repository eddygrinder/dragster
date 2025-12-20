idf_component_register(
    SRCS "Dragster.c"
    INCLUDE_DIRS "." 
    PRIV_REQUIRES blinkt esp_adc driver spi_flash
)
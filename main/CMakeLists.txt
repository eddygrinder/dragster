idf_component_register(
    SRCS "Dragster.c"
    INCLUDE_DIRS "."
    REQUIRES bt
    PRIV_REQUIRES qtr_sensors motors_control tuning encoders ble strip_leds esp_adc driver spi_flash freertos braking
)
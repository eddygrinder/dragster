idf_component_register(
    SRCS "qtr_sensors.c" 
    INCLUDE_DIRS "include" 
    PRIV_REQUIRES main driver freertos strip_leds tuning esp_adc motors_control encoders
)
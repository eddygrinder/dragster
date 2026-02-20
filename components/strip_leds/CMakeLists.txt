idf_component_register(
    SRCS "strip_leds.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver freertos led_strip
)

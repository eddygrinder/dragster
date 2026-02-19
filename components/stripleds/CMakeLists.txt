idf_component_register(
    SRCS "stripleds.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver freertos led_strip
)

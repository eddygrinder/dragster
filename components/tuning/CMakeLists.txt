idf_component_register(
    SRCS "tuning.c"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash
    PRIV_REQUIRES driver freertos led_strip
)

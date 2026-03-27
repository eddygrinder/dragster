idf_component_register(
    SRCS "braking.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES main driver freertos encoders strip_leds tuning motors_control
)

idf_component_register(
    SRCS "motors_control.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES main driver tuning freertos
)

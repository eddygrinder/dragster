idf_component_register(
    SRCS "braking.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES main driver freertos encoders tuning motors_control
)

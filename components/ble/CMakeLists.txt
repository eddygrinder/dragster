idf_component_register(
    SRCS "ble.c"
    REQUIRES bt
    INCLUDE_DIRS "include"
    PRIV_REQUIRES tuning nvs_flash freertos
)
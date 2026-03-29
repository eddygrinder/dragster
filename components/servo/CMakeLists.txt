# CMakeLists.txt do componente servo

idf_component_register(
    SRCS "servo.c"
    INCLUDE_DIRS "include"   # garante que servo.h está visível para quem linkar o component
    PRIV_REQUIRES driver freertos
)
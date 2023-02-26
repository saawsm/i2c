idf_component_register(
    SRCS i2c.c
    INCLUDE_DIRS "include"
    REQUIRES driver freertos 
    PRIV_REQUIRES log
)
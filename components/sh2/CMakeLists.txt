idf_component_register(
    SRCS
        "sh2_util.c"
        "sh2_SensorValue.c"
        "sh2.c"
        "shtp.c"
        "sh2_hal_espidf_spi.c"
        "sh2_hal_espidf_i2c.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_spi esp_driver_i2c esp_timer gimbal_config
)

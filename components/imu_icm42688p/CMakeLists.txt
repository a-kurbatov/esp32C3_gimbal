idf_component_register(SRCS "imu_icm42688p.c"
                      INCLUDE_DIRS "include"
                      REQUIRES imu esp_driver_spi esp_driver_i2c gimbal_config)

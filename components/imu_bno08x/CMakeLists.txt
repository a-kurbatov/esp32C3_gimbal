idf_component_register(SRCS "imu_bno08x.c"
                      INCLUDE_DIRS "include"
                      REQUIRES imu esp_driver_i2c esp_driver_spi gimbal_config driver sh2 esp_timer)

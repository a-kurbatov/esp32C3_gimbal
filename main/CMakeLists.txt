idf_component_register(SRCS "main.c"
                      REQUIRES nvs_flash imu imu_icm42688p imu_bno08x servo_pwm button gimbal_config)

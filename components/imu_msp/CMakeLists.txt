idf_component_register(
    SRCS "imu_msp.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer
)

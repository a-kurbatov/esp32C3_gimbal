idf_component_register(SRCS "imu_emulator.c"
                      INCLUDE_DIRS "include"
                      REQUIRES imu
                      PRIV_REQUIRES esp_timer)

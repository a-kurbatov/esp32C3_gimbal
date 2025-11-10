idf_component_register(SRCS "imu_factory.c"
                      INCLUDE_DIRS "include"
                      REQUIRES imu_icm42688p imu_bno08x imu_emulator)

#pragma once
#include "esp_err.h"
#include "imu/imu_types.h"

esp_err_t imu_icm42688p_init(void);
esp_err_t imu_icm42688p_read(imu_sample_t *out);

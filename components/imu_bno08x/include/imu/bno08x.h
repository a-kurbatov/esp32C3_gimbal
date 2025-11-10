#pragma once
#include "esp_err.h"
#include "imu/imu_types.h"

esp_err_t imu_bno08x_init(void);
esp_err_t imu_bno08x_read(imu_sample_t *out);

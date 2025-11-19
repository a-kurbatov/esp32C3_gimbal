#pragma once
#include "esp_err.h"
#include "imu_types.h"

typedef struct imu_driver_s imu_driver_t;

typedef struct {
    esp_err_t (*init)(void);
    esp_err_t (*read)(imu_sample_t *out);
} imu_vtbl_t;

struct imu_driver_s {
    const imu_vtbl_t *v;
};

// Returns singleton driver selected via Kconfig
imu_driver_t *imu_get(void);

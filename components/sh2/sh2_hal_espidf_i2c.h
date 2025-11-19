#pragma once
#include <stdint.h>
#include "driver/i2c_master.h"
#include "sh2_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// Configure the I2C HAL with an already-created device handle and optional GPIOs.
void sh2_hal_i2c_config(i2c_master_dev_handle_t dev, int int_gpio, int wake_gpio, int rst_gpio);

// Retrieve the HAL object for passing to sh2_open().
sh2_Hal_t *sh2_hal_i2c_get(void);

// For compatibility with existing code paths that open a brief INT bypass window.
void sh2_hal_spi_bypass_int_until(uint32_t until_us);

#ifdef __cplusplus
}
#endif

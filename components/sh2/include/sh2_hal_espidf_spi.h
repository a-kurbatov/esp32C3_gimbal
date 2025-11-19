#pragma once
#include <stdint.h>
#include "driver/spi_master.h"
#include "sh2_hal.h"
#ifdef __cplusplus
extern "C" {
#endif
// Configure the SPI device handle and INT GPIO used by the SH2 HAL
void sh2_hal_spi_config(spi_device_handle_t dev, int int_gpio, int wake_gpio, int rst_gpio);
// Get pointer to the HAL instance for passing to sh2_open()
sh2_Hal_t *sh2_hal_spi_get(void);
// Temporarily bypass INT gating until the given host time (us) for diagnostics
void sh2_hal_spi_bypass_int_until(uint32_t until_us);
#ifdef __cplusplus
}
#endif

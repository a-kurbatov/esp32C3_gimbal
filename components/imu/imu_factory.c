#include "imu/imu.h"
#include "sdkconfig.h"
#include "esp_log.h"

// Forward decls
esp_err_t imu_icm42688p_init(void);
esp_err_t imu_icm42688p_read(imu_sample_t *out);

esp_err_t imu_bno08x_init(void);
esp_err_t imu_bno08x_read(imu_sample_t *out);

esp_err_t imu_emulator_init(void);
esp_err_t imu_emulator_read(imu_sample_t *out);

static esp_err_t not_impl(void) { return ESP_ERR_NOT_SUPPORTED; }

static imu_driver_t s_drv;
static imu_vtbl_t s_tbl;

imu_driver_t *imu_get(void)
{
#if CONFIG_GIMBAL_IMU_SELECT_EMULATOR
    s_tbl.init = imu_emulator_init;
    s_tbl.read = imu_emulator_read;
#elif CONFIG_GIMBAL_IMU_SELECT_ICM42688P
    s_tbl.init = imu_icm42688p_init;
    s_tbl.read = imu_icm42688p_read;
#elif CONFIG_GIMBAL_IMU_SELECT_BNO08X
    s_tbl.init = imu_bno08x_init;
    s_tbl.read = imu_bno08x_read;
#else
    s_tbl.init = not_impl;
    s_tbl.read = (esp_err_t (*)(imu_sample_t *))not_impl;
#endif
    s_drv.v = &s_tbl;
    return &s_drv;
}

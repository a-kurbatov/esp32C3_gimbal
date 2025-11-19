#include "imu/icm42688p.h"
#include "gimbal_config.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include <string.h>

static const char *TAG = "icm42688p";

#define ICM_WHO_AM_I      0x75
#define ICM_WHO_AM_I_VAL  0x47  // expected value for ICM-42688-P

#if CONFIG_GIMBAL_ICM_USE_SPI
static spi_device_handle_t s_icm_spi;
#else
static i2c_master_dev_handle_t s_icm_i2c;
#endif

static esp_err_t icm_spi_read(uint8_t reg, uint8_t *val)
{
#if CONFIG_GIMBAL_ICM_USE_SPI
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), 0x00 };
    uint8_t rx[2] = {0};
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t err = spi_device_transmit(s_icm_spi, &t);
    if (err != ESP_OK) return err;
    *val = rx[1];
    return ESP_OK;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t imu_icm42688p_init(void)
{
#if CONFIG_GIMBAL_ICM_USE_SPI
    // SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = CONFIG_GIMBAL_SPI_MOSI_GPIO,
        .miso_io_num = CONFIG_GIMBAL_SPI_MISO_GPIO,
        .sclk_io_num = CONFIG_GIMBAL_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
        .flags = 0,
        .intr_flags = 0,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO), TAG, "spi bus init");

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000, // 1 MHz init
        .mode = 0,
        .spics_io_num = CONFIG_GIMBAL_SPI_CS_GPIO,
        .queue_size = 1,
        .flags = 0,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(SPI2_HOST, &devcfg, &s_icm_spi), TAG, "add device");

    uint8_t who = 0;
    esp_err_t err = icm_spi_read(ICM_WHO_AM_I, &who);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I read fail: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "WHO_AM_I=0x%02X", who);
    if (who != ICM_WHO_AM_I_VAL) {
        ESP_LOGW(TAG, "Unexpected WHO_AM_I (expected 0x%02X)", ICM_WHO_AM_I_VAL);
    }
    // NOTE: Full configuration of ICM-42688-P (power, ODR, scales, filters) is not implemented here.
    // This stub verifies SPI connectivity. Replace with a proper driver for real data.
    return ESP_OK;
#else
    // I2C path using new v2 API
    i2c_master_bus_config_t buscfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CONFIG_GIMBAL_I2C_SDA_GPIO,
        .scl_io_num = CONFIG_GIMBAL_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&buscfg, &bus), TAG, "i2c bus");

    i2c_device_config_t devcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x68,
        .scl_speed_hz = CONFIG_GIMBAL_I2C_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_probe(bus, devcfg.device_address, 1000), TAG, "probe");
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &devcfg, &s_icm_i2c), TAG, "add dev");
    ESP_LOGI(TAG, "ICM-42688-P I2C device added at 0x%02X", (int)devcfg.device_address);
    return ESP_OK;
#endif
}

esp_err_t imu_icm42688p_read(imu_sample_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    // TODO: Replace with real register reads and unit conversion.
    // For now, return a dummy sample (flat, no rotation)
    out->ax = 0; out->ay = 0; out->az = 1.0f;
    out->gx = 0; out->gy = 0; out->gz = 0;
    return ESP_OK;
}

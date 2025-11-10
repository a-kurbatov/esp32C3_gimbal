#include "imu/bno08x.h"
#include "gimbal_config.h"
#include "sdkconfig.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#if CONFIG_GIMBAL_BNO08X_USE_SPI
#include "sh2_hal_espidf_spi.h"
#elif CONFIG_GIMBAL_BNO08X_USE_I2C
#include "sh2_hal_espidf_i2c.h"
#endif
#include "sh2.h"
#include "sh2_SensorValue.h"

static const char *TAG = "bno08x";
static struct {
    float ax, ay, az, gx, gy, gz;
} s_latest;
static volatile bool s_sh2_ready = false;
static volatile bool s_sensors_enabled = false;
static volatile uint32_t s_last_evt_us = 0;
static volatile uint32_t s_reset_us = 0;
static void sensor_cb(void *cookie, sh2_SensorEvent_t *pEvent) {
    (void)cookie;
    sh2_SensorValue_t v;
    if (sh2_decodeSensorEvent(&v, pEvent) == 0) {
        static bool first = true;
        switch (v.sensorId) {
            case SH2_ACCELEROMETER:
                s_latest.ax = v.un.accelerometer.x;
                s_latest.ay = v.un.accelerometer.y;
                s_latest.az = v.un.accelerometer.z;
                if (first) { ESP_LOGI(TAG, "ACC evt: ax=%.2f ay=%.2f az=%.2f", s_latest.ax, s_latest.ay, s_latest.az); first = false; }
                s_last_evt_us = (uint32_t)esp_timer_get_time();
                break;
            case SH2_GYROSCOPE_CALIBRATED:
                s_latest.gx = v.un.gyroscope.x;
                s_latest.gy = v.un.gyroscope.y;
                s_latest.gz = v.un.gyroscope.z;
                if (first) { ESP_LOGI(TAG, "GYR evt: gx=%.2f gy=%.2f gz=%.2f", s_latest.gx, s_latest.gy, s_latest.gz); first = false; }
                s_last_evt_us = (uint32_t)esp_timer_get_time();
                break;
            default: break;
        }
    }
}
static i2c_master_dev_handle_t s_dev;
static spi_device_handle_t s_spi;
static TaskHandle_t s_task;
static volatile bool s_task_running = false;
static volatile uint32_t s_enable_us = 0;
static void bno08x_spi_task(void *arg);

esp_err_t imu_bno08x_init(void)
{
#if CONFIG_GIMBAL_BNO08X_USE_I2C || CONFIG_GIMBAL_BNO08X_USE_SPI
    // SH-2 will be opened in the SPI service task.
#endif
#if CONFIG_GIMBAL_BNO08X_USE_I2C
    i2c_master_bus_config_t buscfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CONFIG_GIMBAL_I2C_SDA_GPIO,
        .scl_io_num = CONFIG_GIMBAL_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&buscfg, &bus), TAG, "bus");

    i2c_device_config_t devcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CONFIG_GIMBAL_BNO08X_ADDR,
        .scl_speed_hz = CONFIG_GIMBAL_I2C_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &devcfg, &s_dev), TAG, "add dev");
#endif

#if CONFIG_GIMBAL_BNO08X_USE_SPI
    // Initialize SPI bus (shared with other devices if already initialized)
    spi_bus_config_t spi_bus = {
        .mosi_io_num = CONFIG_GIMBAL_SPI_MOSI_GPIO,
        .miso_io_num = CONFIG_GIMBAL_SPI_MISO_GPIO,
        .sclk_io_num = CONFIG_GIMBAL_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1024,
    };
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &spi_bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_RETURN_ON_ERROR(err, TAG, "spi bus init");
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 500 * 1000, // 500 kHz for bring-up
        .mode = CONFIG_GIMBAL_BNO08X_SPI_MODE,
        .spics_io_num = CONFIG_GIMBAL_SPI_CS_GPIO,
        .queue_size = 6,
        .flags = 0,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(SPI2_HOST, &devcfg, &s_spi), TAG, "spi add dev");

    // Configure SH2 HAL for SPI (with optional WAKE/RESET)
    sh2_hal_spi_config(s_spi, CONFIG_GIMBAL_BNO08X_INT_GPIO,
                       CONFIG_GIMBAL_BNO08X_WAKE_GPIO,
                       CONFIG_GIMBAL_BNO08X_RST_GPIO);
    s_task_running = true;
    // Lower priority to avoid starving IDLE; modest stack
    xTaskCreate(bno08x_spi_task, "bno08x_spi", 4096, NULL, 1, &s_task);
#elif CONFIG_GIMBAL_BNO08X_USE_I2C
    // Configure SH2 HAL for I2C
    sh2_hal_i2c_config(s_dev, CONFIG_GIMBAL_BNO08X_INT_GPIO,
                       CONFIG_GIMBAL_BNO08X_WAKE_GPIO,
                       CONFIG_GIMBAL_BNO08X_RST_GPIO);
    s_task_running = true;
    xTaskCreate(bno08x_spi_task, "bno08x_i2c", 4096, NULL, 1, &s_task);
#endif

    // Optional INT pin configuration (not used by stub yet)
    if (CONFIG_GIMBAL_BNO08X_INT_GPIO >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << CONFIG_GIMBAL_BNO08X_INT_GPIO,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);
        ESP_LOGI(TAG, "BNO08x INT mapped to GPIO %d", CONFIG_GIMBAL_BNO08X_INT_GPIO);
    }

    #if CONFIG_GIMBAL_BNO08X_USE_SPI
    ESP_LOGI(TAG, "BNO08x SPI stub initialized: SCLK=%d MOSI=%d MISO=%d CS=%d",
             CONFIG_GIMBAL_SPI_SCLK_GPIO, CONFIG_GIMBAL_SPI_MOSI_GPIO,
             CONFIG_GIMBAL_SPI_MISO_GPIO, CONFIG_GIMBAL_SPI_CS_GPIO);
    #elif CONFIG_GIMBAL_BNO08X_USE_I2C
    ESP_LOGI(TAG, "BNO08x I2C init: SDA=%d SCL=%d addr=0x%02X",
             CONFIG_GIMBAL_I2C_SDA_GPIO, CONFIG_GIMBAL_I2C_SCL_GPIO, CONFIG_GIMBAL_BNO08X_ADDR);
    #endif
    return ESP_OK;
}

static void event_cb(void *cookie, sh2_AsyncEvent_t *pEvent)
{
    (void)cookie;
    if (!pEvent) return;
    if (pEvent->eventId == SH2_RESET) {
        ESP_LOGI(TAG, "SH2 reset complete");
        // Set the flag only; main task will enable sensors
        s_sh2_ready = true;
        s_reset_us = (uint32_t)esp_timer_get_time();
    } else if (pEvent->eventId == SH2_GET_FEATURE_RESP) {
        sh2_SensorId_t sid = pEvent->sh2SensorConfigResp.sensorId;
        const sh2_SensorConfig_t *c = &pEvent->sh2SensorConfigResp.sensorConfig;
        ESP_LOGI(TAG, "GET_FEATURE_RESP sid=0x%02X AO=%d WK=%d CS=%d rel=%d dS=%u int=%lu us",
                 (unsigned)sid,
                 (int)c->alwaysOnEnabled,
                 (int)c->wakeupEnabled,
                 (int)c->changeSensitivityEnabled,
                 (int)c->changeSensitivityRelative,
                 (unsigned)c->changeSensitivity,
                 (unsigned long)c->reportInterval_us);
    }
}

static void bno08x_spi_task(void *arg)
{
    (void)arg;
    // Open SH2 with selected HAL, register callbacks
    #if CONFIG_GIMBAL_BNO08X_USE_SPI
    sh2_Hal_t *hal = sh2_hal_spi_get();
    #elif CONFIG_GIMBAL_BNO08X_USE_I2C
    sh2_Hal_t *hal = sh2_hal_i2c_get();
    #endif
    int rc = sh2_open(hal, event_cb, NULL);
    ESP_LOGI(TAG, "sh2_open rc=%d", rc);
    // Issue a device reset to ensure adverts and a clean start
    (void)sh2_devReset();
    // Open a short bypass window and service to drain initial adverts
    #if CONFIG_GIMBAL_BNO08X_USE_SPI
    sh2_hal_spi_bypass_int_until(((uint32_t)esp_timer_get_time()) + 500000); // 500 ms
    #elif CONFIG_GIMBAL_BNO08X_USE_I2C
    sh2_hal_i2c_bypass_int_until(((uint32_t)esp_timer_get_time()) + 500000);
    #endif
    for (int i = 0; i < 100; ++i) { sh2_service(); vTaskDelay(pdMS_TO_TICKS(5)); }
    // Try to fetch product IDs to confirm link (may fail until adverts processed)
    //sh2_ProductIds_t ids = {0};
    //rc = sh2_getProdIds(&ids);
    //ESP_LOGI(TAG, "sh2_getProdIds rc=%d entries=%u", rc, (unsigned)ids.numEntries);
    // Request a device reset to trigger adverts if not seen yet
    //rc = sh2_devReset();
    //ESP_LOGI(TAG, "sh2_devReset rc=%d", rc);
    // Log INT level at startup
    if (CONFIG_GIMBAL_BNO08X_INT_GPIO >= 0) {
        ESP_LOGI(TAG, "INT level (startup)=%d", gpio_get_level(CONFIG_GIMBAL_BNO08X_INT_GPIO));
    }
    sh2_setSensorCallback(sensor_cb, NULL);
    while (s_task_running) {
        // Service the stack; if INT asserted, drain a few fragments then yield
        if (CONFIG_GIMBAL_BNO08X_INT_GPIO >= 0 && gpio_get_level(CONFIG_GIMBAL_BNO08X_INT_GPIO) == 0) {
            for (int i = 0; i < 3; ++i) {
                sh2_service();
            }
            // brief yield to avoid starving IDLE/WDT
            vTaskDelay(pdMS_TO_TICKS(1));
        } else {
            // No INT, do a single service and sleep a bit to reduce CPU usage
            sh2_service();
            // During bring-up (not ready yet) or the first second after enabling, poll faster
            uint32_t now_fast = (uint32_t)esp_timer_get_time();
            if (!s_sh2_ready || (s_sensors_enabled && (now_fast - s_enable_us) < 1000000)) {
                vTaskDelay(pdMS_TO_TICKS(2));
            } else {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        // After reset, wait briefly, then enable sensors once
        if (s_sh2_ready && !s_sensors_enabled &&
            ((uint32_t)esp_timer_get_time() - s_reset_us) > 700000) {
            // Ensure device is ON
            (void)sh2_devOn();
            vTaskDelay(pdMS_TO_TICKS(50));

            // Re-initialize (some firmwares require host init after reset)
            int rc_init = sh2_reinitialize();
            ESP_LOGI(TAG, "sh2_reinitialize rc=%d", rc_init);
            // Briefly allow polling even without INT to catch short replies
            #if CONFIG_GIMBAL_BNO08X_USE_SPI
            sh2_hal_spi_bypass_int_until(((uint32_t)esp_timer_get_time()) + 1000000); // 1 s
            #elif CONFIG_GIMBAL_BNO08X_USE_I2C
            sh2_hal_i2c_bypass_int_until(((uint32_t)esp_timer_get_time()) + 1000000);
            #endif
            // Briefly service to drain any responses
            for (int i = 0; i < 10; ++i) { sh2_service(); vTaskDelay(pdMS_TO_TICKS(1)); }

            // Start with one report only: Game Rotation Vector at 50 Hz
            sh2_SensorConfig_t cfg_grv = {0};
            cfg_grv.reportInterval_us = 20000; // 50 Hz
            cfg_grv.alwaysOnEnabled = true;
            cfg_grv.wakeupEnabled = false;
            int r3 = sh2_setSensorConfig(SH2_GAME_ROTATION_VECTOR, &cfg_grv);
            ESP_LOGI(TAG, "setSensorConfig grv rc=%d", r3);
            // After feature enables, open a longer INT bypass window to avoid missing first responses
            #if CONFIG_GIMBAL_BNO08X_USE_SPI
            sh2_hal_spi_bypass_int_until(((uint32_t)esp_timer_get_time()) + 1000000); // 1 s
            #elif CONFIG_GIMBAL_BNO08X_USE_I2C
            sh2_hal_i2c_bypass_int_until(((uint32_t)esp_timer_get_time()) + 1000000);
            #endif
            // Request GET_FEATURE for verification (bounded by timeouts in sh2.c)
            sh2_SensorConfig_t rcfg;
            if (sh2_getSensorConfig(SH2_GAME_ROTATION_VECTOR, &rcfg) == SH2_OK) {
                ESP_LOGI(TAG, "getFeature GRV: AO=%d WK=%d int=%lu us",
                         (int)rcfg.alwaysOnEnabled, (int)rcfg.wakeupEnabled, (unsigned long)rcfg.reportInterval_us);
            }
            // Do not bypass INT; read only when INT asserts to avoid zero packets
            s_sensors_enabled = true;
            s_enable_us = (uint32_t)esp_timer_get_time();
        }
        uint32_t now_us = (uint32_t)esp_timer_get_time();
        // Health log every ~1s
        static uint32_t last_log_us = 0;
        uint32_t now = now_us;
        if (now - last_log_us > 1000000) {
            int int_lvl = -1;
            if (CONFIG_GIMBAL_BNO08X_INT_GPIO >= 0) int_lvl = gpio_get_level(CONFIG_GIMBAL_BNO08X_INT_GPIO);
            uint32_t age_ms = s_last_evt_us ? (now - s_last_evt_us) / 1000 : 0xFFFFFFFF;
            // Query counts (best-effort) to check feature on/off state
            sh2_Counts_t c_acc = {0}, c_gyro = {0};
            int rc_a = sh2_getCounts(SH2_ACCELEROMETER, &c_acc);
            int rc_g = sh2_getCounts(SH2_GYROSCOPE_CALIBRATED, &c_gyro);
            ESP_LOGI(TAG, "dbg: int=%d ready=%d enabled=%d last_evt_ms=%u acc_on=%lu gyro_on=%lu rc_a=%d rc_g=%d",
                     int_lvl, (int)s_sh2_ready, (int)s_sensors_enabled, (unsigned)age_ms,
                     (unsigned long)c_acc.on, (unsigned long)c_gyro.on, rc_a, rc_g);
            last_log_us = now;
        }
        // Always yield a little to avoid WDT even if INT storms
        vTaskDelay(pdMS_TO_TICKS(1));
    // Avoid starving IDLE/other tasks (handled in the branch delays above)
    }
    vTaskDelete(NULL);
}

esp_err_t imu_bno08x_read(imu_sample_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    // If SH-2 has a parsed sample, use it; otherwise fallback to placeholder
    #if CONFIG_GIMBAL_BNO08X_USE_SPI || CONFIG_GIMBAL_BNO08X_USE_I2C
    out->ax = s_latest.ax; out->ay = s_latest.ay; out->az = s_latest.az;
    out->gx = s_latest.gx; out->gy = s_latest.gy; out->gz = s_latest.gz;
    return ESP_OK;
    #endif
    static bool warned = false;
    if (!warned) {
        ESP_LOGW(TAG, "BNO08x driver is a stub: returning placeholder accel/gyro (enable SH-2/SHTP driver for real data)");
        warned = true;
    }
    out->ax = 0; out->ay = 0; out->az = 1;
    out->gx = 0; out->gy = 0; out->gz = 0;
    return ESP_OK;
}

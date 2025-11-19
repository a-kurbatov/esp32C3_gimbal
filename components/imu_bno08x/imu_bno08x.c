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
    float qw, qx, qy, qz; // GRV quaternion (real, x, y, z)
} s_latest;
static volatile uint32_t s_cnt_acc = 0, s_cnt_gyr = 0, s_cnt_grv = 0;
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
                s_cnt_acc++;
                if (first) { ESP_LOGI(TAG, "ACC evt: ax=%.2f ay=%.2f az=%.2f", s_latest.ax, s_latest.ay, s_latest.az); first = false; }
                s_last_evt_us = (uint32_t)esp_timer_get_time();
                break;
            case SH2_GYROSCOPE_CALIBRATED:
                // SH-2 reports gyro in rad/s; convert to deg/s to match imu_sample_t contract
                const float RAD2DEG = 57.2957795f;
                s_latest.gx = v.un.gyroscope.x * RAD2DEG;
                s_latest.gy = v.un.gyroscope.y * RAD2DEG;
                s_latest.gz = v.un.gyroscope.z * RAD2DEG;
                s_cnt_gyr++;
                if (first) { ESP_LOGI(TAG, "GYR evt: gx=%.2f gy=%.2f gz=%.2f", s_latest.gx, s_latest.gy, s_latest.gz); first = false; }
                s_last_evt_us = (uint32_t)esp_timer_get_time();
                break;
            case SH2_GAME_ROTATION_VECTOR:
                // GRV quaternion: union fields are i,j,k,real. Store as (qw, qx, qy, qz) with qw=real.
                s_latest.qw = v.un.gameRotationVector.real;
                s_latest.qx = v.un.gameRotationVector.i;
                s_latest.qy = v.un.gameRotationVector.j;
                s_latest.qz = v.un.gameRotationVector.k;
                s_cnt_grv++;
                if (first) { ESP_LOGI(TAG, "GRV evt: qw=%.3f qx=%.3f qy=%.3f qz=%.3f", s_latest.qw, s_latest.qx, s_latest.qy, s_latest.qz); first = false; }
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

// Perform device reset and wait for SH2 RESET event (with timeout and fallback)
static void do_reset_and_wait(void)
{
    s_sh2_ready = false;
    (void)sh2_devOn();
    vTaskDelay(pdMS_TO_TICKS(10));
    (void)sh2_devReset();
    uint32_t t0 = (uint32_t)esp_timer_get_time();
    while (!s_sh2_ready && (((uint32_t)esp_timer_get_time()) - t0) < 1500000UL) {
        sh2_service();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (!s_sh2_ready) {
        ESP_LOGW(TAG, "SH2 RESET event not seen; proceeding with fallback ready");
        s_sh2_ready = true;
        s_reset_us = (uint32_t)esp_timer_get_time();
    }
}

// Helper: enable a sensor and confirm it is active with the requested interval
static bool enable_sensor_confirm(sh2_SensorId_t sid, uint32_t interval_us, int retries)
{
    sh2_SensorConfig_t cfg = {0};
    cfg.reportInterval_us = interval_us;
    cfg.alwaysOnEnabled = true;
    cfg.wakeupEnabled = false;

    for (int attempt = 0; attempt < retries; ++attempt) {
        int r_set = sh2_setSensorConfig(sid, &cfg);
        if (r_set != SH2_OK) {
            ESP_LOGW(TAG, "setSensorConfig sid=0x%02X rc=%d (attempt %d)", (unsigned)sid, r_set, attempt+1);
        }
        // Allow device to process
        for (int i = 0; i < 5; ++i) { sh2_service(); vTaskDelay(pdMS_TO_TICKS(10)); }
        // Read-back
        sh2_SensorConfig_t rcfg = {0};
        int r_get = sh2_getSensorConfig(sid, &rcfg);
        if (r_get == SH2_OK) {
            ESP_LOGI(TAG, "verify sid=0x%02X AO=%d WK=%d int=%lu us (target %lu)",
                     (unsigned)sid, (int)rcfg.alwaysOnEnabled, (int)rcfg.wakeupEnabled,
                     (unsigned long)rcfg.reportInterval_us, (unsigned long)interval_us);
            if (rcfg.alwaysOnEnabled && rcfg.reportInterval_us == interval_us) {
                return true;
            }
        } else {
            ESP_LOGW(TAG, "getSensorConfig sid=0x%02X rc=%d", (unsigned)sid, r_get);
        }
    }
    ESP_LOGW(TAG, "sid=0x%02X failed to enable at %lu us after %d retries", (unsigned)sid, (unsigned long)interval_us, retries);
    return false;
}

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
        .clock_speed_hz = CONFIG_GIMBAL_BNO08X_SPI_HZ,
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
    do_reset_and_wait();
    // Open a short bypass window and service to drain initial adverts
    //#if CONFIG_GIMBAL_BNO08X_USE_SPI
    //sh2_hal_spi_bypass_int_until(((uint32_t)esp_timer_get_time()) + 500000); // 500 ms
    //#elif CONFIG_GIMBAL_BNO08X_USE_I2C
    //sh2_hal_i2c_bypass_int_until(((uint32_t)esp_timer_get_time()) + 500000);
    //#endif
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
    static uint32_t s_last_recovery_us = 0;
    while (s_task_running) {
        // Service the stack; if INT asserted, drain a few fragments then yield
        if (CONFIG_GIMBAL_BNO08X_INT_GPIO >= 0 && gpio_get_level(CONFIG_GIMBAL_BNO08X_INT_GPIO) == 0) {
            for (int i = 0; i < 3; ++i) {
                sh2_service();
            }
            // brief yield to avoid starving IDLE/WDT
            vTaskDelay(pdMS_TO_TICKS(2));
        } else {
            // No INT, do a single service and sleep a bit to reduce CPU usage
            sh2_service();
            // During bring-up (not ready yet) or the first second after enabling, poll faster
            uint32_t now_fast = (uint32_t)esp_timer_get_time();
            if (!s_sh2_ready || (s_sensors_enabled && (now_fast - s_enable_us) < 1000000)) {
                vTaskDelay(pdMS_TO_TICKS(3));
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
            //int rc_init = sh2_reinitialize();
            //ESP_LOGI(TAG, "sh2_reinitialize rc=%d", rc_init);
            // Briefly allow polling even without INT to catch short replies
            //#if CONFIG_GIMBAL_BNO08X_USE_SPI
            //sh2_hal_spi_bypass_int_until(((uint32_t)esp_timer_get_time()) + 1000000); // 1 s
            //#elif CONFIG_GIMBAL_BNO08X_USE_I2C
            //sh2_hal_i2c_bypass_int_until(((uint32_t)esp_timer_get_time()) + 1000000);
            //#endif
            // Briefly service to drain any responses
            for (int i = 0; i < 10; ++i) { sh2_service(); vTaskDelay(pdMS_TO_TICKS(1)); }

            // Configure sensors at 200 Hz, confirm and retry
            const uint32_t interval = 5000; // 200 Hz
            bool ok_gyr = enable_sensor_confirm(SH2_GYROSCOPE_CALIBRATED, interval, 6);
            bool ok_acc = enable_sensor_confirm(SH2_ACCELEROMETER,        interval, 6);
            bool ok_grv = enable_sensor_confirm(SH2_GAME_ROTATION_VECTOR, interval, 6);
            ESP_LOGI(TAG, "enable summary: GYR=%d ACC=%d GRV=%d", (int)ok_gyr, (int)ok_acc, (int)ok_grv);

            // After feature enables, open a longer INT bypass window to avoid missing first responses
            //if CONFIG_GIMBAL_BNO08X_USE_SPI
            //sh2_hal_spi_bypass_int_until(((uint32_t)esp_timer_get_time()) + 1000000); // 1 s
            //#elif CONFIG_GIMBAL_BNO08X_USE_I2C
            //sh2_hal_i2c_bypass_int_until(((uint32_t)esp_timer_get_time()) + 1000000);
            //#endif
            // Request GET_FEATURE for verification (bounded by timeouts in sh2.c)
            sh2_SensorConfig_t rcfg;
            if (sh2_getSensorConfig(SH2_GAME_ROTATION_VECTOR, &rcfg) == SH2_OK) {
                ESP_LOGI(TAG, "getFeature GRV: AO=%d WK=%d int=%lu us",
                         (int)rcfg.alwaysOnEnabled, (int)rcfg.wakeupEnabled, (unsigned long)rcfg.reportInterval_us);
            }
            if (sh2_getSensorConfig(SH2_ACCELEROMETER, &rcfg) == SH2_OK) {
                ESP_LOGI(TAG, "getFeature ACC: AO=%d WK=%d int=%lu us",
                         (int)rcfg.alwaysOnEnabled, (int)rcfg.wakeupEnabled, (unsigned long)rcfg.reportInterval_us);
            }
            if (sh2_getSensorConfig(SH2_GYROSCOPE_CALIBRATED, &rcfg) == SH2_OK) {
                ESP_LOGI(TAG, "getFeature GYR: AO=%d WK=%d int=%lu us",
                         (int)rcfg.alwaysOnEnabled, (int)rcfg.wakeupEnabled, (unsigned long)rcfg.reportInterval_us);
            }
            // Do not bypass INT; read only when INT asserts to avoid zero packets
            s_sensors_enabled = true;
            s_enable_us = (uint32_t)esp_timer_get_time();
        }
        uint32_t now_us = (uint32_t)esp_timer_get_time();
        // Field debug log at ~0.1 Hz (every 10s): INT level and age of last IMU event
        static uint32_t last_log_us = 0;
        if (now_us - last_log_us > 10000000UL) { // 10,000,000 us = 10 s
            int int_lvl = -1;
            if (CONFIG_GIMBAL_BNO08X_INT_GPIO >= 0) int_lvl = gpio_get_level(CONFIG_GIMBAL_BNO08X_INT_GPIO);
            uint32_t age_ms = s_last_evt_us ? ((now_us - s_last_evt_us) / 1000) : 0xFFFFFFFF;
            ESP_LOGI(TAG, "dbg: INT=%d age_ms=%u", int_lvl, (unsigned)age_ms);
            last_log_us = now_us;
        }
        // Recovery: if sensors enabled and no events for > 2s, reset and re-enable
        if (s_sensors_enabled) {
            uint32_t age_us = s_last_evt_us ? (now_us - s_last_evt_us) : 0xFFFFFFFF;
            if (age_us > 2000000UL && (now_us - s_last_recovery_us) > 3000000UL) {
                ESP_LOGW(TAG, "No IMU events for %u ms; recovering (reset + re-enable)", (unsigned)(age_us/1000));
                s_sensors_enabled = false;
                s_last_recovery_us = now_us;
                do_reset_and_wait();
                // After reset, loop will hit the enable block below once ready and 700ms elapsed
            }
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
    out->qw = s_latest.qw; out->qx = s_latest.qx; out->qy = s_latest.qy; out->qz = s_latest.qz;
    return ESP_OK;
    #endif
    static bool warned = false;
    if (!warned) {
        ESP_LOGW(TAG, "BNO08x driver is a stub: returning placeholder accel/gyro (enable SH-2/SHTP driver for real data)");
        warned = true;
    }
    out->ax = 0; out->ay = 0; out->az = 1;
    out->gx = 0; out->gy = 0; out->gz = 0;
    out->qw = 1; out->qx = 0; out->qy = 0; out->qz = 0;
    return ESP_OK;
}

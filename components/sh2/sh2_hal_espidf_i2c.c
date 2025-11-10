#include "sh2_hal_espidf_i2c.h"
#include "gimbal_config.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "sh2_hal_i2c";

static sh2_Hal_t s_hal;
static i2c_master_dev_handle_t s_dev = NULL;
static int s_int_gpio = -1;
static int s_wake_gpio = -1;
static int s_rst_gpio = -1;
static volatile uint32_t s_bypass_int_until_us = 0;

static int hal_open(sh2_Hal_t *self) {
    (void)self;
    // Configure optional WAKE/RESET and perform reset cycle
    ESP_LOGI(TAG, "I2C HAL open (INT=%d WAKE=%d RST=%d)", s_int_gpio, s_wake_gpio, s_rst_gpio);
    if (s_wake_gpio >= 0) {
        gpio_config_t io = { .pin_bit_mask = 1ULL << s_wake_gpio, .mode = GPIO_MODE_OUTPUT };
        gpio_config(&io);
        gpio_set_level(s_wake_gpio, 1);
    }
    if (s_rst_gpio >= 0) {
        gpio_config_t io = { .pin_bit_mask = 1ULL << s_rst_gpio, .mode = GPIO_MODE_OUTPUT };
        gpio_config(&io);
        gpio_set_level(s_rst_gpio, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(s_rst_gpio, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
    } else {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return 0;
}

static void hal_close(sh2_Hal_t *self) { (void)self; }

static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
    (void)self;
    if (!s_dev || len < 4) return 0;
    // INT gating
    if (s_int_gpio >= 0) {
        int level = gpio_get_level(s_int_gpio);
        uint32_t now = (uint32_t)esp_timer_get_time();
        bool bypass = (now < s_bypass_int_until_us);
        bool asserted =
#if CONFIG_GIMBAL_BNO08X_INT_ACTIVE_LOW
            (level == 0);
#else
            (level != 0);
#endif
        if (!asserted && !bypass) {
            return 0;
        }
    }
    // Read 4-byte SHTP header
    esp_err_t e = i2c_master_receive(s_dev, pBuffer, 4, 50);
    if (e != ESP_OK) {
        return 0;
    }
    uint16_t lenField = (uint16_t)pBuffer[0] | ((uint16_t)pBuffer[1] << 8);
    uint16_t pkt_len = lenField & 0x7FFF;
    if (pkt_len == 0 || pkt_len > len) return 0;
    size_t payload_len = pkt_len - 4;
    if (payload_len > 0) {
        e = i2c_master_receive(s_dev, pBuffer + 4, payload_len, 50);
        if (e != ESP_OK) {
            return 0;
        }
    }
    if (t_us) *t_us = (uint32_t)esp_timer_get_time();
    return pkt_len;
}

static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    (void)self;
    if (!s_dev || len < 4) return 0;
    esp_err_t e = i2c_master_transmit(s_dev, pBuffer, len, 50);
    if (e != ESP_OK) return 0;
    return (int)len;
}

static uint32_t hal_getTimeUs(sh2_Hal_t *self) {
    (void)self;
    return (uint32_t)esp_timer_get_time();
}

void sh2_hal_i2c_config(i2c_master_dev_handle_t dev, int int_gpio, int wake_gpio, int rst_gpio) {
    s_dev = dev;
    s_int_gpio = int_gpio;
    s_wake_gpio = wake_gpio;
    s_rst_gpio = rst_gpio;
    s_hal.open = hal_open;
    s_hal.close = hal_close;
    s_hal.read = hal_read;
    s_hal.write = hal_write;
    s_hal.getTimeUs = hal_getTimeUs;
}

sh2_Hal_t *sh2_hal_i2c_get(void) { return &s_hal; }

void sh2_hal_i2c_bypass_int_until(uint32_t until_us) { s_bypass_int_until_us = until_us; }

#include "sh2_hal_espidf_spi.h"
#include "gimbal_config.h"
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "sh2_hal_spi";
static spi_device_handle_t s_dev = NULL;
static int s_int_gpio = -1;
static int s_wake_gpio = -1;
static int s_rst_gpio = -1;
static sh2_Hal_t s_hal;
static uint32_t s_rx_count = 0;
static uint32_t s_tx_count = 0;
static int s_last_int = -1;
static uint32_t s_int_log_count = 0;
static uint8_t s_dummy[SH2_HAL_DMA_SIZE];
static uint32_t s_trace_count = 0;
static volatile uint32_t s_bypass_int_until_us = 0;

// Briefly assert WAKE to ensure the device is awake before command writes
// Pulse WAKE pin low to wake the BNO08x from sleep
static void hal_wake(void) {
    if (s_wake_gpio < 0) return;

    // Pulse WAKE (PS0) low to wake device
    gpio_set_level(s_wake_gpio, 0);

    // Wait 500us (datasheet twk is ~150us)
    esp_rom_delay_us(500);

    // Return WAKE high (idle state)
    gpio_set_level(s_wake_gpio, 1);

    // Give the device time to assert H_INTN.
    // The hal_read() function will see this assertion.
    vTaskDelay(pdMS_TO_TICKS(1));
}

static int hal_open(sh2_Hal_t *self) {
    (void)self;
    // Configure optional WAKE/RESET and perform reset cycle
    if (s_wake_gpio >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << s_wake_gpio,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);
        // Set WAKE (PS0) high and leave it high, as required by datasheet
        // for SPI mode selection during reset.
        gpio_set_level(s_wake_gpio, 1);
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (s_rst_gpio >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << s_rst_gpio,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);
        // Active low reset pulse
        gpio_set_level(s_rst_gpio, 0);
        vTaskDelay(pdMS_TO_TICKS(5));
        gpio_set_level(s_rst_gpio, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
    } else {
        // No reset control; small delay to allow device ready
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return 0;
}

static void hal_close(sh2_Hal_t *self) {
    (void)self;
}
static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
    (void)self;
    if (!s_dev || len < 4) return 0;
    // Only read when INT indicates data ready (polarity configurable)
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
            if (s_last_int != level) {
                s_last_int = level;
                if (s_int_log_count < 10) {
                    ESP_LOGI(TAG, "INT level=%d (polarity %s)", level,
#if CONFIG_GIMBAL_BNO08X_INT_ACTIVE_LOW
                    "active-low"
#else
                    "active-high"
#endif
                    );
                    s_int_log_count++;
                }
            }
            return 0;
        }
        if (s_last_int != level) {
            s_last_int = level;
            if (s_int_log_count < 10) {
                ESP_LOGI(TAG, "INT asserted (level=%d)", level);
                s_int_log_count++;
            }
        }
    }
   
    // Acquire bus to guarantee CS keep-active sequence is not interleaved
    esp_err_t e = spi_device_acquire_bus(s_dev, portMAX_DELAY);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "spi acquire bus err=%d", e);
        return 0;
    }

    // Read 4-byte header (CS kept active for payload phase)
    spi_transaction_t th;
    memset(&th, 0, sizeof(th));
    // Keep CS active between header and payload to ensure we read the
    // payload of the same transfer (BNO08x expects continuous read)
    th.flags = SPI_TRANS_CS_KEEP_ACTIVE;
    th.length = 8 * 4; // 4 bytes
    th.rxlength = th.length;
    th.tx_buffer = s_dummy;
    th.rx_buffer = pBuffer; // Read directly into the provided buffer
    // Use queued, blocking transmit
    e = spi_device_transmit(s_dev, &th);
    if (e != ESP_OK) {
        static uint8_t err_cnt = 0;
        if (err_cnt < 5) {
            ESP_LOGE(TAG, "spi rx header err=%d", e);
            err_cnt++;
        }
        spi_device_release_bus(s_dev);
        return 0;
    }
    uint16_t lenField = (uint16_t)pBuffer[0] | ((uint16_t)pBuffer[1] << 8);
    bool continuation = (lenField & 0x8000) != 0;
    uint16_t pkt_len = lenField & 0x7FFF; // mask off continuation bit per SHTP
    if (s_rx_count < 10) {
        ESP_LOGI(TAG, "SHTP hdr lenField=0x%04X len=%u cont=%d ch=%u seq=%u raw=%02X %02X %02X %02X",
                 (unsigned)lenField, (unsigned)pkt_len, (int)continuation, pBuffer[2], pBuffer[3], pBuffer[0], pBuffer[1], pBuffer[2], pBuffer[3]);
    }
    // If header indicates no data or invalid, bail out
    if (pkt_len == 0 || pkt_len == 0xFFFF) { spi_device_release_bus(s_dev); return 0; }
    if (pkt_len < 4 || pkt_len > len) { spi_device_release_bus(s_dev); return 0; }
    size_t payload_len = pkt_len - 4;
    // Per common SHTP-over-SPI implementations (SparkFun/Adafruit), round total transfer to 4-byte boundary
    // Compute how many pad bytes are needed after payload to reach a 4-byte multiple
    uint32_t rem4 = (pkt_len & 0x3U);
    uint32_t pad_bytes = (rem4 == 0) ? 0 : (4 - rem4);
    // payload logging omitted to reduce noise
    if (payload_len) {
        // Datasheet allows small CS setup/hold; add ~1us margin between header and payload with CS kept active
        esp_rom_delay_us(1);
        // Read payload in chunks to respect host max transfer size
        const size_t MAX_CHUNK = 1000; // safe under typical max_transfer_sz=1024
        size_t remaining = payload_len;
        size_t offset = 0;
        while (remaining > 0) {
            size_t chunk = remaining > MAX_CHUNK ? MAX_CHUNK : remaining;
            if (chunk > sizeof(s_dummy)) chunk = sizeof(s_dummy);
            spi_transaction_t tp;
            memset(&tp, 0, sizeof(tp));
            // Keep CS active across all but last payload chunk and any pad bytes
            bool more_after = (remaining > chunk) || (pad_bytes > 0);
            tp.flags = more_after ? SPI_TRANS_CS_KEEP_ACTIVE : 0;
            tp.length = 8 * chunk;
            tp.rxlength = tp.length;
            tp.tx_buffer = s_dummy;
            tp.rx_buffer = pBuffer + 4 + offset;
            e = spi_device_transmit(s_dev, &tp);
            if (e != ESP_OK) { spi_device_release_bus(s_dev); return 0; }
            remaining -= chunk;
            offset += chunk;
        }
        // Read pad bytes if any, keeping CS active only until last pad
        for (uint32_t i = 0; i < pad_bytes; ++i) {
            uint8_t pad;
            spi_transaction_t tpad;
            memset(&tpad, 0, sizeof(tpad));
            // Keep CS active for all but last pad byte
            tpad.flags = (i + 1 < pad_bytes) ? SPI_TRANS_CS_KEEP_ACTIVE : 0;
            tpad.length = 8; // 1 byte each
            tpad.rxlength = 8;
            tpad.tx_buffer = s_dummy;
            tpad.rx_buffer = &pad;
            e = spi_device_transmit(s_dev, &tpad);
            if (e != ESP_OK) { spi_device_release_bus(s_dev); return 0; }
        }
    }
    // Optional SPI trace: dump first few header+payload packets
#if CONFIG_GIMBAL_BNO08X_SPI_TRACE
    if (s_trace_count < 10) {
        // Dump header
        char hex[3 * 4 + 1];
        for (int i = 0; i < 4; i++) {
            sprintf(&hex[i * 3], "%02X ", pBuffer[i]);
        }
        hex[3 * 4] = '\0';
        ESP_LOGI(TAG, "TRACE HDR: %s", hex);
        // Dump payload (cap at 32 bytes)
        int dump_len = (int)pkt_len - 4;
        if (dump_len > 32) dump_len = 32;
        if (dump_len > 0) {
            int bytes = dump_len;
            char phex[3 * 32 + 1];
            for (int i = 0; i < bytes; i++) {
                sprintf(&phex[i * 3], "%02X ", pBuffer[4 + i]);
            }
            phex[3 * bytes] = '\0';
            ESP_LOGI(TAG, "TRACE PAY: %s", phex);
        }
        s_trace_count++;
    }
#endif
    if (t_us) *t_us = (uint32_t)esp_timer_get_time();
    s_rx_count++;
    spi_device_release_bus(s_dev);
    return pkt_len;
}
static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    (void)self;

    // Wake the device before every command write
    hal_wake();

    if (!s_dev || len < 4) return 0;
    // Acquire bus to serialize with any pending read sequence
    esp_err_t e = spi_device_acquire_bus(s_dev, portMAX_DELAY);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "spi acquire bus (write) err=%d", e);
        return 0;
    }
    uint32_t rem4 = (len & 0x3U);
    uint32_t pad_bytes = (rem4 == 0) ? 0 : (4 - rem4);
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.flags = (pad_bytes > 0) ? SPI_TRANS_CS_KEEP_ACTIVE : 0;
    t.length = 8 * len;
    t.tx_buffer = pBuffer;
    t.rx_buffer = NULL;
    e = spi_device_transmit(s_dev, &t);
    if (e == ESP_OK && pad_bytes > 0) {
        // Send pad bytes (0x00) to round to 4-byte boundary
        for (uint32_t i = 0; i < pad_bytes; ++i) {
            uint8_t pad = 0x00;
            spi_transaction_t tpad;
            memset(&tpad, 0, sizeof(tpad));
            tpad.length = 8;
            tpad.tx_buffer = &pad;
            tpad.rx_buffer = NULL;
            e = spi_device_transmit(s_dev, &tpad);
            if (e != ESP_OK) break;
        }
    }
    spi_device_release_bus(s_dev);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "spi tx err=%d", e);
        return 0;
    }
    if (s_tx_count < 5) {
        ESP_LOGI(TAG, "SHTP tx len=%u", (unsigned)len);
    }
    s_tx_count++;
    return (int)len;
}
static uint32_t hal_getTimeUs(sh2_Hal_t *self) {
    (void)self;
    return (uint32_t)esp_timer_get_time();
}

void sh2_hal_spi_config(spi_device_handle_t dev, int int_gpio, int wake_gpio, int rst_gpio) {
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

sh2_Hal_t *sh2_hal_spi_get(void) {
    return &s_hal;
}

void sh2_hal_spi_bypass_int_until(uint32_t until_us) {
    s_bypass_int_until_us = until_us;
}

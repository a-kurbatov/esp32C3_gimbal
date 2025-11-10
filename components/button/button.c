#include "button.h"
#include "driver/gpio.h"
#include "esp_timer.h"

static struct {
    int gpio;
    int64_t long_us;
    button_longpress_cb_t on_long;
    void *ctx;

    bool last_level;
    int64_t press_start;
} s_btn;

esp_err_t button_init(const button_cfg_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    s_btn.gpio = cfg->gpio;
    s_btn.long_us = (int64_t)cfg->long_ms * 1000;
    s_btn.on_long = cfg->on_long;
    s_btn.ctx = cfg->user_ctx;
    s_btn.press_start = 0;

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << s_btn.gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    s_btn.last_level = gpio_get_level(s_btn.gpio);
    return ESP_OK;
}

void button_poll(void)
{
    bool level = gpio_get_level(s_btn.gpio); // active-low
    int64_t now = esp_timer_get_time();
    if (!level) { // pressed
        if (s_btn.press_start == 0) s_btn.press_start = now;
        if (now - s_btn.press_start >= s_btn.long_us && s_btn.on_long) {
            s_btn.on_long(s_btn.ctx);
            s_btn.on_long = NULL; // fire once
        }
    } else {
        s_btn.press_start = 0;
    }
    s_btn.last_level = level;
}

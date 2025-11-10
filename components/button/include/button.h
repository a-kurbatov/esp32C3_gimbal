#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef void (*button_longpress_cb_t)(void *user_ctx);

typedef struct {
    int gpio;
    int long_ms;          // duration for long press
    button_longpress_cb_t on_long;
    void *user_ctx;
} button_cfg_t;

esp_err_t button_init(const button_cfg_t *cfg);
void       button_poll(void); // call periodically (e.g., in main loop)

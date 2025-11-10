#pragma once
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    int gpio;
    int freq_hz;         // typically 50 Hz
    int min_us;          // e.g., 500
    int max_us;          // e.g., 2500
} servo_pwm_cfg_t;

esp_err_t servo_pwm_init(const servo_pwm_cfg_t *cfg);

// Write pulse in microseconds (will be clamped to [min_us, max_us])
esp_err_t servo_pwm_write_us(int pulse_us);

// Helper: write angle mapped over [min_angle_deg, max_angle_deg]
esp_err_t servo_pwm_write_angle(float angle_deg, float min_angle_deg, float max_angle_deg);

// Get last pulse written (us)
int servo_pwm_get_last_us(void);

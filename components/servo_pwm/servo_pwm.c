#include "servo_pwm.h"
#include "driver/ledc.h"
#include "esp_check.h"

static const char *TAG = "servo_pwm";

static struct {
    servo_pwm_cfg_t cfg;
    ledc_channel_t channel;
    ledc_timer_t timer;
    uint32_t max_duty;
    int last_us;
} s_servo;

esp_err_t servo_pwm_init(const servo_pwm_cfg_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg, ESP_ERR_INVALID_ARG, TAG, "cfg is null");
    s_servo.cfg = *cfg;
    s_servo.channel = LEDC_CHANNEL_0;
    s_servo.timer = LEDC_TIMER_0;

    const ledc_timer_config_t tcfg = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_14_BIT,
        .timer_num        = s_servo.timer,
        .freq_hz          = cfg->freq_hz,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&tcfg), TAG, "timer config failed");

    const ledc_channel_config_t ccfg = {
        .gpio_num       = cfg->gpio,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = s_servo.channel,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = s_servo.timer,
        .duty           = 0,
        .hpoint         = 0,
        .flags.output_invert = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ccfg), TAG, "channel config failed");

    s_servo.max_duty = (1u << LEDC_TIMER_14_BIT) - 1;
    s_servo.last_us = 0;
    return ESP_OK;
}

esp_err_t servo_pwm_write_us(int pulse_us)
{
    if (pulse_us < s_servo.cfg.min_us) pulse_us = s_servo.cfg.min_us;
    if (pulse_us > s_servo.cfg.max_us) pulse_us = s_servo.cfg.max_us;

    const uint32_t period_us = 1000000UL / s_servo.cfg.freq_hz; // e.g., 20000us
    uint32_t duty = (uint32_t)((((uint64_t)pulse_us) * s_servo.max_duty) / period_us);
    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, s_servo.channel, duty);
    if (err != ESP_OK) return err;
    s_servo.last_us = pulse_us;
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, s_servo.channel);
}

esp_err_t servo_pwm_write_angle(float angle_deg, float min_angle_deg, float max_angle_deg)
{
    if (max_angle_deg <= min_angle_deg) return ESP_ERR_INVALID_ARG;
    float t = (angle_deg - min_angle_deg) / (max_angle_deg - min_angle_deg);
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    int pulse = s_servo.cfg.min_us + (int)((s_servo.cfg.max_us - s_servo.cfg.min_us) * t);
    return servo_pwm_write_us(pulse);
}

int servo_pwm_get_last_us(void)
{
    return s_servo.last_us;
}

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "imu/imu.h"
#include "imu/imu_types.h"
#include "servo_pwm.h"
#include "button.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"

static const char *TAG = "gimbal_main";

// NVS keys
#define NVS_NS      "gimbal"
#define NVS_KEY_OFF "h_offset"

static float s_horizon_offset_deg = 0.0f;

static void save_offset_nvs(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NS, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_blob(nvs, NVS_KEY_OFF, &s_horizon_offset_deg, sizeof(s_horizon_offset_deg));
        nvs_commit(nvs);
        nvs_close(nvs);
        ESP_LOGI(TAG, "Saved horizon offset: %.2f deg", s_horizon_offset_deg);
    }
}

static void load_offset_nvs(void)
{
    nvs_handle_t nvs;
    size_t len = sizeof(s_horizon_offset_deg);
    if (nvs_open(NVS_NS, NVS_READONLY, &nvs) == ESP_OK) {
        if (nvs_get_blob(nvs, NVS_KEY_OFF, &s_horizon_offset_deg, &len) == ESP_OK && len == sizeof(float)) {
            ESP_LOGI(TAG, "Loaded horizon offset: %.2f deg", s_horizon_offset_deg);
        }
        nvs_close(nvs);
    }
}

static void on_longpress(void *ctx)
{
    // Capture current platform tilt as new horizon offset (ctx holds last tilt value)
    float *last_tilt = (float *)ctx;
    s_horizon_offset_deg = *last_tilt;
    save_offset_nvs();
}

typedef struct {
    float kp, ki, kd;
    float i_term;
    float last_err;
    float d_filtered;
    float i_max; // clamp for integral term (anti-windup)
} pid_t;

static void pid_init(pid_t *p, float kp, float ki, float kd, float i_max) {
    p->kp = kp; p->ki = ki; p->kd = kd; p->i_term = 0; p->last_err = 0; p->d_filtered = 0; p->i_max = i_max;
}

// PID update with:
// - small deadband to avoid jitter
// - clamped integral (anti-windup)
// - low-pass on derivative to reduce noise
static float pid_update(pid_t *p, float err, float dt, float max_output) {
    if (dt <= 0.0f) return 0.0f;

    // Deadband: ignore tiny errors (prevents constant small corrections)
    const float deadband_deg = 0.15f; // degrees
    if (fabsf(err) < deadband_deg) {
        err = 0.0f;
    }

    // Integral with simple clamping (anti-windup)
    if (p->ki != 0.0f) {
        p->i_term += err * dt;
        if (p->i_term > p->i_max) p->i_term = p->i_max;
        if (p->i_term < -p->i_max) p->i_term = -p->i_max;
    }

    // Derivative (filtered) to avoid noisy derivative term
    float raw_d = (err - p->last_err) / dt;
    const float d_alpha = 0.85f; // higher = more smoothing
    p->d_filtered = d_alpha * p->d_filtered + (1.0f - d_alpha) * raw_d;
    p->last_err = err;

    float out = p->kp * err + p->ki * p->i_term + p->kd * p->d_filtered;

    // Clamp to max_output (antenna limits) to keep reasonable
    if (out > max_output) out = max_output;
    if (out < -max_output) out = -max_output;

    return out;
}

static float radians_to_degrees(float r) { return r * (180.0f / (float)M_PI); }

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    // Servo
    servo_pwm_cfg_t scfg = {
        .gpio = CONFIG_GIMBAL_SERVO_GPIO,
        .freq_hz = CONFIG_GIMBAL_SERVO_FREQ_HZ,
        .min_us = CONFIG_GIMBAL_SERVO_MIN_US,
        .max_us = CONFIG_GIMBAL_SERVO_MAX_US,
    };
    ESP_ERROR_CHECK(servo_pwm_init(&scfg));
    servo_pwm_write_us((CONFIG_GIMBAL_SERVO_MIN_US + CONFIG_GIMBAL_SERVO_MAX_US)/2);

#if CONFIG_GIMBAL_UART_RVC_SNIFFER
    // Put SPI pins (SCLK/MOSI/CS) into Hi-Z inputs; leave INT as input
    int spi_pins[] = { CONFIG_GIMBAL_SPI_SCLK_GPIO, CONFIG_GIMBAL_SPI_MOSI_GPIO, CONFIG_GIMBAL_SPI_CS_GPIO };
    for (size_t i = 0; i < sizeof(spi_pins)/sizeof(spi_pins[0]); ++i) {
        if (spi_pins[i] >= 0) {
            gpio_config_t io = {
                .pin_bit_mask = 1ULL << spi_pins[i],
                .mode = GPIO_MODE_INPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE,
            };
            gpio_config(&io);
        }
    }
    if (CONFIG_GIMBAL_BNO08X_INT_GPIO >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << CONFIG_GIMBAL_BNO08X_INT_GPIO,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);
    }

    // Start UART-RVC sniffer (RX only)
    uart_config_t ucfg = {
        .baud_rate = CONFIG_GIMBAL_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_GIMBAL_UART_PORT, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CONFIG_GIMBAL_UART_PORT, &ucfg));
    int tx_io = (CONFIG_GIMBAL_UART_TX_GPIO < 0) ? UART_PIN_NO_CHANGE : CONFIG_GIMBAL_UART_TX_GPIO;
    ESP_ERROR_CHECK(uart_set_pin(CONFIG_GIMBAL_UART_PORT, tx_io, CONFIG_GIMBAL_UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGW(TAG, "UART-RVC sniffer enabled: UART%d RX=GPIO%d baud=%d. SPI/IMU disabled.",
             CONFIG_GIMBAL_UART_PORT, CONFIG_GIMBAL_UART_RX_GPIO, CONFIG_GIMBAL_UART_BAUD);
    ESP_LOGW(TAG, "Straps for RVC: PS1=HIGH, PS0=LOW. Wire BNO TX -> ESP32 RX. Power-cycle after strap change.");

    uint8_t buf[256];
    while (1) {
        int n = uart_read_bytes(CONFIG_GIMBAL_UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(100));
        if (n > 0) {
            // Print a compact hex dump
            char line[16*3 + 32];
            int idx = 0;
            int printed = 0;
            for (int i = 0; i < n; ++i) {
                if (idx == 0) {
                    printed = snprintf(line, sizeof(line), "rvc %3d: ", n);
                    idx = printed;
                }
                if (idx + 3 >= (int)sizeof(line)) {
                    ESP_LOGI(TAG, "%s", line);
                    idx = 0;
                    i--; // reprocess this byte on next line
                    continue;
                }
                idx += snprintf(line + idx, sizeof(line) - idx, "%02X ", buf[i]);
            }
            if (idx > 0) {
                ESP_LOGI(TAG, "%s", line);
            }
        }
    }
    // Not reached
#endif

    // IMU
    imu_driver_t *imu = imu_get();
    ESP_ERROR_CHECK(imu->v->init());

    // Load calibration
    load_offset_nvs();

    // Button
    static float last_tilt_deg = 0.0f;
    button_cfg_t bcfg = {
        .gpio = CONFIG_GIMBAL_BUTTON_GPIO,
        .long_ms = CONFIG_GIMBAL_BUTTON_LONG_MS,
        .on_long = on_longpress,
        .user_ctx = &last_tilt_deg,
    };
    ESP_ERROR_CHECK(button_init(&bcfg));

    const float alpha = CONFIG_GIMBAL_ALPHA / 1000.0f;
    pid_t pid; pid_init(&pid,
                        CONFIG_GIMBAL_PID_KP / 1000.0f,
                        CONFIG_GIMBAL_PID_KI / 1000.0f,
                        CONFIG_GIMBAL_PID_KD / 1000.0f,
                        CONFIG_GIMBAL_ANTENNA_LIMIT_DEG /* i_max */);

    const float dt = 1.0f / CONFIG_GIMBAL_LOOP_HZ; // seconds
    const int loop_delay_ms = (int)(dt * 1000.0f);

    // GRV-only stabilization (most stable): keep a smoothed GRV tilt and use it for control
    float tilt_grv_filt = 0.0f; // deg
    float tilt_used = 0.0f;     // deg

    int log_count = 0;
    const int log_every = (int)(2.0f / dt); // 2 Hz
    if (log_every < 1) {
        // Fallback to every loop if dt is large
        // (shouldn't happen with 10 Hz)
    }

    while (1) {
        button_poll();

        imu_sample_t s = {0};
        if (imu->v->read(&s) == ESP_OK) {
            // GRV-only mode: compute pitch from quaternion
            // (Accel/Gyro paths are computed no longer; GRV handles fusion)

            // Fallback: if raw accel/gyro stream is missing (all zeros), derive pitch from GRV for ACC path
            bool raw_missing = (s.ax == 0.0f && s.ay == 0.0f && s.az == 0.0f &&
                                s.gx == 0.0f && s.gy == 0.0f && s.gz == 0.0f);
            float qn = s.qw*s.qw + s.qx*s.qx + s.qy*s.qy + s.qz*s.qz;
            float tilt_grv = 0.0f;
            if (qn > 0.25f) {
                float qw = s.qw, qx = s.qx, qy = s.qy, qz = s.qz;
                float gx_b = 2.0f*(qx*qz - qw*qy);
                float gy_b = 2.0f*(qw*qx + qy*qz);
                float gz_b = qw*qw - qx*qx - qy*qy + qz*qz;
                tilt_grv = radians_to_degrees(atan2f(-gx_b, sqrtf(gy_b*gy_b + gz_b*gz_b)));
            }
            // Light LPF on GRV tilt using same alpha (acts as smoothing only)
            float tilt_pred_grv = tilt_grv_filt; // no gyro integration on GRV
            tilt_grv_filt = alpha * tilt_pred_grv + (1 - alpha) * tilt_grv;
            // GRV-only: use GRV tilt directly
            tilt_used = tilt_grv_filt;
            last_tilt_deg = tilt_used;

            // Apply stored horizon offset (invert to counteract platform tilt)
            float platform_tilt = tilt_used - s_horizon_offset_deg;
            float target_antenna = 0.0f; // keep level
            float error = target_antenna - platform_tilt;

            // Clamp to antenna ±limit
            float ant_lim = CONFIG_GIMBAL_ANTENNA_LIMIT_DEG;
            if (error > ant_lim) error = ant_lim;
            if (error < -ant_lim) error = -ant_lim;

            // PID (with improved anti-windup, derivative filtering and deadband)
            // Reduce sensitivity: require ~6x larger IMU tilt to reach same controller output
            const float err_scale = 1.0f / 6.0f;
            float u = pid_update(&pid, error * err_scale, dt, ant_lim);

            // Convert to servo angle via mechanical ratio and clamp to servo ±limit
            float servo_lim = CONFIG_GIMBAL_SERVO_LIMIT_DEG;
            float servo_angle = u * (CONFIG_GIMBAL_MECH_RATIO / 1000.0f); // deg
            if (servo_angle > servo_lim) servo_angle = servo_lim;
            if (servo_angle < -servo_lim) servo_angle = -servo_lim;

            // Write servo (map -servo_lim..+servo_lim)
            servo_pwm_write_angle(servo_angle, -servo_lim, +servo_lim);

            if (++log_count >= log_every) {
                log_count = 0;
                int pwm = servo_pwm_get_last_us();
                ESP_LOGI(TAG, "imu ax=%.2f ay=%.2f az=%.2f gx=%.2f gy=%.2f gz=%.2f | tilt=%.2f src=GRV off=%.2f err=%.2f servo=%.2f pwm=%dus | grv qw=%.3f qx=%.3f qy=%.3f qz=%.3f",
                         s.ax, s.ay, s.az, s.gx, s.gy, s.gz,
                         tilt_used, s_horizon_offset_deg, error, servo_angle, pwm,
                         s.qw, s.qx, s.qy, s.qz);
            }
        } else {
            // No IMU data; hold neutral
            servo_pwm_write_us((CONFIG_GIMBAL_SERVO_MIN_US + CONFIG_GIMBAL_SERVO_MAX_US)/2);
        }

        vTaskDelay(pdMS_TO_TICKS(loop_delay_ms));
    }
}

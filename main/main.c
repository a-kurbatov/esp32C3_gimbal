#include <stdio.h>
#include <math.h>
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
} pid_t;

static void pid_init(pid_t *p, float kp, float ki, float kd) {
    p->kp = kp; p->ki = ki; p->kd = kd; p->i_term = 0; p->last_err = 0;
}

static float pid_update(pid_t *p, float err, float dt) {
    p->i_term += err * dt;
    float d = (err - p->last_err) / dt;
    p->last_err = err;
    return p->kp * err + p->ki * p->i_term + p->kd * d;
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
                        CONFIG_GIMBAL_PID_KD / 1000.0f);

    const float dt = 1.0f / CONFIG_GIMBAL_LOOP_HZ; // seconds
    const int loop_delay_ms = (int)(dt * 1000.0f);

    float tilt_filt = 0; // deg

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
            // Estimate pitch from accelerometer (assuming +Z up, X forward)
            float acc_pitch_deg = radians_to_degrees(atan2f(-s.ax, sqrtf(s.ay*s.ay + s.az*s.az)));
            float gyro_pitch_degps = s.gx; // depends on mounting; adjust axis if needed

            // Complementary filter
            float tilt_pred = tilt_filt + gyro_pitch_degps * dt;
            tilt_filt = alpha * tilt_pred + (1 - alpha) * acc_pitch_deg;
            last_tilt_deg = tilt_filt;

            // Apply stored horizon offset (invert to counteract platform tilt)
            float platform_tilt = tilt_filt - s_horizon_offset_deg;
            float target_antenna = 0.0f; // keep level
            float error = target_antenna - platform_tilt;

            // Clamp to antenna ±limit
            float ant_lim = CONFIG_GIMBAL_ANTENNA_LIMIT_DEG;
            if (error > ant_lim) error = ant_lim;
            if (error < -ant_lim) error = -ant_lim;

            // PID
            float u = pid_update(&pid, error, dt);

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
                ESP_LOGI(TAG, "imu ax=%.2f ay=%.2f az=%.2f gx=%.2f gy=%.2f gz=%.2f | tilt=%.2f off=%.2f err=%.2f servo=%.2f pwm=%dus",
                         s.ax, s.ay, s.az, s.gx, s.gy, s.gz,
                         tilt_filt, s_horizon_offset_deg, error, servo_angle, pwm);
            }
        } else {
            // No IMU data; hold neutral
            servo_pwm_write_us((CONFIG_GIMBAL_SERVO_MIN_US + CONFIG_GIMBAL_SERVO_MAX_US)/2);
        }

        vTaskDelay(pdMS_TO_TICKS(loop_delay_ms));
    }
}

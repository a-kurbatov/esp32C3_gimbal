#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "imu/imu.h"
#include "imu/imu_types.h"
#if CONFIG_GIMBAL_IMU_SELECT_MSP_2AXIS
#include "imu_msp.h"
#endif
#include "servo_pwm.h"
#include "button.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"

static const char *TAG = "gimbal_main";

// NVS keys
#define NVS_NS      "gimbal"
#define NVS_KEY_OFF "h_offset"
#define NVS_KEY_YAW_CTR "yaw_ctr"

static float s_horizon_offset_deg = 0.0f;
static float s_yaw_center_deg = 0.0f; // heading at which yaw servo centers

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

static void save_yaw_center_nvs(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NS, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_blob(nvs, NVS_KEY_YAW_CTR, &s_yaw_center_deg, sizeof(s_yaw_center_deg));
        nvs_commit(nvs);
        nvs_close(nvs);
        ESP_LOGI(TAG, "Saved yaw center: %.1f deg", s_yaw_center_deg);
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

static void load_yaw_center_nvs(void)
{
    nvs_handle_t nvs;
    size_t len = sizeof(s_yaw_center_deg);
    if (nvs_open(NVS_NS, NVS_READONLY, &nvs) == ESP_OK) {
        if (nvs_get_blob(nvs, NVS_KEY_YAW_CTR, &s_yaw_center_deg, &len) == ESP_OK && len == sizeof(float)) {
            ESP_LOGI(TAG, "Loaded yaw center: %.1f deg", s_yaw_center_deg);
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

#if CONFIG_GIMBAL_IMU_SELECT_MSP_2AXIS
// TODO: Axis conventions: verify MSP roll/pitch/yaw signs and rotation order on your FC.
// Assumptions (to be confirmed):
// - Yaw positive CCW (Z-up), Pitch positive nose-up, Roll positive right-wing-down.
// - Body rotation R = Rz(yaw) * Ry(pitch) * Rx(roll).
// - Gimbal kinematics: yaw axis first (Z), then pitch about rotated Y'.
// These affect sign flips in the decomposition below.

typedef struct {
    float yaw_deg;
    float pitch_deg;
    float roll_deg;
} last_att_t;

static float s_target_azimuth_deg = 0.0f; // default target: heading 0 maps to servo center
static inline float deg2rad(float d) { return d * ((float)M_PI / 180.0f); }
static inline float rad2deg(float r) { return r * (180.0f / (float)M_PI); }
static inline float wrap_deg180(float a) { while (a > 180.0f) a -= 360.0f; while (a <= -180.0f) a += 360.0f; return a; }

static void on_longpress_msp(void *ctx)
{
    last_att_t *la = (last_att_t *)ctx;
    // Capture current heading as new yaw center so servo centers at this yaw
    s_yaw_center_deg = la->yaw_deg;
    save_yaw_center_nvs();
    ESP_LOGI(TAG, "Captured yaw center: %.1f deg", s_yaw_center_deg);
    // TODO: persist to NVS for reboot retention
}

// Minimal second servo (yaw) on a separate LEDC channel using same timer as servo_pwm
static struct {
    ledc_channel_t ch;
    ledc_timer_t timer;
    int gpio;
    uint32_t max_duty;
    int min_us, max_us;
} s_yaw_servo;

static esp_err_t yaw_servo_init(int gpio, int freq_hz, int min_us, int max_us)
{
    s_yaw_servo.gpio = gpio;
    s_yaw_servo.ch = LEDC_CHANNEL_1; // pitch uses channel 0 inside servo_pwm
    s_yaw_servo.timer = LEDC_TIMER_0; // reuse same timer config
    s_yaw_servo.min_us = min_us; s_yaw_servo.max_us = max_us;
    const ledc_channel_config_t ccfg = {
        .gpio_num = gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = s_yaw_servo.ch,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = s_yaw_servo.timer,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 0,
    };
    esp_err_t err = ledc_channel_config(&ccfg);
    s_yaw_servo.max_duty = (1u << LEDC_TIMER_14_BIT) - 1; // matches servo_pwm
    return err;
}

static void yaw_servo_write_angle(float angle_deg, float min_angle_deg, float max_angle_deg)
{
    if (max_angle_deg <= min_angle_deg) return;
    float t = (angle_deg - min_angle_deg) / (max_angle_deg - min_angle_deg);
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    int pulse = s_yaw_servo.min_us + (int)((s_yaw_servo.max_us - s_yaw_servo.min_us) * t);
    const uint32_t period_us = 1000000UL / CONFIG_GIMBAL_SERVO_FREQ_HZ;
    uint32_t duty = (uint32_t)((((uint64_t)pulse) * s_yaw_servo.max_duty) / period_us);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, s_yaw_servo.ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, s_yaw_servo.ch);
}
#endif // CONFIG_GIMBAL_IMU_SELECT_MSP_2AXIS

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

    // IMU/Source selection
#if CONFIG_GIMBAL_IMU_SELECT_MSP_2AXIS
    // MSP 2-axis source
    ESP_ERROR_CHECK(imu_msp_init());
#else
    // Legacy single-axis IMU
    imu_driver_t *imu = imu_get();
    ESP_ERROR_CHECK(imu->v->init());
#endif

    // Load calibration
    load_offset_nvs();
    load_yaw_center_nvs();

    // Button
    static float last_tilt_deg = 0.0f;
#if CONFIG_GIMBAL_IMU_SELECT_MSP_2AXIS
    static last_att_t last_att = {0};
    button_cfg_t bcfg = {
        .gpio = CONFIG_GIMBAL_BUTTON_GPIO,
        .long_ms = CONFIG_GIMBAL_BUTTON_LONG_MS,
        .on_long = on_longpress_msp,
        .user_ctx = &last_att,
    };
#else
    button_cfg_t bcfg = {
        .gpio = CONFIG_GIMBAL_BUTTON_GPIO,
        .long_ms = CONFIG_GIMBAL_BUTTON_LONG_MS,
        .on_long = on_longpress,
        .user_ctx = &last_tilt_deg,
    };
#endif
    ESP_ERROR_CHECK(button_init(&bcfg));

    // Complementary filter and PID setup
    const float alpha = CONFIG_GIMBAL_ALPHA / 1000.0f;
    const float kp = CONFIG_GIMBAL_PID_KP / 1000.0f;
    const float ki = CONFIG_GIMBAL_PID_KI / 1000.0f;
    const float kd_rate = CONFIG_GIMBAL_PID_KD / 1000.0f; // derivative on measurement (gyro) gain
    pid_t pid; pid_init(&pid, kp, ki, 0.0f); // kd=0 inside; D handled from gyro directly

    const float dt = 1.0f / CONFIG_GIMBAL_LOOP_HZ; // seconds
    const int loop_delay_ms = (int)(dt * 1000.0f);

    float tilt_filt = 0; // deg

    // Noise reduction helpers (legacy IMU path)
    const float deadband_deg = 0.3f;           // ignore tiny errors (adjust 0.2..0.5)
    const float d_cut_hz = 10.0f;              // derivative (gyro) low-pass cutoff (Hz)
    const float out_tau_s = 0.08f;             // output smoothing time constant (s) ~80 ms
    // Derived coefficients
    const float d_alpha = dt / (dt + (1.0f / (2.0f * (float)M_PI * d_cut_hz))); // 1st order LPF
    const float out_alpha = expf(-dt / out_tau_s);
    // States
    float gyro_filt = 0.0f;                    // deg/s
    float servo_cmd_deg = 0.0f;                // filtered servo command (deg)

    int log_count = 0;
    const int log_every = (int)(2.0f / dt); // 2 Hz
    if (log_every < 1) {
        // Fallback to every loop if dt is large
        // (shouldn't happen with 10 Hz)
    }

#if CONFIG_GIMBAL_IMU_SELECT_MSP_2AXIS
    // Initialize yaw servo channel
    ESP_ERROR_CHECK(yaw_servo_init(CONFIG_GIMBAL_SERVO_YAW_GPIO,
                                   CONFIG_GIMBAL_SERVO_FREQ_HZ,
                                   CONFIG_GIMBAL_SERVO_MIN_US,
                                   CONFIG_GIMBAL_SERVO_MAX_US));
    // Neutral both servos at boot
    servo_pwm_write_us((CONFIG_GIMBAL_SERVO_MIN_US + CONFIG_GIMBAL_SERVO_MAX_US)/2);
    yaw_servo_write_angle(0.0f, -CONFIG_GIMBAL_SERVO_LIMIT_DEG, +CONFIG_GIMBAL_SERVO_LIMIT_DEG);

    // Control parameters (reuse dt, out_alpha, deadband from earlier setup)
    float yaw_cmd_deg = 0.0f, pitch_cmd_deg = 0.0f;
    float yaw_out_deg = 0.0f, pitch_out_deg = 0.0f;
    // Wrap/flip state and limits
    static float prev_gimbal_yaw_cmd = 0.0f;     // deg (pre-smoothing)
    const float servo_half_allowed_deg = CONFIG_GIMBAL_YAW_SERVO_ALLOWED_DEG / 2.0f; // enforce safe window
    const float yaw_ratio_servo_per_ant = CONFIG_GIMBAL_YAW_RATIO_SERVO_PER_ANT_X1000 / 1000.0f;
    const float beam_half_deg = CONFIG_GIMBAL_ANTENNA_OPENING_DEG / 2.0f;
    static float phase_deg = 0.0f;               // multiples of 360 applied for wrap branch
    static bool phase_anim = false;
    static float phase_from = 0.0f, phase_to = 0.0f;
    static uint32_t phase_t0_us = 0;
    static uint32_t last_flip_us = 0;
    static int near_limit_count = 0;
    const uint32_t phase_T_us = 1000000;         // 1 second full rotate to "other side"
    const uint32_t flip_cooldown_us = 1500000;   // 1.5s cooldown between flips (avoid thrash)
    const int near_limit_required = 5;           // require consecutive detections before flip
    

    while (1) {
        button_poll();
        msp_att_t att;
        if (imu_msp_read(&att) == ESP_OK) {
            last_att.yaw_deg = att.yaw_deg;
            last_att.pitch_deg = att.pitch_deg;
            last_att.roll_deg = att.roll_deg;

            // Simpler controller:
            // Animate phase (±360) if we decide to flip to the other branch
            uint32_t now_us2 = (uint32_t)esp_timer_get_time();
            if (phase_anim) {
                float t = (now_us2 - phase_t0_us) / (float)phase_T_us;
                if (t >= 1.0f) { phase_deg = phase_to; phase_anim = false; }
                else { phase_deg = phase_from + (phase_to - phase_from) * t; }
            }

            // Base yaw error (front target): heading 0 = servo center (UNWRAPPED)
            // Choose among e0, e0±360 the one closest to previous command for continuity.
            float e0 = s_yaw_center_deg - att.yaw_deg + phase_deg;
            float e_cand = e0;
            float e_p = e0 + 360.0f;
            float e_m = e0 - 360.0f;
            if (fabsf(e_p - prev_gimbal_yaw_cmd) < fabsf(e_cand - prev_gimbal_yaw_cmd)) e_cand = e_p;
            if (fabsf(e_m - prev_gimbal_yaw_cmd) < fabsf(e_cand - prev_gimbal_yaw_cmd)) e_cand = e_m;

            // Apply antenna beam overlap: allow ±beam_half_deg without rotating (smooth deadband)
            float e_eff = e_cand;
            if (fabsf(e_eff) <= beam_half_deg) e_eff = 0.0f; else e_eff = copysignf(fabsf(e_eff) - beam_half_deg, e_eff);

            // Directional gimbal yaw limits based on servo swing and yaw gearing
            float gimbal_left_limit  = servo_half_allowed_deg / yaw_ratio_servo_per_ant;  // e > 0 (left)
            float gimbal_right_limit = servo_half_allowed_deg / yaw_ratio_servo_per_ant;  // e < 0 (right)
            // Asymmetric range per request: left up to +90, right up to -270 (subject to servo swing)
            if (gimbal_left_limit  > 90.0f)  gimbal_left_limit  = 90.0f;
            if (gimbal_right_limit > 270.0f) gimbal_right_limit = 270.0f;

            // If near limits (after overlap deadband), plan a flip by adding/subtracting 360 to phase and animate over 1s
            float margin = 2.0f; // deg slack before triggering flip
            bool near_limit = (e_eff > gimbal_left_limit - margin || e_eff < -gimbal_right_limit + margin);
            if (!phase_anim && near_limit) {
                near_limit_count++;
            } else {
                near_limit_count = 0;
            }
            if (!phase_anim && near_limit_count >= near_limit_required &&
                (now_us2 - last_flip_us) > flip_cooldown_us) {
                phase_from = phase_deg;
                // If we are near left limit (e>0), subtract 360 to flip; near right limit (e<0), add 360.
                phase_to = phase_deg + ((e_cand > 0) ? -360.0f : 360.0f);
                phase_t0_us = now_us2;
                phase_anim = true;
                last_flip_us = now_us2;
                near_limit_count = 0;
            }
            // Recompute with (possibly) updated phase, without wrap
            e0 = s_target_azimuth_deg - att.yaw_deg + phase_deg;
            // pick nearest branch again
            e_cand = e0; e_p = e0 + 360.0f; e_m = e0 - 360.0f;
            if (fabsf(e_p - prev_gimbal_yaw_cmd) < fabsf(e_cand - prev_gimbal_yaw_cmd)) e_cand = e_p;
            if (fabsf(e_m - prev_gimbal_yaw_cmd) < fabsf(e_cand - prev_gimbal_yaw_cmd)) e_cand = e_m;
            // overlap
            float e_final = e_cand;
            if (fabsf(e_final) <= beam_half_deg) e_final = 0.0f; else e_final = copysignf(fabsf(e_final) - beam_half_deg, e_final);
            // clamp to asymmetric limits
            if (e_final > gimbal_left_limit)  e_final = gimbal_left_limit;
            if (e_final < -gimbal_right_limit) e_final = -gimbal_right_limit;
            yaw_cmd_deg = e_final;

            // - Pitch command holds horizon (0 deg) with yaw-dependent gain
            //   When |yaw_cmd| ~ 90 deg, reduce pitch authority (cos goes to 0)
            float pitch_err = -att.pitch_deg; // target 0 deg (horizon)
            float pitch_gain = cosf(fabsf(deg2rad(yaw_cmd_deg))); // 1 at 0, ->0 at 90
            pitch_cmd_deg = pitch_err * pitch_gain;

            // Deadband
            if (fabsf(yaw_cmd_deg) < deadband_deg) yaw_cmd_deg = 0.0f;
            if (fabsf(pitch_cmd_deg) < deadband_deg) pitch_cmd_deg = 0.0f;

            // Smooth outputs
            yaw_out_deg = out_alpha * yaw_out_deg + (1.0f - out_alpha) * yaw_cmd_deg;
            pitch_out_deg = out_alpha * pitch_out_deg + (1.0f - out_alpha) * pitch_cmd_deg;
            prev_gimbal_yaw_cmd = yaw_cmd_deg;

            // Map to servo angles via mechanical ratio and clamp to servo limits
            float ratio = CONFIG_GIMBAL_MECH_RATIO / 1000.0f; // pitch ratio (servo per antenna)
            float servo_lim = CONFIG_GIMBAL_SERVO_LIMIT_DEG;
            float yaw_servo = yaw_out_deg * yaw_ratio_servo_per_ant;
            float pitch_servo = pitch_out_deg * ratio;
            if (yaw_servo > servo_half_allowed_deg) yaw_servo = servo_half_allowed_deg;
            if (yaw_servo < -servo_half_allowed_deg) yaw_servo = -servo_half_allowed_deg;
            if (pitch_servo > servo_lim) pitch_servo = servo_lim;
            if (pitch_servo < -servo_lim) pitch_servo = -servo_lim;

            // Write servos
            yaw_servo_write_angle(yaw_servo, -servo_half_allowed_deg, +servo_half_allowed_deg);
            servo_pwm_write_angle(pitch_servo, -servo_lim, +servo_lim);

            if (++log_count >= log_every) {
                log_count = 0;
                // Requested 2Hz format: imu_msp raw ints + gimbal relative
                ESP_LOGI(TAG, "imu_msp: raw ints r=%d p=%d y=%d; gimbal_rel: pitch=%.1f yaw=%.1f",
                         (int)att.raw_roll_i16, (int)att.raw_pitch_i16, (int)att.raw_yaw_i16,
                         pitch_out_deg, yaw_out_deg);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(loop_delay_ms));
    }
#else
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
            // Smooth deadband around zero
            if (fabsf(error) < deadband_deg) {
                error = 0.0f;
            } else {
                error = copysignf(fabsf(error) - deadband_deg, error);
            }

            // Clamp to antenna ±limit
            float ant_lim = CONFIG_GIMBAL_ANTENNA_LIMIT_DEG;
            if (error > ant_lim) error = ant_lim;
            if (error < -ant_lim) error = -ant_lim;

            // PID: P+I from error, D from (filtered) gyro (negative rate feedback)
            float u_pi = pid_update(&pid, error, dt); // kd=0 inside
            gyro_filt += d_alpha * (gyro_pitch_degps - gyro_filt);
            float u_d = -kd_rate * gyro_filt;
            float u = u_pi + u_d;

            // Convert to servo angle via mechanical ratio and clamp to servo ±limit
            float servo_lim = CONFIG_GIMBAL_SERVO_LIMIT_DEG;
            float servo_angle_cmd = u * (CONFIG_GIMBAL_MECH_RATIO / 1000.0f); // deg
            // Output smoothing to reduce jitter
            servo_cmd_deg = out_alpha * servo_cmd_deg + (1.0f - out_alpha) * servo_angle_cmd;
            float servo_angle = servo_cmd_deg;
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
#endif
}

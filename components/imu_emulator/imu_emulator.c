#include "imu/emulator.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <math.h>

// 15-second cycle, 6 segments of 2.5 s each
// Sequence: +pitch 30, +yaw 30, +roll 30, -yaw 30, -pitch 30, -roll 30

static int64_t s_t0_us;

esp_err_t imu_emulator_init(void)
{
    s_t0_us = esp_timer_get_time();
    return ESP_OK;
}

static void make_static_gravity_pitch(float deg, imu_sample_t *o)
{
    float th = deg * (float)M_PI / 180.0f;
    o->ax = -sinf(th);
    o->ay = 0.0f;
    o->az = cosf(th);
    o->gx = 0.0f; o->gy = 0.0f; o->gz = 0.0f;
}

static void make_static_gravity_roll(float deg, imu_sample_t *o)
{
    float ph = deg * (float)M_PI / 180.0f;
    o->ax = 0.0f;
    o->ay = sinf(ph);
    o->az = cosf(ph);
    o->gx = 0.0f; o->gy = 0.0f; o->gz = 0.0f;
}

static void make_yaw_only(float deg_per_s, imu_sample_t *o)
{
    // Yaw does not change gravity vector; output a constant yaw rate
    o->ax = 0.0f; o->ay = 0.0f; o->az = 1.0f;
    o->gx = 0.0f; o->gy = 0.0f; o->gz = deg_per_s;
}

esp_err_t imu_emulator_read(imu_sample_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    const float seg_s = 2.5f;
    const float total_s = 15.0f;
    float t = (esp_timer_get_time() - s_t0_us) / 1000000.0f;
    while (t >= total_s) t -= total_s;
    int seg = (int)(t / seg_s);

    switch (seg) {
    case 0: make_static_gravity_pitch(+30.0f, out); break;
    case 1: make_yaw_only(+60.0f/seg_s, out); break;     // complete +30 yaw over 2.5s ~12 deg/s; use faster for visibility
    case 2: make_static_gravity_roll(+30.0f, out); break;
    case 3: make_yaw_only(-60.0f/seg_s, out); break;
    case 4: make_static_gravity_pitch(-30.0f, out); break;
    case 5: make_static_gravity_roll(-30.0f, out); break;
    default: make_static_gravity_pitch(0.0f, out); break;
    }
    return ESP_OK;
}

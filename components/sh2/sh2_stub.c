#include "sh2_glue.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "sh2_stub";

static volatile struct {
    float ax, ay, az;
    float gx, gy, gz;
    bool valid;
} s_last;

void sh2_init(void)
{
    memset((void*)&s_last, 0, sizeof(s_last));
    ESP_LOGW(TAG, "Using SH-2/SHTP STUB (no real parsing). Integrate vendor SH-2 to enable reports.");
}

void sh2_on_rx(uint8_t channel, uint8_t seq, const uint8_t *payload, uint16_t len)
{
    (void)seq;
    // For visibility, log a compact line when control or sensor channels deliver data.
    // Real implementation will decode control responses and sensor reports here.
    ESP_LOGI(TAG, "rx ch=%u len=%u", (unsigned)channel, (unsigned)len);
    // Keep placeholder values; when integrated, update s_last.* here.
}

bool sh2_has_tx(void)
{
    // No outbound frames in stub (no Set Feature commands yet).
    return false;
}

uint16_t sh2_get_tx(uint8_t *buf, uint16_t buf_sz)
{
    (void)buf; (void)buf_sz;
    return 0;
}

bool sh2_get_latest_sample(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
    if (!s_last.valid) return false;
    if (ax) *ax = s_last.ax;
    if (ay) *ay = s_last.ay;
    if (az) *az = s_last.az;
    if (gx) *gx = s_last.gx;
    if (gy) *gy = s_last.gy;
    if (gz) *gz = s_last.gz;
    return true;
}

#pragma once
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    uint32_t age_ms; // time since last update (approx)
} msp_att_t;

// Initialize MSP UART and start polling task (MSP v1 MSP_ATTITUDE)
esp_err_t imu_msp_init(void);

// Read latest attitude snapshot (non-blocking). Returns ESP_OK if data available.
esp_err_t imu_msp_read(msp_att_t *out);

#ifdef __cplusplus
}
#endif

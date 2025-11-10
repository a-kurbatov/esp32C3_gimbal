#pragma once
#include <stdbool.h>
#include <stdint.h>

// Minimal glue API to integrate SH-2/SHTP stack.
// This is a placeholder stub until the vendor code is integrated.

#ifdef __cplusplus
extern "C" {
#endif

// Initialize SH-2/SHTP state (transport-independent).
void sh2_init(void);

// Notify incoming SHTP packet from transport.
// channel: SHTP logical channel (0..4)
// seq: sequence number observed in header
// payload: pointer to bytes after 4-byte SHTP header
// len: payload length (header length - 4)
void sh2_on_rx(uint8_t channel, uint8_t seq, const uint8_t *payload, uint16_t len);

// Check whether there is a transport frame pending transmit (e.g., control commands).
bool sh2_has_tx(void);

// Get the next SHTP frame to transmit. Returns frame length including 4-byte SHTP header.
// buf must be large enough (recommend >= 384 bytes). Returns 0 if none.
uint16_t sh2_get_tx(uint8_t *buf, uint16_t buf_sz);

// Try to extract the latest accel/gyro sample parsed so far.
// Returns true if a fresh sample is available; outputs SI units (m/s^2 and rad/s).
bool sh2_get_latest_sample(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);

#ifdef __cplusplus
}
#endif

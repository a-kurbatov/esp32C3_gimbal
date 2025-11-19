#pragma once
#include <stdint.h>

typedef struct {
    float ax, ay, az;     // m/s^2 or g (unitless) depending on driver; assumed g here
    float gx, gy, gz;     // deg/s
    // Optional: Game Rotation Vector quaternion (unitless, normalized)
    // When available (e.g., BNO08x GRV), qw is the real part.
    float qw, qx, qy, qz;
} imu_sample_t;

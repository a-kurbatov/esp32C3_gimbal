#pragma once
#include <stdint.h>

typedef struct {
    float ax, ay, az;     // m/s^2 or g (unitless) depending on driver; assumed g here
    float gx, gy, gz;     // deg/s
} imu_sample_t;

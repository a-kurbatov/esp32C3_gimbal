/* Vendored from Adafruit_BNO08x (Apache 2.0) */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "sh2.h"

typedef struct sh2_Accelerometer { float x,y,z; } sh2_Accelerometer_t;
typedef struct sh2_Gyroscope { float x,y,z; } sh2_Gyroscope_t;

typedef struct sh2_SensorValue {
    uint8_t sensorId;
    uint8_t sequence;
    uint8_t status;
    uint64_t timestamp;
    uint32_t delay;
    union {
        sh2_Accelerometer_t accelerometer;
        sh2_Gyroscope_t gyroscope;
    } un;
} sh2_SensorValue_t;

int sh2_decodeSensorEvent(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);

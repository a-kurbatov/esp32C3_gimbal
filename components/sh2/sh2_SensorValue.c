/* Vendored minimal subset from Adafruit_BNO08x (Apache 2.0) */
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include "sh2_util.h"
#define SCALE_Q(n) (1.0f / (1 << n))
int sh2_decodeSensorEvent(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->sensorId = event->reportId;
    value->timestamp = event->timestamp_uS;
    value->sequence = event->report[1];
    value->status = event->report[2] & 0x03;
    switch (value->sensorId) {
        case SH2_ACCELEROMETER:
            value->un.accelerometer.x = (float)read16(&event->report[4]) * SCALE_Q(8);
            value->un.accelerometer.y = (float)read16(&event->report[6]) * SCALE_Q(8);
            value->un.accelerometer.z = (float)read16(&event->report[8]) * SCALE_Q(8);
            return SH2_OK;
        case SH2_GYROSCOPE_CALIBRATED:
            value->un.gyroscope.x = (float)read16(&event->report[4]) * SCALE_Q(9);
            value->un.gyroscope.y = (float)read16(&event->report[6]) * SCALE_Q(9);
            value->un.gyroscope.z = (float)read16(&event->report[8]) * SCALE_Q(9);
            return SH2_OK;
        case SH2_GAME_ROTATION_VECTOR:
            // Quaternion components in report are i, j, k, real (Q14)
            value->un.gameRotationVector.i    = (float)read16(&event->report[4])  * SCALE_Q(14);
            value->un.gameRotationVector.j    = (float)read16(&event->report[6])  * SCALE_Q(14);
            value->un.gameRotationVector.k    = (float)read16(&event->report[8])  * SCALE_Q(14);
            value->un.gameRotationVector.real = (float)read16(&event->report[10]) * SCALE_Q(14);
            return SH2_OK;
        default:
            return SH2_ERR;
    }
}

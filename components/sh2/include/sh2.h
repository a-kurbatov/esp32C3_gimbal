/* Vendored types from Hillcrest SH-2 (Apache 2.0) – minimal subset to satisfy sh2.c */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "sh2_hal.h"
#include "sh2_err.h"
#include "shtp.h"

#ifdef __cplusplus
extern "C" {
#endif

// Increase buffer to safely hold largest sensor reports (e.g., GRV, metadata)
#define SH2_MAX_SENSOR_EVENT_LEN (64)
typedef struct sh2_SensorEvent {
    uint64_t timestamp_uS;
    uint8_t len;
    uint8_t reportId;
    uint8_t report[SH2_MAX_SENSOR_EVENT_LEN];
} sh2_SensorEvent_t;

typedef void (sh2_SensorCallback_t)(void * cookie, sh2_SensorEvent_t *pEvent);

typedef struct sh2_ProductId_s {
    uint8_t resetCause;
    uint8_t swVersionMajor;
    uint8_t swVersionMinor;
    uint32_t swPartNumber;
    uint32_t swBuildNumber;
    uint16_t swVersionPatch;
    uint8_t reserved0;
    uint8_t reserved1;
} sh2_ProductId_t;

#define SH2_MAX_PROD_ID_ENTRIES (5)
typedef struct sh2_ProductIds_s {
    sh2_ProductId_t entry[SH2_MAX_PROD_ID_ENTRIES];
    uint8_t numEntries;
} sh2_ProductIds_t;

enum sh2_SensorId_e {
    SH2_RAW_ACCELEROMETER = 0x14,
    SH2_ACCELEROMETER = 0x01,
    SH2_LINEAR_ACCELERATION = 0x04,
    SH2_GRAVITY = 0x06,
    SH2_RAW_GYROSCOPE = 0x15,
    SH2_GYROSCOPE_CALIBRATED = 0x02,
    SH2_GYROSCOPE_UNCALIBRATED = 0x07,
    SH2_RAW_MAGNETOMETER = 0x16,
    SH2_MAGNETIC_FIELD_CALIBRATED = 0x03,
    SH2_MAGNETIC_FIELD_UNCALIBRATED = 0x0f,
    SH2_ROTATION_VECTOR = 0x05,
    SH2_GAME_ROTATION_VECTOR = 0x08,
    SH2_GEOMAGNETIC_ROTATION_VECTOR = 0x09,
    SH2_PRESSURE = 0x0a,
    SH2_AMBIENT_LIGHT = 0x0b,
    SH2_HUMIDITY = 0x0c,
    SH2_PROXIMITY = 0x0d,
    SH2_TEMPERATURE = 0x0e,
    SH2_TAP_DETECTOR = 0x10,
    SH2_STEP_DETECTOR = 0x18,
    SH2_STEP_COUNTER = 0x11,
    SH2_SIGNIFICANT_MOTION = 0x12,
    SH2_STABILITY_CLASSIFIER = 0x13,
    SH2_SHAKE_DETECTOR = 0x19,
    SH2_FLIP_DETECTOR = 0x1a,
    SH2_PICKUP_DETECTOR = 0x1b,
    SH2_STABILITY_DETECTOR = 0x1c,
    SH2_PERSONAL_ACTIVITY_CLASSIFIER = 0x1e,
    SH2_SLEEP_DETECTOR = 0x1f,
    SH2_TILT_DETECTOR = 0x20,
    SH2_POCKET_DETECTOR = 0x21,
    SH2_CIRCLE_DETECTOR = 0x22,
    SH2_ARVR_STABILIZED_RV = 0x28,
    SH2_ARVR_STABILIZED_GRV = 0x29,
    SH2_GYRO_INTEGRATED_RV = 0x2A,
    SH2_MAX_SENSOR_ID = 0x2B,
};
typedef uint8_t sh2_SensorId_t;

typedef struct sh2_SensorConfig {
    bool changeSensitivityEnabled;
    bool changeSensitivityRelative;
    bool wakeupEnabled;
    bool alwaysOnEnabled;
    uint16_t changeSensitivity;
    uint32_t reportInterval_us;
    uint32_t batchInterval_us;
    uint32_t sensorSpecific;
} sh2_SensorConfig_t;

// Async events
typedef enum sh2_AsyncEventId_e {
    SH2_RESET,
    SH2_SHTP_EVENT,
    SH2_GET_FEATURE_RESP,
} sh2_AsyncEventId_t;

typedef struct {
    sh2_SensorId_t sensorId;
    sh2_SensorConfig_t sensorConfig;
} sh2_SensorConfigResp_t;

typedef struct sh2_AsyncEvent {
    uint32_t eventId;
    union {
        shtp_Event_t shtpEvent;
        sh2_SensorConfigResp_t sh2SensorConfigResp;
    };
} sh2_AsyncEvent_t;

typedef void (sh2_EventCallback_t)(void * cookie, sh2_AsyncEvent_t *pEvent);

// Error record and counts
typedef struct sh2_ErrorRecord {
    uint8_t severity;
    uint8_t sequence;
    uint8_t source;
    uint8_t error;
    uint8_t module;
    uint8_t code;
} sh2_ErrorRecord_t;

typedef struct sh2_Counts {
    uint32_t offered;
    uint32_t accepted;
    uint32_t on;
    uint32_t attempted;
} sh2_Counts_t;

// Oscillator type
typedef enum {
    SH2_OSC_INTERNAL    = 0,
    SH2_OSC_EXT_CRYSTAL = 1,
    SH2_OSC_EXT_CLOCK   = 2,
} sh2_OscType_t;

// Calibration
typedef enum {
    SH2_CAL_SUCCESS = 0,
} sh2_CalStatus_t;
#define SH2_CAL_ACCEL  (0x01)
#define SH2_CAL_GYRO   (0x02)
#define SH2_CAL_MAG    (0x04)
#define SH2_CAL_PLANAR (0x08)

// Tare & quaternion
typedef enum { SH2_TARE_BASIS_ROTATION_VECTOR = 0, SH2_TARE_BASIS_GAMING_ROTATION_VECTOR = 1, SH2_TARE_BASIS_GEOMAGNETIC_ROTATION_VECTOR = 2 } sh2_TareBasis_t;
typedef struct sh2_Quaternion { double x,y,z,w; } sh2_Quaternion_t;

// Interactive ZRO intent (minimal)
typedef enum {
    SH2_IZRO_MI_UNKNOWN = 0,
    SH2_IZRO_MI_STATIONARY_NO_VIBRATION = 1,
    SH2_IZRO_MI_STATIONARY_WITH_VIBRATION = 2,
    SH2_IZRO_MI_IN_MOTION = 3,
} sh2_IZroMotionIntent_t;

// Sensor metadata
typedef struct sh2_SensorMetadata {
    uint8_t meVersion; uint8_t mhVersion; uint8_t shVersion;
    uint32_t range; uint32_t resolution;
    uint16_t revision; uint16_t power_mA;
    uint32_t minPeriod_uS; uint32_t maxPeriod_uS;
    uint16_t fifoReserved; uint16_t fifoMax;
    uint32_t batchBufferBytes;
    uint16_t qPoint1; uint16_t qPoint2; uint16_t qPoint3;
    uint32_t vendorIdLen; char vendorId[48];
    uint32_t sensorSpecificLen; uint8_t sensorSpecific[48];
} sh2_SensorMetadata_t;

// FRS metadata record IDs used by sh2.c
#define FRS_ID_META_RAW_ACCELEROMETER            (0xE301)
#define FRS_ID_META_ACCELEROMETER                (0xE302)
#define FRS_ID_META_LINEAR_ACCELERATION          (0xE303)
#define FRS_ID_META_GRAVITY                      (0xE304)
#define FRS_ID_META_RAW_GYROSCOPE                (0xE305)
#define FRS_ID_META_GYROSCOPE_CALIBRATED         (0xE306)
#define FRS_ID_META_GYROSCOPE_UNCALIBRATED       (0xE307)
#define FRS_ID_META_RAW_MAGNETOMETER             (0xE308)
#define FRS_ID_META_MAGNETIC_FIELD_CALIBRATED    (0xE309)
#define FRS_ID_META_MAGNETIC_FIELD_UNCALIBRATED  (0xE30A)
#define FRS_ID_META_ROTATION_VECTOR              (0xE30B)
#define FRS_ID_META_GAME_ROTATION_VECTOR         (0xE30C)
#define FRS_ID_META_GEOMAGNETIC_ROTATION_VECTOR  (0xE30D)
#define FRS_ID_META_PRESSURE                     (0xE30E)
#define FRS_ID_META_AMBIENT_LIGHT                (0xE30F)
#define FRS_ID_META_HUMIDITY                     (0xE310)
#define FRS_ID_META_PROXIMITY                    (0xE311)
#define FRS_ID_META_TEMPERATURE                  (0xE312)
#define FRS_ID_META_TAP_DETECTOR                 (0xE313)
#define FRS_ID_META_STEP_DETECTOR                (0xE314)
#define FRS_ID_META_STEP_COUNTER                 (0xE315)
#define FRS_ID_META_SIGNIFICANT_MOTION           (0xE316)
#define FRS_ID_META_STABILITY_CLASSIFIER         (0xE317)
#define FRS_ID_META_SHAKE_DETECTOR               (0xE318)
#define FRS_ID_META_FLIP_DETECTOR                (0xE319)
#define FRS_ID_META_PICKUP_DETECTOR              (0xE31A)
#define FRS_ID_META_STABILITY_DETECTOR           (0xE31B)
#define FRS_ID_META_PERSONAL_ACTIVITY_CLASSIFIER (0xE31C)
#define FRS_ID_META_SLEEP_DETECTOR               (0xE31D)
#define FRS_ID_META_TILT_DETECTOR                (0xE31E)
#define FRS_ID_META_POCKET_DETECTOR              (0xE31F)
#define FRS_ID_META_CIRCLE_DETECTOR              (0xE320)

// API
int sh2_open(sh2_Hal_t *pHal, sh2_EventCallback_t *eventCallback, void *eventCookie);
void sh2_close(void);
void sh2_service(void);
int sh2_setSensorCallback(sh2_SensorCallback_t *callback, void *cookie);
int sh2_devReset(void);
int sh2_devOn(void);
int sh2_devSleep(void);
int sh2_getProdIds(sh2_ProductIds_t *prodIds);
int sh2_getSensorConfig(sh2_SensorId_t sensorId, sh2_SensorConfig_t *pConfig);
int sh2_setSensorConfig(sh2_SensorId_t sensorId, const sh2_SensorConfig_t *pConfig);
int sh2_getMetadata(sh2_SensorId_t sensorId, sh2_SensorMetadata_t *pData);
int sh2_getErrors(uint8_t severity, sh2_ErrorRecord_t *pErrors, uint16_t *numErrors);
int sh2_getCounts(sh2_SensorId_t sensorId, sh2_Counts_t *pCounts);
int sh2_getOscType(sh2_OscType_t *pOscType);
int sh2_finishCal(sh2_CalStatus_t *status);
int sh2_setTareNow(uint8_t axes, sh2_TareBasis_t basis);
int sh2_setReorientation(sh2_Quaternion_t *orientation);
int sh2_setDcdAutoSave(bool enabled);
int sh2_reinitialize(void);

#ifdef __cplusplus
}
#endif

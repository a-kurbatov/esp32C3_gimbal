/* Vendored from Adafruit_BNO08x (Apache 2.0) */
#pragma once
#include <stdint.h>

#define SH2_HAL_MAX_TRANSFER_OUT (256)
#define SH2_HAL_MAX_PAYLOAD_OUT  (256)
#define SH2_HAL_MAX_TRANSFER_IN  (384)
#define SH2_HAL_MAX_PAYLOAD_IN   (384)
#define SH2_HAL_DMA_SIZE (512)

typedef struct sh2_Hal_s sh2_Hal_t;

struct sh2_Hal_s {
    int (*open)(sh2_Hal_t *self);
    void (*close)(sh2_Hal_t *self);
    int (*read)(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
    int (*write)(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
    uint32_t (*getTimeUs)(sh2_Hal_t *self);
};

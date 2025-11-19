/* Vendored from Adafruit_BNO08x (Apache 2.0) */
#pragma once
#include <stdint.h>

// Increase buffers to accommodate large SHTP transfers seen during adverts and metadata.
// Values chosen to handle up to 8KB single-fragment transfers and payload assemblies.
#define SH2_HAL_MAX_TRANSFER_OUT (512)
#define SH2_HAL_MAX_PAYLOAD_OUT  (512)
#define SH2_HAL_MAX_TRANSFER_IN  (8192)
#define SH2_HAL_MAX_PAYLOAD_IN   (12288)
#define SH2_HAL_DMA_SIZE (8192)

typedef struct sh2_Hal_s sh2_Hal_t;

struct sh2_Hal_s {
    int (*open)(sh2_Hal_t *self);
    void (*close)(sh2_Hal_t *self);
    int (*read)(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
    int (*write)(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
    uint32_t (*getTimeUs)(sh2_Hal_t *self);
};

/* Vendored from Adafruit_BNO08x (Apache 2.0) */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "sh2_hal.h"

// Advertisement TLV tags
#define TAG_NULL 0
#define TAG_GUID 1
#define TAG_MAX_CARGO_PLUS_HEADER_WRITE 2
#define TAG_MAX_CARGO_PLUS_HEADER_READ 3
#define TAG_MAX_TRANSFER_WRITE 4
#define TAG_MAX_TRANSFER_READ 5
#define TAG_NORMAL_CHANNEL 6
#define TAG_WAKE_CHANNEL 7
#define TAG_APP_NAME 8
#define TAG_CHANNEL_NAME 9
#define TAG_ADV_COUNT 10
#define TAG_APP_SPECIFIC 0x80

// Standard SHTP header length
#define SHTP_HDR_LEN 4

typedef enum shtp_Event_e {
    SHTP_TX_DISCARD = 0,
    SHTP_SHORT_FRAGMENT = 1,
    SHTP_TOO_LARGE_PAYLOADS = 2,
    SHTP_BAD_RX_CHAN = 3,
    SHTP_BAD_TX_CHAN = 4,
} shtp_Event_t;

typedef void shtp_Callback_t(void * cookie, uint8_t *payload, uint16_t len, uint32_t timestamp);
typedef void shtp_AdvertCallback_t(void * cookie, uint8_t tag, uint8_t len, uint8_t *value);
typedef void shtp_SendCallback_t(void *cookie);
typedef void shtp_EventCallback_t(void *cookie, shtp_Event_t shtpEvent);

void *shtp_open(sh2_Hal_t *pHal);
void shtp_close(void *pShtp);
void shtp_setEventCallback(void *pInstance, shtp_EventCallback_t *eventCallback, void *eventCookie);
int shtp_listenChan(void *pShtp, uint16_t guid, const char * chan, shtp_Callback_t *callback, void *cookie);
int shtp_listenAdvert(void *pShtp, uint16_t guid, shtp_AdvertCallback_t *advertCallback, void * cookie);
uint8_t shtp_chanNo(void *pShtp, const char * appName, const char * chanName);
int shtp_send(void *pShtp, uint8_t channel, const uint8_t *payload, uint16_t len);
void shtp_service(void *pShtp);

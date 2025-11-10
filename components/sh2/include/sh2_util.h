/* Vendored from Adafruit_BNO08x (Apache 2.0) */
#pragma once
#include <stdint.h>
#ifndef ARRAY_LEN
#define ARRAY_LEN(a) (sizeof(a)/sizeof(a[0]))
#endif
uint8_t  readu8(const uint8_t * buffer);
void writeu8(uint8_t * buffer, uint8_t value);
uint16_t readu16(const uint8_t * buffer);
void writeu16(uint8_t * buffer, uint16_t value);
uint32_t readu32(const uint8_t * buffer);
void writeu32(uint8_t * buffer, uint32_t value);
int8_t read8(const uint8_t * buffer);
void write8(uint8_t * buffer, int8_t value);
int16_t read16(const uint8_t * buffer);
void write16(uint8_t * buffer, int16_t value);
int32_t read32(const uint8_t * buffer);
void write32(uint8_t * buffer, int32_t value);

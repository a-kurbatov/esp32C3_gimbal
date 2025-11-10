/* Vendored from Adafruit_BNO08x (Apache 2.0) */
#include "sh2_util.h"
uint8_t readu8(const uint8_t *p){return p[0];}
void writeu8(uint8_t *p,uint8_t v){*p=v;}
uint16_t readu16(const uint8_t *p){return (uint16_t)(p[0]|(p[1]<<8));}
void writeu16(uint8_t *p,uint16_t v){*p++=(uint8_t)(v&0xFF);*p=(uint8_t)((v>>8)&0xFF);} 
uint32_t readu32(const uint8_t *p){return (uint32_t)(p[0]|(p[1]<<8)|(p[2]<<16)|(p[3]<<24));}
void writeu32(uint8_t *p,uint32_t v){*p++=(uint8_t)(v&0xFF);v>>=8;*p++=(uint8_t)(v&0xFF);v>>=8;*p++=(uint8_t)(v&0xFF);v>>=8;*p=(uint8_t)(v&0xFF);} 
int8_t read8(const uint8_t *p){return (int8_t)p[0];}
void write8(uint8_t *p,int8_t v){*p=(uint8_t)v;}
int16_t read16(const uint8_t *p){return (int16_t)(p[0]|(p[1]<<8));}
void write16(uint8_t *p,int16_t v){*p++=(uint8_t)(v&0xFF);*p=(uint8_t)((v>>8)&0xFF);} 
int32_t read32(const uint8_t *p){return (int32_t)(p[0]|(p[1]<<8)|(p[2]<<16)|(p[3]<<24));}
void write32(uint8_t *p,int32_t v){*p++=(uint8_t)(v&0xFF);v>>=8;*p++=(uint8_t)(v&0xFF);v>>=8;*p++=(uint8_t)(v&0xFF);v>>=8;*p=(uint8_t)(v&0xFF);} 

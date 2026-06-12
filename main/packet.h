/*
 * packet.h - Dinh dang goi tin LoRa giua Node va Gateway.
 * FILE NAY PHAI GIONG HET NHAU o ca 2 project (Node va Gateway).
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

#define SENSOR_PKT_MAGIC0   0xA5
#define SENSOR_PKT_MAGIC1   0x5A
#define SENSOR_PKT_VERSION  1

/* Bit co trong truong flags: sensor nao doc thanh cong thi bit = 1 */
#define PKT_F_SHT30_OK      (1u << 0)
#define PKT_F_BMP280_OK     (1u << 1)
#define PKT_F_BH1750_OK     (1u << 2)

typedef struct __attribute__((packed)) {
    uint8_t  magic0;        /* 0xA5 */
    uint8_t  magic1;        /* 0x5A */
    uint8_t  version;       /* SENSOR_PKT_VERSION */
    uint8_t  node_id;
    uint16_t seq;           /* so thu tu goi tin */
    uint8_t  flags;         /* PKT_F_xxx */
    float    sht_temp_c;    /* SHT30  - nhiet do (do C) */
    float    sht_hum_pct;   /* SHT30  - do am (%RH) */
    float    bmp_temp_c;    /* BMP280 - nhiet do (do C) */
    float    bmp_press_hpa; /* BMP280 - ap suat (hPa) */
    float    lux;           /* BH1750 - anh sang (lux) */
    uint8_t  crc8;          /* CRC8 cua tat ca byte phia truoc */
} sensor_packet_t;          /* tong cong 28 byte */

/* CRC8: poly 0x31, init 0xFF (giong ho CRC cua Sensirion) */
static inline uint8_t sensor_pkt_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static inline void sensor_pkt_seal(sensor_packet_t *p)
{
    p->magic0  = SENSOR_PKT_MAGIC0;
    p->magic1  = SENSOR_PKT_MAGIC1;
    p->version = SENSOR_PKT_VERSION;
    p->crc8 = sensor_pkt_crc8((const uint8_t *)p, sizeof(*p) - 1);
}

/* Tra ve 1 neu goi tin hop le */
static inline int sensor_pkt_valid(const sensor_packet_t *p)
{
    return p->magic0 == SENSOR_PKT_MAGIC0 &&
           p->magic1 == SENSOR_PKT_MAGIC1 &&
           p->version == SENSOR_PKT_VERSION &&
           p->crc8 == sensor_pkt_crc8((const uint8_t *)p, sizeof(*p) - 1);
}

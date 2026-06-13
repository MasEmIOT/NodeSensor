/*
 * packet.h - Goi tin LoRa MA HOA giua Node va Gateway (v3).
 * FILE NAY + packet.c + crypto* + ascon* PHAI GIONG HET o ca 2 project.
 *
 *  Tren duong truyen:  [env_header_t][ciphertext (plen byte)][tag 16 byte]
 *  Phan ciphertext la app_payload_t da ma hoa (AES-GCM hoac ASCON-128).
 *  ACK (Gateway->Node) khong ma hoa, chi CRC8.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

#define PKT_VERSION         3
#define ENV_MAGIC0          0xA5
#define ENV_MAGIC1          0x5A
#define ACK_MAGIC0          0x5A
#define ACK_MAGIC1          0xA5

/* Co sensor doc thanh cong */
#define PKT_F_SHT30_OK      (1u << 0)
#define PKT_F_BMP180_OK     (1u << 1)
#define PKT_F_BH1750_OK     (1u << 2)

/* ====== Payload ung dung (phan duoc MA HOA) ====== */
typedef struct __attribute__((packed)) {
    uint8_t  node_id;
    uint16_t seq;
    uint8_t  flags;
    float    sht_temp_c;
    float    sht_hum_pct;
    float    bmp_temp_c;
    float    bmp_press_hpa;
    float    lux;
    uint16_t rtt_ms;        /* round-trip chu ky truoc (ms) */
    uint16_t dist_dm;       /* khoang cach uoc luong (decimet = 0.1m) */
    uint16_t proc_us;       /* thoi gian xu ly (doc sensor + ma hoa) (us) */
    uint16_t enc_us;        /* thoi gian ma hoa rieng (us) */
    int16_t  dl_rssi;       /* RSSI cua ACK ma node thu duoc (downlink) */
    uint8_t  algo;          /* thuat toan ma hoa dang dung */
} app_payload_t;

/* ====== Header tren khong (cleartext) ====== */
typedef struct __attribute__((packed)) {
    uint8_t  magic0;        /* 0xA5 */
    uint8_t  magic1;        /* 0x5A */
    uint8_t  version;       /* PKT_VERSION */
    uint8_t  algo;          /* CRYPTO_ALGO_xxx */
    uint8_t  node_id;       /* cleartext de loc/dinh tuyen */
    uint8_t  plen;          /* do dai plaintext */
    uint8_t  nonce[16];
} env_header_t;

#define ENV_TAG_LEN     16
#define ENV_MAX_LEN     (sizeof(env_header_t) + sizeof(app_payload_t) + ENV_TAG_LEN)

/* ====== ACK (Gateway -> Node), khong ma hoa ====== */
#define ACK_STATUS_OK   0
#define ACK_STATUS_BUSY 1

typedef struct __attribute__((packed)) {
    uint8_t  magic0;        /* 0x5A */
    uint8_t  magic1;        /* 0xA5 */
    uint8_t  version;
    uint8_t  node_id;
    uint16_t seq;
    int16_t  ul_rssi;       /* RSSI gateway thu duoc tu node (uplink) */
    int8_t   ul_snr;        /* SNR (dB, lam tron) */
    uint8_t  status;
    uint8_t  crc8;
} ack_packet_t;

/* CRC8 dung chung (poly 0x31, init 0xFF) */
static inline uint8_t pkt_crc8(const uint8_t *data, size_t len)
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

/* ====== ACK helpers ====== */
static inline void ack_pkt_seal(ack_packet_t *a, uint8_t node_id, uint16_t seq,
                                int16_t ul_rssi, int8_t ul_snr, uint8_t status)
{
    a->magic0 = ACK_MAGIC0;
    a->magic1 = ACK_MAGIC1;
    a->version = PKT_VERSION;
    a->node_id = node_id;
    a->seq = seq;
    a->ul_rssi = ul_rssi;
    a->ul_snr = ul_snr;
    a->status = status;
    a->crc8 = pkt_crc8((const uint8_t *)a, sizeof(*a) - 1);
}

static inline int ack_pkt_valid(const ack_packet_t *a)
{
    return a->magic0 == ACK_MAGIC0 && a->magic1 == ACK_MAGIC1 &&
           a->version == PKT_VERSION &&
           a->crc8 == pkt_crc8((const uint8_t *)a, sizeof(*a) - 1);
}

/* ====== Envelope seal/open (trong packet.c) ====== */

/* Ma hoa payload -> out (toi thieu ENV_MAX_LEN byte). *out_len = tong byte. 0=OK */
int packet_seal(const app_payload_t *pl, uint8_t algo, const uint8_t key[16],
                uint8_t *out, size_t *out_len);

/* Giai ma buf (len byte) -> pl_out. *algo_out = thuat toan. 0=OK, !=0 loi/gia mao */
int packet_open(const uint8_t *buf, size_t len, const uint8_t key[16],
                app_payload_t *pl_out, uint8_t *algo_out);

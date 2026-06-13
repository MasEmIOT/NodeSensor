/*
 * crypto.h - Lop AEAD thong nhat: NONE / AES-128-GCM / ASCON-128.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

#define CRYPTO_KEY_LEN    16
#define CRYPTO_NONCE_LEN  16
#define CRYPTO_TAG_LEN    16

/* Ma hoa. Tra ve 0 neu OK. ct phai du pt_len byte, tag du 16 byte. */
int crypto_aead_encrypt(uint8_t algo, const uint8_t *key, const uint8_t *nonce,
                        const uint8_t *aad, size_t aad_len,
                        const uint8_t *pt, size_t pt_len,
                        uint8_t *ct, uint8_t *tag);

/* Giai ma + xac thuc. Tra ve 0 neu tag dung, khac 0 neu loi/gia mao. */
int crypto_aead_decrypt(uint8_t algo, const uint8_t *key, const uint8_t *nonce,
                        const uint8_t *aad, size_t aad_len,
                        const uint8_t *ct, size_t ct_len,
                        const uint8_t *tag, uint8_t *pt);

const char *crypto_algo_name(uint8_t algo);

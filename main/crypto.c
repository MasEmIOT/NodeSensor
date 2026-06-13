#include <string.h>
#include <stdlib.h>
#include "crypto.h"
#include "crypto_cfg.h"
#include "ascon.h"

// Thư viện mới của ESP-IDF v6.0+ thay thế cho mbedtls/gcm.h
#include "psa/crypto.h" 

const char *crypto_algo_name(uint8_t algo)
{
    switch (algo) {
        case CRYPTO_ALGO_AES:   return "AES-128-GCM";
        case CRYPTO_ALGO_ASCON: return "ASCON-128";
        default:                return "NONE";
    }
}

static int aes_gcm_enc(const uint8_t *key, const uint8_t *nonce,
                       const uint8_t *aad, size_t aad_len,
                       const uint8_t *pt, size_t pt_len,
                       uint8_t *ct, uint8_t *tag)
{
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_key_id_t key_id = 0;
    psa_status_t status;

    // 1. Cấu hình thuộc tính cho AES-128 GCM
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_ENCRYPT);
    psa_set_key_algorithm(&attributes, PSA_ALG_GCM);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, 128);

    // 2. Import Key
    status = psa_import_key(&attributes, key, 16, &key_id); // 128 bits = 16 bytes
    if (status != PSA_SUCCESS) {
        return (int)status;
    }

    // 3. Cấp phát bộ đệm tạm vì PSA gộp chung ciphertext và tag
    size_t buf_size = pt_len + CRYPTO_TAG_LEN;
    uint8_t *buf = (uint8_t *)malloc(buf_size);
    if (!buf) {
        psa_destroy_key(key_id);
        return -1; // Lỗi cấp phát bộ nhớ
    }

    // 4. Thực thi mã hóa
    size_t out_len = 0;
    status = psa_aead_encrypt(key_id, PSA_ALG_GCM,
                              nonce, CRYPTO_NONCE_LEN,
                              aad, aad_len,
                              pt, pt_len,
                              buf, buf_size,
                              &out_len);

    // 5. Tách dữ liệu trả về đúng các con trỏ yêu cầu
    if (status == PSA_SUCCESS) {
        memcpy(ct, buf, pt_len);
        memcpy(tag, buf + pt_len, CRYPTO_TAG_LEN);
    }

    // 6. Dọn dẹp
    free(buf);
    psa_destroy_key(key_id);
    return (int)status;
}

static int aes_gcm_dec(const uint8_t *key, const uint8_t *nonce,
                       const uint8_t *aad, size_t aad_len,
                       const uint8_t *ct, size_t ct_len,
                       const uint8_t *tag, uint8_t *pt)
{
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_key_id_t key_id = 0;
    psa_status_t status;

    // 1. Cấu hình thuộc tính cho AES-128 GCM
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_DECRYPT);
    psa_set_key_algorithm(&attributes, PSA_ALG_GCM);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, 128);

    // 2. Import Key
    status = psa_import_key(&attributes, key, 16, &key_id);
    if (status != PSA_SUCCESS) {
        return (int)status;
    }

    // 3. Cấp phát bộ đệm tạm và gộp chung ciphertext + tag vào nhau
    size_t buf_size = ct_len + CRYPTO_TAG_LEN;
    uint8_t *buf = (uint8_t *)malloc(buf_size);
    if (!buf) {
        psa_destroy_key(key_id);
        return -1;
    }

    memcpy(buf, ct, ct_len);
    memcpy(buf + ct_len, tag, CRYPTO_TAG_LEN);

    // 4. Thực thi giải mã (Xác thực Tag tự động xảy ra bên trong hàm này)
    size_t out_len = 0;
    status = psa_aead_decrypt(key_id, PSA_ALG_GCM,
                              nonce, CRYPTO_NONCE_LEN,
                              aad, aad_len,
                              buf, buf_size,
                              pt, ct_len,
                              &out_len);

    // 5. Dọn dẹp
    free(buf);
    psa_destroy_key(key_id);
    return (int)status;
}

int crypto_aead_encrypt(uint8_t algo, const uint8_t *key, const uint8_t *nonce,
                        const uint8_t *aad, size_t aad_len,
                        const uint8_t *pt, size_t pt_len,
                        uint8_t *ct, uint8_t *tag)
{
    switch (algo) {
        case CRYPTO_ALGO_AES:
            return aes_gcm_enc(key, nonce, aad, aad_len, pt, pt_len, ct, tag);
        case CRYPTO_ALGO_ASCON:
            ascon128_encrypt(ct, tag, pt, pt_len, aad, aad_len, nonce, key);
            return 0;
        case CRYPTO_ALGO_NONE:
        default:
            memcpy(ct, pt, pt_len);
            memset(tag, 0, CRYPTO_TAG_LEN);
            return 0;
    }
}

int crypto_aead_decrypt(uint8_t algo, const uint8_t *key, const uint8_t *nonce,
                        const uint8_t *aad, size_t aad_len,
                        const uint8_t *ct, size_t ct_len,
                        const uint8_t *tag, uint8_t *pt)
{
    switch (algo) {
        case CRYPTO_ALGO_AES:
            return aes_gcm_dec(key, nonce, aad, aad_len, ct, ct_len, tag, pt);
        case CRYPTO_ALGO_ASCON:
            return ascon128_decrypt(pt, ct, ct_len, aad, aad_len, tag, nonce, key);
        case CRYPTO_ALGO_NONE:
        default:
            memcpy(pt, ct, ct_len);
            return 0;
    }
}
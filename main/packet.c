#include <string.h>
#include "packet.h"
#include "crypto.h"
#include "esp_random.h"

int packet_seal(const app_payload_t *pl, uint8_t algo, const uint8_t key[16],
                uint8_t *out, size_t *out_len)
{
    env_header_t *h = (env_header_t *)out;
    h->magic0 = ENV_MAGIC0;
    h->magic1 = ENV_MAGIC1;
    h->version = PKT_VERSION;
    h->algo = algo;
    h->node_id = pl->node_id;
    h->plen = (uint8_t)sizeof(app_payload_t);
    esp_fill_random(h->nonce, sizeof(h->nonce));

    /* AAD = 6 byte header dau (rang buoc toan ven header) */
    const uint8_t *aad = out;
    size_t aad_len = offsetof(env_header_t, nonce);

    uint8_t *ct = out + sizeof(env_header_t);
    uint8_t *tag = ct + h->plen;

    int r = crypto_aead_encrypt(algo, key, h->nonce, aad, aad_len,
                                (const uint8_t *)pl, sizeof(app_payload_t),
                                ct, tag);
    if (r != 0) {
        return r;
    }
    *out_len = sizeof(env_header_t) + h->plen + ENV_TAG_LEN;
    return 0;
}

int packet_open(const uint8_t *buf, size_t len, const uint8_t key[16],
                app_payload_t *pl_out, uint8_t *algo_out)
{
    if (len < sizeof(env_header_t) + ENV_TAG_LEN) {
        return -1;
    }
    const env_header_t *h = (const env_header_t *)buf;
    if (h->magic0 != ENV_MAGIC0 || h->magic1 != ENV_MAGIC1 || h->version != PKT_VERSION) {
        return -2;
    }
    if (h->plen != sizeof(app_payload_t)) {
        return -3;
    }
    if (len != sizeof(env_header_t) + h->plen + ENV_TAG_LEN) {
        return -4;
    }

    const uint8_t *aad = buf;
    size_t aad_len = offsetof(env_header_t, nonce);
    const uint8_t *ct = buf + sizeof(env_header_t);
    const uint8_t *tag = ct + h->plen;

    int r = crypto_aead_decrypt(h->algo, key, h->nonce, aad, aad_len,
                                ct, h->plen, tag, (uint8_t *)pl_out);
    if (r != 0) {
        return -5;   /* sai tag / gia mao / loi giai ma */
    }
    if (algo_out) {
        *algo_out = h->algo;
    }
    return 0;
}

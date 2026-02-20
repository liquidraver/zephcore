/*
 * SPDX-License-Identifier: Apache-2.0
 * TransportKeyStore - Transport key cache for region filtering
 */

#include "TransportKeyStore.h"
#include <string.h>
#include <mbedtls/sha256.h>
#include <mbedtls/md.h>

uint16_t TransportKey::calcTransportCode(const mesh::Packet* packet) const {
    /* HMAC-SHA256 using mbedTLS */
    uint8_t hmac[32];
    uint8_t type = packet->getPayloadType();

    /* Build message: type + payload */
    uint8_t msg[MAX_PACKET_PAYLOAD + 1];
    msg[0] = type;
    memcpy(&msg[1], packet->payload, packet->payload_len);
    size_t msg_len = 1 + packet->payload_len;

    /* Calculate HMAC-SHA256 */
    mbedtls_md_context_t ctx;
    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
    mbedtls_md_hmac_starts(&ctx, key, sizeof(key));
    mbedtls_md_hmac_update(&ctx, msg, msg_len);
    mbedtls_md_hmac_finish(&ctx, hmac);
    mbedtls_md_free(&ctx);

    /* Extract first 2 bytes as transport code */
    uint16_t code = (hmac[0] << 8) | hmac[1];
    if (code == 0) {       // reserve codes 0000 and FFFF
        code++;
    } else if (code == 0xFFFF) {
        code--;
    }
    return code;
}

bool TransportKey::isNull() const {
    for (size_t i = 0; i < sizeof(key); i++) {
        if (key[i]) return false;
    }
    return true;  // key is all zeroes
}

void TransportKeyStore::putCache(uint16_t id, const TransportKey& key) {
    if (num_cache < MAX_TKS_ENTRIES) {
        cache_ids[num_cache] = id;
        cache_keys[num_cache] = key;
        num_cache++;
    } else {
        // TODO: evict oldest cache entry
    }
}

void TransportKeyStore::getAutoKeyFor(uint16_t id, const char* name, TransportKey& dest) {
    // first, check cache
    for (int i = 0; i < num_cache; i++) {
        if (cache_ids[i] == id) {  // cache hit!
            dest = cache_keys[i];
            return;
        }
    }

    // calc key for publicly-known hashtag region name (SHA256 hash, first 16 bytes)
    uint8_t hash[32];
    mbedtls_sha256((const unsigned char*)name, strlen(name), hash, 0);
    memcpy(dest.key, hash, sizeof(dest.key));

    putCache(id, dest);
}

int TransportKeyStore::loadKeysFor(uint16_t id, TransportKey keys[], int max_num) {
    int n = 0;
    // first, check cache
    for (int i = 0; i < num_cache && n < max_num; i++) {
        if (cache_ids[i] == id) {
            keys[n++] = cache_keys[i];
        }
    }
    if (n > 0) return n;  // cache hit!

    // TODO: retrieve from hardware keystore

    // store in cache (if room)
    for (int i = 0; i < n; i++) {
        putCache(id, keys[i]);
    }
    return n;
}

bool TransportKeyStore::saveKeysFor(uint16_t id, const TransportKey keys[], int num) {
    invalidateCache();
    // TODO: update hardware keystore
    return false;  // failed
}

bool TransportKeyStore::removeKeys(uint16_t id) {
    invalidateCache();
    // TODO: remove from hardware keystore
    return false;  // failed
}

bool TransportKeyStore::clear() {
    invalidateCache();
    // TODO: clear hardware keystore
    return false;  // failed
}

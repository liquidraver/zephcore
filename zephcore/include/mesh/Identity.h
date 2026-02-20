/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore Identity - Ed25519 public identity
 */

#pragma once

#include <mesh/Utils.h>
#include <mesh/MeshCore.h>
#include <mesh/RNG.h>
#include <stddef.h>
#include <string.h>

namespace mesh {

class Identity {
public:
	uint8_t pub_key[PUB_KEY_SIZE];

	Identity();
	Identity(const char *pub_hex);
	Identity(const uint8_t *_pub) { memcpy(pub_key, _pub, PUB_KEY_SIZE); }

	int copyHashTo(uint8_t *dest) const {
		memcpy(dest, pub_key, PATH_HASH_SIZE);
		return PATH_HASH_SIZE;
	}
	bool isHashMatch(const uint8_t *hash) const {
		return memcmp(hash, pub_key, PATH_HASH_SIZE) == 0;
	}
	bool isHashMatch(const uint8_t *hash, uint8_t len) const {
		return memcmp(hash, pub_key, len) == 0;
	}
	bool verify(const uint8_t *sig, const uint8_t *message, int msg_len) const;
	bool matches(const Identity &other) const { return memcmp(pub_key, other.pub_key, PUB_KEY_SIZE) == 0; }
	bool matches(const uint8_t *other_pubkey) const { return memcmp(pub_key, other_pubkey, PUB_KEY_SIZE) == 0; }
	bool readFrom(const uint8_t *src, size_t len);
	bool writeTo(uint8_t *dest, size_t max_len) const;
};

class LocalIdentity : public Identity {
	uint8_t prv_key[PRV_KEY_SIZE];
public:
	LocalIdentity();
	LocalIdentity(const char *prv_hex, const char *pub_hex);
	LocalIdentity(RNG *rng);

	void sign(uint8_t *sig, const uint8_t *message, int msg_len) const;
	void calcSharedSecret(uint8_t *secret, const Identity &other) const { calcSharedSecret(secret, other.pub_key); }
	void calcSharedSecret(uint8_t *secret, const uint8_t *other_pub_key) const;
	static bool validatePrivateKey(const uint8_t prv[64]);

	bool readFrom(const uint8_t *src, size_t len);
	size_t writeTo(uint8_t *dest, size_t max_len) const;
};

} /* namespace mesh */

/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore Utils - crypto and helpers (Zephyr port)
 */

#pragma once

#include <mesh/MeshCore.h>
#include <string.h>
#include <stddef.h>

namespace mesh {

class Utils {
public:
	static void sha256(uint8_t *hash, size_t hash_len, const uint8_t *msg, int msg_len);
	static void sha256(uint8_t *hash, size_t hash_len, const uint8_t *frag1, int frag1_len, const uint8_t *frag2, int frag2_len);
	static int encrypt(const uint8_t *shared_secret, uint8_t *dest, const uint8_t *src, int src_len);
	static int decrypt(const uint8_t *shared_secret, uint8_t *dest, const uint8_t *src, int src_len);
	static int encryptThenMAC(const uint8_t *shared_secret, uint8_t *dest, const uint8_t *src, int src_len);
	static int MACThenDecrypt(const uint8_t *shared_secret, uint8_t *dest, const uint8_t *src, int src_len);
	static void toHex(char *dest, const uint8_t *src, size_t len);
	static bool fromHex(uint8_t *dest, int dest_size, const char *src_hex);
	static bool isHexChar(char c);
	static int parseTextParts(char *text, const char *parts[], int max_num, char separator = ',');
};

} /* namespace mesh */

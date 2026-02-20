/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore Packet implementation
 */

#include <mesh/Packet.h>
#include <mesh/Utils.h>
#include <string.h>

namespace mesh {

Packet::Packet()
{
	header = 0;
	path_len = 0;
	payload_len = 0;
}

int Packet::getRawLength() const
{
	return 2 + path_len + payload_len + (hasTransportCodes() ? 4 : 0);
}

void Packet::calculatePacketHash(uint8_t *hash) const
{
	uint8_t t = getPayloadType();
	if (t == PAYLOAD_TYPE_TRACE) {
		uint8_t buf[2 + MAX_PACKET_PAYLOAD];
		buf[0] = t;
		memcpy(buf + 1, &path_len, sizeof(path_len));
		memcpy(buf + 2, payload, payload_len);
		Utils::sha256(hash, MAX_HASH_SIZE, buf, 2 + payload_len);
	} else {
		uint8_t buf[1 + MAX_PACKET_PAYLOAD];
		buf[0] = t;
		memcpy(buf + 1, payload, payload_len);
		Utils::sha256(hash, MAX_HASH_SIZE, buf, 1 + payload_len);
	}
}

uint8_t Packet::writeTo(uint8_t dest[]) const
{
	uint8_t i = 0;
	dest[i++] = header;
	if (hasTransportCodes()) {
		memcpy(&dest[i], &transport_codes[0], 2); i += 2;
		memcpy(&dest[i], &transport_codes[1], 2); i += 2;
	}
	dest[i++] = path_len;
	memcpy(&dest[i], path, path_len); i += path_len;
	memcpy(&dest[i], payload, payload_len); i += payload_len;
	return i;
}

bool Packet::readFrom(const uint8_t src[], uint8_t len)
{
	uint8_t i = 0;
	header = src[i++];
	if (hasTransportCodes()) {
		memcpy(&transport_codes[0], &src[i], 2); i += 2;
		memcpy(&transport_codes[1], &src[i], 2); i += 2;
	} else {
		transport_codes[0] = transport_codes[1] = 0;
	}
	path_len = src[i++];
	if (path_len > sizeof(path)) return false;
	memcpy(path, &src[i], path_len); i += path_len;
	if (i >= len) return false;
	payload_len = len - i;
	if (payload_len > sizeof(payload)) return false;
	memcpy(payload, &src[i], payload_len);
	return true;
}

} /* namespace mesh */

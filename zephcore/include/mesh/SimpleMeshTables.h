/*
 * SPDX-License-Identifier: Apache-2.0
 * SimpleMeshTables - hash-based deduplication
 */

#pragma once

#include <mesh/Mesh.h>
#include <string.h>

namespace mesh {

#define MAX_PACKET_HASHES  128
#define MAX_PACKET_ACKS     64

class SimpleMeshTables : public MeshTables {
	uint8_t _hashes[MAX_PACKET_HASHES * MAX_HASH_SIZE];
	int _next_idx;
	uint32_t _acks[MAX_PACKET_ACKS];
	int _next_ack_idx;
	uint32_t _direct_dups, _flood_dups;

public:
	SimpleMeshTables() {
		memset(_hashes, 0, sizeof(_hashes));
		_next_idx = 0;
		memset(_acks, 0, sizeof(_acks));
		_next_ack_idx = 0;
		_direct_dups = _flood_dups = 0;
	}

	bool hasSeen(const Packet *packet) override {
		if (packet->getPayloadType() == PAYLOAD_TYPE_ACK) {
			uint32_t ack;
			memcpy(&ack, packet->payload, 4);
			for (int i = 0; i < MAX_PACKET_ACKS; i++) {
				if (ack == _acks[i]) {
					if (packet->isRouteDirect()) {
						_direct_dups++;
					} else {
						_flood_dups++;
					}
					return true;
				}
			}
			_acks[_next_ack_idx] = ack;
			_next_ack_idx = (_next_ack_idx + 1) % MAX_PACKET_ACKS;
			return false;
		}

		uint8_t hash[MAX_HASH_SIZE];
		packet->calculatePacketHash(hash);
		const uint8_t *sp = _hashes;
		for (int i = 0; i < MAX_PACKET_HASHES; i++, sp += MAX_HASH_SIZE) {
			if (memcmp(hash, sp, MAX_HASH_SIZE) == 0) {
				if (packet->isRouteDirect()) {
					_direct_dups++;
				} else {
					_flood_dups++;
				}
				return true;
			}
		}
		memcpy(&_hashes[_next_idx * MAX_HASH_SIZE], hash, MAX_HASH_SIZE);
		_next_idx = (_next_idx + 1) % MAX_PACKET_HASHES;
		return false;
	}

	void clear(const Packet *packet) override {
		if (packet->getPayloadType() == PAYLOAD_TYPE_ACK) {
			uint32_t ack;
			memcpy(&ack, packet->payload, 4);
			for (int i = 0; i < MAX_PACKET_ACKS; i++) {
				if (ack == _acks[i]) {
					_acks[i] = 0;
					break;
				}
			}
		} else {
			uint8_t hash[MAX_HASH_SIZE];
			packet->calculatePacketHash(hash);
			uint8_t *sp = _hashes;
			for (int i = 0; i < MAX_PACKET_HASHES; i++, sp += MAX_HASH_SIZE) {
				if (memcmp(hash, sp, MAX_HASH_SIZE) == 0) {
					memset(sp, 0, MAX_HASH_SIZE);
					break;
				}
			}
		}
	}

	uint32_t getNumDirectDups() const { return _direct_dups; }
	uint32_t getNumFloodDups() const { return _flood_dups; }

	void resetStats() { _direct_dups = _flood_dups = 0; }
};

} /* namespace mesh */

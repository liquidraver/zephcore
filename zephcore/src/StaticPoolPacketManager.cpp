/*
 * SPDX-License-Identifier: Apache-2.0
 * Static pool PacketManager - no dynamic allocation
 */

#include <mesh/StaticPoolPacketManager.h>
#include <mesh/Packet.h>
#include <string.h>

namespace mesh {

#define POOL_SIZE 16
#define QUEUE_SIZE 16

struct PacketQueue {
	Packet *_table[QUEUE_SIZE];
	uint8_t _pri_table[QUEUE_SIZE];
	uint32_t _schedule_table[QUEUE_SIZE];
	int _num;

	PacketQueue() : _num(0) {
		memset(_table, 0, sizeof(_table));
	}

	int countBefore(uint32_t now) const {
		int n = 0;
		for (int j = 0; j < _num; j++) {
			if (_schedule_table[j] > now) continue;
			n++;
		}
		return n;
	}

	Packet *get(uint32_t now) {
		uint8_t min_pri = 0xFF;
		int best_idx = -1;
		for (int j = 0; j < _num; j++) {
			if (_schedule_table[j] > now) continue;
			if (_pri_table[j] < min_pri) {
				min_pri = _pri_table[j];
				best_idx = j;
			}
		}
		if (best_idx < 0) return nullptr;

		Packet *top = _table[best_idx];
		for (int i = best_idx; i < _num - 1; i++) {
			_table[i] = _table[i + 1];
			_pri_table[i] = _pri_table[i + 1];
			_schedule_table[i] = _schedule_table[i + 1];
		}
		_num--;
		return top;
	}

	Packet *removeByIdx(int i) {
		if (i >= _num) return nullptr;
		Packet *item = _table[i];
		for (int j = i; j < _num - 1; j++) {
			_table[j] = _table[j + 1];
			_pri_table[j] = _pri_table[j + 1];
			_schedule_table[j] = _schedule_table[j + 1];
		}
		_num--;
		return item;
	}

	bool add(Packet *packet, uint8_t priority, uint32_t scheduled_for) {
		if (_num >= QUEUE_SIZE) return false;
		_table[_num] = packet;
		_pri_table[_num] = priority;
		_schedule_table[_num] = scheduled_for;
		_num++;
		return true;
	}

	int count() const { return _num; }
	Packet *itemAt(int i) const { return (i < _num) ? _table[i] : nullptr; }
};

static Packet _packet_pool[POOL_SIZE];
static PacketQueue _unused;
static PacketQueue _send_queue;
static PacketQueue _rx_queue;
static bool _initialized = false;

static void init_pool() {
	if (_initialized) return;
	_initialized = true;
	for (int i = 0; i < POOL_SIZE; i++) {
		_unused.add(&_packet_pool[i], 0, 0);
	}
}

Packet *StaticPoolPacketManager::allocNew()
{
	init_pool();
	return _unused.removeByIdx(0);
}

void StaticPoolPacketManager::free(Packet *packet)
{
	if (packet) _unused.add(packet, 0, 0);
}

void StaticPoolPacketManager::queueOutbound(Packet *packet, uint8_t priority, uint32_t scheduled_for)
{
	if (!_send_queue.add(packet, priority, scheduled_for)) {
		free(packet);
	}
}

Packet *StaticPoolPacketManager::getNextOutbound(uint32_t now)
{
	return _send_queue.get(now);
}

int StaticPoolPacketManager::getOutboundCount(uint32_t now) const
{
	return _send_queue.countBefore(now);
}

int StaticPoolPacketManager::getFreeCount() const
{
	return _unused.count();
}

Packet *StaticPoolPacketManager::getOutboundByIdx(int i)
{
	return _send_queue.itemAt(i);
}

Packet *StaticPoolPacketManager::removeOutboundByIdx(int i)
{
	return _send_queue.removeByIdx(i);
}

void StaticPoolPacketManager::queueInbound(Packet *packet, uint32_t scheduled_for)
{
	if (!_rx_queue.add(packet, 0, scheduled_for)) {
		free(packet);
	}
}

Packet *StaticPoolPacketManager::getNextInbound(uint32_t now)
{
	return _rx_queue.get(now);
}

} /* namespace mesh */

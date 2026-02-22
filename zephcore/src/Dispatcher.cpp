/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore Dispatcher implementation
 */

#include <mesh/Dispatcher.h>
#include <mesh/MeshCore.h>
#include <mesh/Utils.h>
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_dispatcher, CONFIG_ZEPHCORE_LORA_LOG_LEVEL);

#if IS_ENABLED(CONFIG_ZEPHCORE_PACKET_LOGGING)
/* Payload types for [src -> dest] logging */
#define PAYLOAD_TYPE_REQ         0x00
#define PAYLOAD_TYPE_RESPONSE    0x01
#define PAYLOAD_TYPE_TXT_MSG     0x02
#define PAYLOAD_TYPE_PATH        0x08
#endif

namespace mesh {

#define MAX_RX_DELAY_MILLIS   32000
#define NOISE_FLOOR_CALIB_INTERVAL 2000

Dispatcher::Dispatcher(Radio &radio, MillisecondClock &ms, PacketManager &mgr)
	: _radio(&radio), _ms(&ms), _mgr(&mgr)
{
	outbound = nullptr;
	total_air_time = rx_air_time = 0;
	next_tx_time = 0;
	cad_busy_start = 0;
	next_floor_calib_time = next_agc_reset_time = 0;
	_err_flags = 0;
	_duty_cycle.init(0);
	radio_nonrx_start = 0;
	prev_isrecv_mode = true;
	n_sent_flood = n_sent_direct = 0;
	n_recv_flood = n_recv_direct = 0;
	_tx_queued_cb = nullptr;
	_tx_queued_user_data = nullptr;
}

void Dispatcher::begin()
{
	n_sent_flood = n_sent_direct = 0;
	n_recv_flood = n_recv_direct = 0;
	_err_flags = 0;
	radio_nonrx_start = (uint32_t)_ms->getMillis();
	_radio->begin();
	prev_isrecv_mode = _radio->isInRecvMode();
	_duty_cycle.init(getDutyCyclePercent());
}

uint8_t Dispatcher::getDutyCyclePercent() const
{
	return 10;
}

bool Dispatcher::isAdminPacket(const Packet *pkt)
{
	uint8_t t = pkt->getPayloadType();
	return t == PAYLOAD_TYPE_REQ || t == PAYLOAD_TYPE_RESPONSE ||
	       t == PAYLOAD_TYPE_ANON_REQ || t == PAYLOAD_TYPE_CONTROL;
}

int Dispatcher::calcRxDelay(float score, uint32_t air_time) const
{
	/* Lookup table: (10^(0.85 - i*0.1) - 1) for i=0..10 (score 0.0 to 1.0)
	 * Replaces powf() to save ~1.9KB flash. Linear interpolation between entries. */
	static const float lut[11] = {
		6.0793f, 4.6236f, 3.4674f, 2.5489f, 1.8184f, 1.2389f,
		0.7783f, 0.4125f, 0.1220f, -0.1089f, -0.2921f
	};
	if (score <= 0.0f) return (int)(lut[0] * (float)air_time);
	if (score >= 1.0f) return (int)(lut[10] * (float)air_time);
	float idx = score * 10.0f;
	int i = (int)idx;
	float frac = idx - (float)i;
	float val = lut[i] + frac * (lut[i + 1] - lut[i]);
	return (int)(val * (float)air_time);
}

uint32_t Dispatcher::getCADFailRetryDelay() const
{
	return 200;
}

uint32_t Dispatcher::getCADFailMaxDuration() const
{
	return 4000;
}

void Dispatcher::loop()
{
	if (millisHasNowPassed(next_floor_calib_time)) {
		_radio->triggerNoiseFloorCalibrate(getInterferenceThreshold());
		next_floor_calib_time = futureMillis(NOISE_FLOOR_CALIB_INTERVAL);
	}

	bool is_recv = _radio->isInRecvMode();
	if (is_recv != prev_isrecv_mode) {
		prev_isrecv_mode = is_recv;
		if (!is_recv) {
			radio_nonrx_start = (uint32_t)_ms->getMillis();
		}
	}
	if (!is_recv && (uint32_t)_ms->getMillis() - radio_nonrx_start > 8000) {
		_err_flags |= ERR_EVENT_STARTRX_TIMEOUT;
	}

	if (outbound) {
		if (_radio->isSendComplete()) {
			uint32_t t = (uint32_t)_ms->getMillis() - outbound_start;
			total_air_time += t;
			_duty_cycle.recordTx(t, (uint32_t)_ms->getMillis());
			_radio->onSendFinished();
			logTx(outbound, 2 + outbound->path_len + outbound->payload_len);
			if (outbound->isRouteFlood()) {
				n_sent_flood++;
			} else {
				n_sent_direct++;
			}
			releasePacket(outbound);
			outbound = nullptr;
		} else if (millisHasNowPassed(outbound_expiry)) {
			_radio->onSendFinished();
			logTxFail(outbound, 2 + outbound->path_len + outbound->payload_len);
			releasePacket(outbound);
			outbound = nullptr;
		} else {
			return;
		}
		next_agc_reset_time = futureMillis(getAGCResetInterval());
	}

	if (getAGCResetInterval() > 0 && millisHasNowPassed(next_agc_reset_time)) {
		_radio->resetAGC();
		next_agc_reset_time = futureMillis(getAGCResetInterval());
	}

	{
		Packet *pkt = _mgr->getNextInbound((uint32_t)_ms->getMillis());
		if (pkt) {
			processRecvPacket(pkt);
		}
	}
	checkRecv();
	checkSend();
}

void Dispatcher::checkRecv()
{
	/* Drain ALL queued LoRa packets per wake.
	 * k_event is a bitfield (not a counter), so multiple ISR arrivals
	 * may only produce one wake.  We must empty the ring each time. */
	for (;;) {
		uint8_t raw[MAX_TRANS_UNIT + 1];
		int len = _radio->recvRaw(raw, MAX_TRANS_UNIT);
		if (len <= 0) {
			break;  /* ring empty — done */
		}

		LOG_INF("checkRecv: got raw packet len=%d", len);
		logRxRaw(_radio->getLastSNR(), _radio->getLastRSSI(), raw, len);

		Packet *pkt = _mgr->allocNew();
		if (pkt == nullptr) {
			LOG_WRN("checkRecv: packet alloc failed");
			break;
		}

		float score = 0.0f;
		uint32_t air_time = 0;

		LOG_INF("checkRecv: parsing packet");
		int i = 0;
		pkt->header = raw[i++];
		if (pkt->hasTransportCodes()) {
			memcpy(&pkt->transport_codes[0], &raw[i], 2); i += 2;
			memcpy(&pkt->transport_codes[1], &raw[i], 2); i += 2;
		} else {
			pkt->transport_codes[0] = pkt->transport_codes[1] = 0;
		}
		pkt->path_len = raw[i++];

		if (pkt->path_len > MAX_PATH_SIZE || i + pkt->path_len > len) {
			LOG_WRN("checkRecv: bad path_len=%d", pkt->path_len);
			_mgr->free(pkt);
			continue;
		}

		memcpy(pkt->path, &raw[i], pkt->path_len);
		i += pkt->path_len;
		pkt->payload_len = len - i;
		if (pkt->payload_len > (int)sizeof(pkt->payload)) {
			LOG_WRN("checkRecv: payload too large %d", pkt->payload_len);
			_mgr->free(pkt);
			continue;
		}

		memcpy(pkt->payload, &raw[i], pkt->payload_len);
		pkt->_snr = (int8_t)(_radio->getLastSNR() * 4.0f);
		score = _radio->packetScore(_radio->getLastSNR(), len);
		air_time = _radio->getEstAirtimeFor(len);
		rx_air_time += air_time;
		LOG_INF("checkRecv: header=0x%02x type=%d route=%s path_len=%d payload_len=%d",
			pkt->header, pkt->getPayloadType(),
			pkt->isRouteDirect() ? "direct" : "flood",
			pkt->path_len, pkt->payload_len);
		/* Log path hashes to identify forwarding nodes */
		if (pkt->path_len > 0) {
			LOG_INF("checkRecv: path[0]=0x%02x%s%s",
				pkt->path[0],
				pkt->path_len > 1 ? " path[1]=0x" : "",
				pkt->path_len > 1 ? "" : "");
			if (pkt->path_len > 1) {
				LOG_INF("  path bytes: %02x %02x %02x %02x",
					pkt->path[0],
					pkt->path_len > 1 ? pkt->path[1] : 0,
					pkt->path_len > 2 ? pkt->path[2] : 0,
					pkt->path_len > 3 ? pkt->path[3] : 0);
			}
		}

#if IS_ENABLED(CONFIG_ZEPHCORE_PACKET_LOGGING)
		/* Arduino-compatible packet logging - use printk to bypass log level filtering */
		{
			static uint8_t packet_hash[MAX_HASH_SIZE];
			static char hash_hex[MAX_HASH_SIZE * 2 + 1];
			pkt->calculatePacketHash(packet_hash);
			Utils::toHex(hash_hex, packet_hash, MAX_HASH_SIZE);

			uint8_t ptype = pkt->getPayloadType();
			if (ptype == PAYLOAD_TYPE_PATH || ptype == PAYLOAD_TYPE_REQ ||
			    ptype == PAYLOAD_TYPE_RESPONSE || ptype == PAYLOAD_TYPE_TXT_MSG) {
				printk("%s: RX, len=%d (type=%d, route=%s, payload_len=%d) SNR=%d RSSI=%d score=%d time=%u hash=%s [%02X -> %02X]\n",
					getLogDateTime(), pkt->getRawLength(), ptype,
					pkt->isRouteDirect() ? "D" : "F", pkt->payload_len,
					(int)pkt->getSNR(), (int)_radio->getLastRSSI(),
					(int)(score * 1000), air_time, hash_hex,
					(uint32_t)pkt->payload[1], (uint32_t)pkt->payload[0]);
			} else {
				printk("%s: RX, len=%d (type=%d, route=%s, payload_len=%d) SNR=%d RSSI=%d score=%d time=%u hash=%s\n",
					getLogDateTime(), pkt->getRawLength(), ptype,
					pkt->isRouteDirect() ? "D" : "F", pkt->payload_len,
					(int)pkt->getSNR(), (int)_radio->getLastRSSI(),
					(int)(score * 1000), air_time, hash_hex);
			}
		}
#endif
		LOG_INF("checkRecv: processing packet");
		logRx(pkt, pkt->getRawLength(), score);
		if (pkt->isRouteFlood()) {
			n_recv_flood++;
			int delay = calcRxDelay(score, air_time);
			if (delay < 50) {
				LOG_INF("checkRecv: processing flood packet immediately");
				processRecvPacket(pkt);
			} else {
				if (delay > (int)MAX_RX_DELAY_MILLIS) delay = MAX_RX_DELAY_MILLIS;
				LOG_INF("checkRecv: queueing flood packet, delay=%d", delay);
				_mgr->queueInbound(pkt, futureMillis(delay));
			}
		} else {
			n_recv_direct++;
			LOG_INF("checkRecv: processing direct packet");
			processRecvPacket(pkt);
		}
		LOG_INF("checkRecv: done");
	}
}

void Dispatcher::processRecvPacket(Packet *pkt)
{
	DispatcherAction action = onRecvPacket(pkt);
	if (action == ACTION_RELEASE) {
		_mgr->free(pkt);
	} else if (action == ACTION_MANUAL_HOLD) {
		/* subclass holds packet */
	} else {
		uint8_t priority = (uint8_t)((action >> 24) - 1);
		uint32_t delay = action & 0xFFFFFF;
		_mgr->queueOutbound(pkt, priority, futureMillis((int)delay));
		if (_tx_queued_cb && delay > 0) {
			_tx_queued_cb(delay, _tx_queued_user_data);
		}
	}
}

void Dispatcher::checkSend()
{
	uint32_t now = (uint32_t)_ms->getMillis();
	int count = _mgr->getOutboundCount(now);
	if (count == 0) return;

	if (!millisHasNowPassed(next_tx_time)) {
		LOG_DBG("checkSend: waiting for tx_time (count=%d)", count);
		return;
	}
	if (_radio->isReceiving()) {
		if (cad_busy_start == 0) {
			cad_busy_start = now;
			LOG_INF("checkSend: channel busy, starting wait");
		}
		if (now - cad_busy_start > getCADFailMaxDuration()) {
			_err_flags |= ERR_EVENT_CAD_TIMEOUT;
			LOG_WRN("checkSend: CAD timeout exceeded");
		} else {
			next_tx_time = futureMillis((int)getCADFailRetryDelay());
			return;
		}
	}
	cad_busy_start = 0;

	outbound = _mgr->getNextOutbound(now);
	if (outbound) {
		/* Duty cycle enforcement — exempt admin packets */
		if (!isAdminPacket(outbound) && _duty_cycle.isExceeded(now)) {
			LOG_WRN("checkSend: duty cycle exceeded (%u/%u ms), re-queuing type=%d",
				_duty_cycle.window_airtime_ms, _duty_cycle.budgetMs(),
				outbound->getPayloadType());
			_mgr->queueOutbound(outbound, 0, futureMillis(5000));
			outbound = nullptr;
			return;
		}
		LOG_INF("checkSend: got outbound type=%d payload_len=%d", outbound->getPayloadType(), outbound->payload_len);
		uint8_t raw[MAX_TRANS_UNIT];
		int len = 0;
		raw[len++] = outbound->header;
		if (outbound->hasTransportCodes()) {
			memcpy(&raw[len], &outbound->transport_codes[0], 2); len += 2;
			memcpy(&raw[len], &outbound->transport_codes[1], 2); len += 2;
		}
		raw[len++] = outbound->path_len;
		memcpy(&raw[len], outbound->path, outbound->path_len);
		len += outbound->path_len;

		if (len + outbound->payload_len > MAX_TRANS_UNIT) {
			LOG_WRN("checkSend: packet too large len=%d+%d > %d", len, outbound->payload_len, MAX_TRANS_UNIT);
			_mgr->free(outbound);
			outbound = nullptr;
		} else {
			memcpy(&raw[len], outbound->payload, outbound->payload_len);
			len += outbound->payload_len;

			uint32_t max_airtime = _radio->getEstAirtimeFor(len) * 3 / 2;
			outbound_start = now;

#if IS_ENABLED(CONFIG_ZEPHCORE_PACKET_LOGGING)
			/* Arduino-compatible packet logging - use printk to bypass log level filtering */
			{
				uint8_t ptype = outbound->getPayloadType();
				if (ptype == PAYLOAD_TYPE_PATH || ptype == PAYLOAD_TYPE_REQ ||
				    ptype == PAYLOAD_TYPE_RESPONSE || ptype == PAYLOAD_TYPE_TXT_MSG) {
					printk("%s: TX, len=%d (type=%d, route=%s, payload_len=%d) [%02X -> %02X]\n",
						getLogDateTime(), len, ptype,
						outbound->isRouteDirect() ? "D" : "F", outbound->payload_len,
						(uint32_t)outbound->payload[1], (uint32_t)outbound->payload[0]);
				} else {
					printk("%s: TX, len=%d (type=%d, route=%s, payload_len=%d)\n",
						getLogDateTime(), len, ptype,
						outbound->isRouteDirect() ? "D" : "F", outbound->payload_len);
				}
			}
#endif

			/* Final LBT check — close the gap between initial
			 * isReceiving() and actual TX start (serialisation +
			 * logging can take 1-5 ms). */
			if (_radio->isReceiving()) {
				LOG_INF("checkSend: channel busy at TX commit, re-queuing");
				_mgr->queueOutbound(outbound, 0, futureMillis((int)getCADFailRetryDelay()));
				outbound = nullptr;
				return;
			}

			LOG_INF("checkSend: calling startSendRaw len=%d", len);
			bool success = _radio->startSendRaw(raw, len);
			if (!success) {
				LOG_ERR("checkSend: startSendRaw failed!");
				logTxFail(outbound, outbound->getRawLength());
				releasePacket(outbound);
				outbound = nullptr;
			} else {
				LOG_INF("checkSend: TX started, max_airtime=%u", max_airtime);
				outbound_expiry = futureMillis((int)max_airtime);
			}
		}
	}
}

Packet *Dispatcher::obtainNewPacket()
{
	Packet *pkt = _mgr->allocNew();
	if (pkt == nullptr) {
		_err_flags |= ERR_EVENT_FULL;
	} else {
		pkt->payload_len = pkt->path_len = 0;
		pkt->_snr = 0;
	}
	return pkt;
}

void Dispatcher::releasePacket(Packet *packet)
{
	_mgr->free(packet);
}

void Dispatcher::sendPacket(Packet *packet, uint8_t priority, uint32_t delay_millis)
{
	LOG_INF("sendPacket: type=%d payload_len=%d path_len=%d pri=%d delay=%u",
		packet->getPayloadType(), packet->payload_len, packet->path_len, priority, delay_millis);
	if (packet->path_len > MAX_PATH_SIZE || packet->payload_len > MAX_PACKET_PAYLOAD) {
		LOG_WRN("sendPacket: rejected - path_len=%d or payload_len=%d too large",
			packet->path_len, packet->payload_len);
		_mgr->free(packet);
	} else {
		_mgr->queueOutbound(packet, priority, futureMillis((int)delay_millis));
		LOG_INF("sendPacket: queued outbound count=%d", _mgr->getOutboundCount((uint32_t)_ms->getMillis()));
		if (_tx_queued_cb && delay_millis > 0) {
			_tx_queued_cb(delay_millis, _tx_queued_user_data);
		}
	}
}

bool Dispatcher::millisHasNowPassed(uint32_t timestamp) const
{
	return (int32_t)((uint32_t)_ms->getMillis() - timestamp) > 0;
}

uint32_t Dispatcher::futureMillis(int millis_from_now) const
{
	return (uint32_t)_ms->getMillis() + millis_from_now;
}

} /* namespace mesh */

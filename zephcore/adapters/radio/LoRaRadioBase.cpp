/*
 * SPDX-License-Identifier: Apache-2.0
 * LoRa radio base class — all shared algorithms.
 */

#include "LoRaRadioBase.h"
#include "radio_common.h"
#include <mesh/LoRaConfig.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <math.h>


#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lora_radio_base, CONFIG_ZEPHCORE_LORA_LOG_LEVEL);

namespace mesh {

/* ── Constructor ──────────────────────────────────────────────────────── */

LoRaRadioBase::LoRaRadioBase(const struct device *lora_dev, MainBoard &board,
			     NodePrefs *prefs)
	: _dev(lora_dev), _prefs(prefs), _board(&board),
	  _in_recv_mode(false), _tx_active(false),
	  _last_rssi(0), _last_snr(0),
	  _rx_head(0), _rx_tail(0),
	  _noise_floor(DEFAULT_NOISE_FLOOR), _calibration_threshold(0),
	  _rx_duty_cycle_enabled(IS_ENABLED(CONFIG_ZEPHCORE_LORA_RX_DUTY_CYCLE)),
	  _rx_boost_enabled(true),
	  _config_cached(false),
	  _rx_cb(nullptr), _rx_cb_user_data(nullptr),
	  _tx_done_cb(nullptr), _tx_done_cb_user_data(nullptr),
	  _tx_thread_running(false),
	  _packets_recv(0), _packets_sent(0), _packets_recv_errors(0)
{
	k_poll_signal_init(&_tx_signal);
	k_sem_init(&_tx_start_sem, 0, 1);
	memset(_rx_ring, 0, sizeof(_rx_ring));
}

/* ── TX wait thread ───────────────────────────────────────────────────── */

void LoRaRadioBase::txWaitThreadFn(void *p1, void *p2, void *p3)
{
	LoRaRadioBase *self = static_cast<LoRaRadioBase *>(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_INF("TX wait thread started");

	for (;;) {
		k_sem_take(&self->_tx_start_sem, K_FOREVER);

		if (!self->_tx_active) {
			continue;
		}

		LOG_DBG("TX wait: waiting for signal...");

		struct k_poll_event events[1] = {
			K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
						 K_POLL_MODE_NOTIFY_ONLY,
						 &self->_tx_signal),
		};

		/* Check if signal was already raised */
		unsigned int signaled;
		int result;
		k_poll_signal_check(&self->_tx_signal, &signaled, &result);
		if (signaled) {
			LOG_DBG("TX wait: signal already raised (result=%d)", result);
			k_poll_signal_reset(&self->_tx_signal);
			self->_board->onAfterTransmit();
			self->startReceive();
			self->_tx_active = false;
			self->_packets_sent++;
			if (self->_tx_done_cb) {
				self->_tx_done_cb(self->_tx_done_cb_user_data);
			}
			continue;
		}

		int ret = k_poll(events, 1, K_MSEC(TX_TIMEOUT_MS));
		if (ret == -EAGAIN) {
			LOG_ERR("TX wait: TIMEOUT!");
			self->_board->onAfterTransmit();
			self->startReceive();
			self->_tx_active = false;
			if (self->_tx_done_cb) {
				self->_tx_done_cb(self->_tx_done_cb_user_data);
			}
			continue;
		}

		if (ret == 0 && events[0].state == K_POLL_STATE_SIGNALED) {
			k_poll_signal_reset(&self->_tx_signal);
			self->_board->onAfterTransmit();
			self->startReceive();
			self->_tx_active = false;
			self->_packets_sent++;
			LOG_INF("TX complete, RX restarted");

			if (self->_tx_done_cb) {
				self->_tx_done_cb(self->_tx_done_cb_user_data);
			}
		} else {
			LOG_ERR("TX wait: k_poll returned %d, state=%d — recovering",
				ret, events[0].state);
			k_poll_signal_reset(&self->_tx_signal);
			self->_board->onAfterTransmit();
			self->startReceive();
			self->_tx_active = false;

			if (self->_tx_done_cb) {
				self->_tx_done_cb(self->_tx_done_cb_user_data);
			}
		}
	}
}

void LoRaRadioBase::startTxThread(k_thread_stack_t *stack, size_t stack_size)
{
	if (_tx_thread_running) {
		return;
	}
	k_thread_create(&_tx_wait_thread, stack, stack_size,
			txWaitThreadFn, this, NULL, NULL,
			TX_WAIT_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&_tx_wait_thread, "lora_tx_wait");
	_tx_thread_running = true;
}

/* ── RX callback (static, ISR-safe) ──────────────────────────────────── */

void LoRaRadioBase::rxCallbackStatic(const struct device *dev, uint8_t *data,
				     uint16_t size, int16_t rssi, int8_t snr,
				     void *user_data)
{
	LoRaRadioBase *self = static_cast<LoRaRadioBase *>(user_data);

	/* NULL data = RX error (CRC/header error) */
	if (data == NULL && size == 0) {
		self->_packets_recv_errors++;
		LOG_DBG("RX error (CRC/header), total errors: %u",
			self->_packets_recv_errors);
		/* Driver restarted with Radio.Rx(0) — restore duty cycle */
		if (self->_rx_duty_cycle_enabled) {
			self->hwSetRxDutyCycle(true);
		}
		return;
	}

	LOG_DBG("RX callback: size=%u rssi=%d snr=%d", size, rssi, snr);

	/* Ring buffer write */
	uint8_t next_head = (self->_rx_head + 1) % RX_RING_SIZE;
	if (next_head == self->_rx_tail) {
		LOG_WRN("RX ring full, dropping oldest packet");
		self->_rx_tail = (self->_rx_tail + 1) % RX_RING_SIZE;
	}

	RxPacket *pkt = &self->_rx_ring[self->_rx_head];
	uint16_t copy_len = (size > sizeof(pkt->data)) ? sizeof(pkt->data) : size;
	memcpy(pkt->data, data, copy_len);
	pkt->len = copy_len;
	pkt->rssi = rssi;
	pkt->snr = snr;

	self->_rx_head = next_head;
	self->_last_rssi = (float)rssi;
	self->_last_snr = (float)snr;
	self->_packets_recv++;

	/* Driver restarted with Radio.Rx(0) — restore duty cycle */
	if (self->_rx_duty_cycle_enabled) {
		self->hwSetRxDutyCycle(true);
	}

	if (self->_rx_cb) {
		self->_rx_cb(self->_rx_cb_user_data);
	}
}

/* ── Config helpers ───────────────────────────────────────────────────── */

void LoRaRadioBase::buildModemConfig(struct lora_modem_config &cfg, bool tx)
{
	memset(&cfg, 0, sizeof(cfg));
	cfg.frequency = _prefs ? (uint32_t)(_prefs->freq * 1000000.0f)
			       : LoRaConfig::FREQ_HZ;
	cfg.bandwidth = bw_khz_to_enum(
		_prefs ? (uint16_t)(_prefs->bw) : (uint16_t)LoRaConfig::BANDWIDTH);
	cfg.datarate = (enum lora_datarate)(
		_prefs ? _prefs->sf : LoRaConfig::SPREADING_FACTOR);
	cfg.coding_rate = cr_to_enum(
		_prefs ? _prefs->cr : LoRaConfig::CODING_RATE);
	cfg.preamble_len = LoRaConfig::PREAMBLE_LEN;
	cfg.tx_power = _prefs ? (int8_t)_prefs->tx_power_dbm
			      : LoRaConfig::TX_POWER_DBM;
#ifdef CONFIG_ZEPHCORE_MAX_TX_POWER_DBM
	if (cfg.tx_power > CONFIG_ZEPHCORE_MAX_TX_POWER_DBM) {
		cfg.tx_power = CONFIG_ZEPHCORE_MAX_TX_POWER_DBM;
	}
#endif
	cfg.tx = tx;
	cfg.iq_inverted = false;
	cfg.public_network = false;
	cfg.packet_crc_disable = false;
}

/**
 * Compare radio-relevant fields of two modem configs.
 * Ignores the tx flag — that only selects TX vs RX mode, the actual
 * modem parameters (freq, SF, BW, CR, power) are what the driver
 * programs into registers.
 */
static bool configParamsEqual(const struct lora_modem_config &a,
			      const struct lora_modem_config &b)
{
	/* CRITICAL: a.tx == b.tx MUST be compared — without it, switching
	 * RX→TX skips RadioSetTxConfig(), leaving TxTimeout=0 which causes
	 * the loramac-node software timeout to fire after 1ms and break TX. */
	return a.frequency == b.frequency &&
	       a.bandwidth == b.bandwidth &&
	       a.datarate == b.datarate &&
	       a.coding_rate == b.coding_rate &&
	       a.preamble_len == b.preamble_len &&
	       a.tx_power == b.tx_power &&
	       a.tx == b.tx &&
	       a.iq_inverted == b.iq_inverted &&
	       a.public_network == b.public_network;
}

void LoRaRadioBase::configureRx()
{
	struct lora_modem_config cfg;
	buildModemConfig(cfg, false);

	if (_config_cached && configParamsEqual(cfg, _last_cfg)) {
		LOG_DBG("configureRx: params unchanged, skipping hwConfigure");
		return;
	}

	LOG_DBG("configureRx: freq=%u bw=%d sf=%d cr=%d pwr=%d",
		cfg.frequency, (int)cfg.bandwidth, (int)cfg.datarate,
		(int)cfg.coding_rate, cfg.tx_power);

	hwConfigure(cfg);
	_last_cfg = cfg;
	_config_cached = true;
}

void LoRaRadioBase::configureTx()
{
	struct lora_modem_config cfg;
	buildModemConfig(cfg, true);

	if (_config_cached && configParamsEqual(cfg, _last_cfg)) {
		LOG_DBG("configureTx: params unchanged, skipping hwConfigure");
		return;
	}

	hwConfigure(cfg);
	_last_cfg = cfg;
	_config_cached = true;
}

/* ── Lifecycle ────────────────────────────────────────────────────────── */

void LoRaRadioBase::begin()
{
	if (!device_is_ready(_dev)) {
		LOG_ERR("LoRa device not ready");
		return;
	}

	/* Subclass begin() calls startTxThread() before calling us.
	 *
	 * RX boost and duty cycle are set via constructor defaults:
	 *   _rx_boost_enabled = true (boosted +3dB, overridable via setRxBoost())
	 *   _rx_duty_cycle_enabled = CONFIG_ZEPHCORE_LORA_RX_DUTY_CYCLE
	 * Callers can override after begin() via setRxBoost() / enableRxDutyCycle().
	 */

	startReceive();

	uint32_t freq = _prefs ? (uint32_t)(_prefs->freq * 1000000.0f)
			       : LoRaConfig::FREQ_HZ;
	uint8_t sf = _prefs ? _prefs->sf : LoRaConfig::SPREADING_FACTOR;
	uint16_t bw_khz = _prefs ? (uint16_t)(_prefs->bw)
				 : (uint16_t)LoRaConfig::BANDWIDTH;
	uint8_t cr = _prefs ? _prefs->cr : LoRaConfig::CODING_RATE;
	int8_t tx_pwr = _prefs ? (int8_t)_prefs->tx_power_dbm
			       : LoRaConfig::TX_POWER_DBM;

	LOG_INF("radio started: freq=%u bw=%u sf=%u cr=%u pwr=%d",
		freq, bw_khz, sf, cr, tx_pwr);
}

void LoRaRadioBase::reconfigure()
{
	hwCancelReceive();
	_in_recv_mode = false;
	_config_cached = false;  /* Force full reconfigure */
	startReceive();

	uint32_t freq = _prefs ? (uint32_t)(_prefs->freq * 1000000.0f)
			       : LoRaConfig::FREQ_HZ;
	uint8_t sf = _prefs ? _prefs->sf : LoRaConfig::SPREADING_FACTOR;
	uint16_t bw_khz = _prefs ? (uint16_t)(_prefs->bw)
				 : (uint16_t)LoRaConfig::BANDWIDTH;
	uint8_t cr = _prefs ? _prefs->cr : LoRaConfig::CODING_RATE;
	int8_t tx_pwr = _prefs ? (int8_t)_prefs->tx_power_dbm
			       : LoRaConfig::TX_POWER_DBM;

	LOG_INF("radio reconfigured: freq=%u bw=%u sf=%u cr=%u pwr=%d",
		freq, bw_khz, sf, cr, tx_pwr);
}

void LoRaRadioBase::reconfigureWithParams(float freq, float bw, uint8_t sf, uint8_t cr)
{
	if (_prefs) {
		_prefs->freq = freq;
		_prefs->bw = bw;
		_prefs->sf = sf;
		_prefs->cr = cr;
	}
	reconfigure();
}

void LoRaRadioBase::startReceive()
{
	configureRx();
	hwStartReceive();
}

/* ── RX/TX ────────────────────────────────────────────────────────────── */

int LoRaRadioBase::recvRaw(uint8_t *bytes, int sz)
{
	if (_rx_head == _rx_tail) {
		return 0;
	}

	RxPacket *pkt = &_rx_ring[_rx_tail];
	uint16_t len = pkt->len;
	if (len > (uint16_t)sz) {
		len = (uint16_t)sz;
	}

	memcpy(bytes, pkt->data, len);
	_last_rssi = (float)pkt->rssi;
	_last_snr = (float)pkt->snr;
	_rx_tail = (_rx_tail + 1) % RX_RING_SIZE;
	return (int)len;
}

bool LoRaRadioBase::startSendRaw(const uint8_t *bytes, int len)
{
	if (len > (int)sizeof(_tx_buf)) {
		return false;
	}

	_board->onBeforeTransmit();
	_tx_active = true;
	_in_recv_mode = false;

	hwCancelReceive();
	configureTx();

	memcpy(_tx_buf, bytes, len);
	k_poll_signal_reset(&_tx_signal);

	int ret = hwSendAsync(_tx_buf, (uint32_t)len, &_tx_signal);
	if (ret < 0) {
		LOG_ERR("hwSendAsync failed: %d", ret);
		_board->onAfterTransmit();
		_tx_active = false;
		startReceive();
		return false;
	}

	LOG_DBG("TX started async, len=%d", len);
	k_sem_give(&_tx_start_sem);
	return true;
}

bool LoRaRadioBase::isSendComplete()
{
	return !_tx_active;
}

void LoRaRadioBase::onSendFinished()
{
	/* Nothing needed — TX state tracked via _tx_active */
}

bool LoRaRadioBase::isInRecvMode() const
{
	return _in_recv_mode;
}

float LoRaRadioBase::getLastRSSI() const
{
	return _last_rssi;
}

float LoRaRadioBase::getLastSNR() const
{
	return _last_snr;
}

/* ── Airtime + scoring ────────────────────────────────────────────────── */

uint32_t LoRaRadioBase::getEstAirtimeFor(int len_bytes)
{
	uint8_t sf = _prefs ? _prefs->sf : LoRaConfig::SPREADING_FACTOR;
	float bw = _prefs ? _prefs->bw : (float)LoRaConfig::BANDWIDTH;
	uint8_t cr_val = _prefs ? _prefs->cr : LoRaConfig::CODING_RATE;

	if (sf < 6) sf = 6;
	if (sf > 12) sf = 12;
	if (bw < 7.0f) bw = 125.0f;
	if (cr_val < 5) cr_val = 5;
	if (cr_val > 8) cr_val = 8;

	float t_sym = (float)(1 << sf) / (bw * 1000.0f);
	float t_preamble = (LoRaConfig::PREAMBLE_LEN + 4.25f) * t_sym;

	float de = (sf >= 11) ? 1.0f : 0.0f;
	float num = 8.0f * len_bytes - 4.0f * sf + 28.0f + 16.0f;
	float den = 4.0f * (sf - 2.0f * de);
	if (den < 1.0f) den = 4.0f;
	float n_payload = 8.0f + fmaxf(ceilf(num / den) * (cr_val - 4 + 4), 0.0f);

	float t_payload = n_payload * t_sym;
	return (uint32_t)((t_preamble + t_payload) * 1000.0f);
}

float LoRaRadioBase::packetScore(float snr, int packet_len)
{
	int sf = _prefs ? _prefs->sf : LoRaConfig::SPREADING_FACTOR;
	if (sf < 7 || sf > 12) return 0.0f;
	if (snr < lora_snr_threshold[sf - 7]) return 0.0f;

	float success_rate = (snr - lora_snr_threshold[sf - 7]) / 10.0f;
	float collision_penalty = 1.0f - ((float)packet_len / 256.0f);
	float score = success_rate * collision_penalty;
	if (score < 0.0f) score = 0.0f;
	if (score > 1.0f) score = 1.0f;
	return score;
}

/* ── Advanced radio features ──────────────────────────────────────────── */

int LoRaRadioBase::getNoiseFloor() const
{
	return _noise_floor;
}

void LoRaRadioBase::triggerNoiseFloorCalibrate(int threshold)
{
	_calibration_threshold = threshold;

	if (!_in_recv_mode || _tx_active) {
		return;
	}

	/* RX duty cycle: radio alternates HW RX/sleep autonomously.
	 * During sleep windows hwGetCurrentRSSI() returns garbage (0 or very
	 * high) — the sampling filter rejects those, and the count>=32
	 * threshold ensures we only update when enough valid samples exist. */

	int sum = 0;
	int count = 0;
	for (int i = 0; i < NUM_NOISE_FLOOR_SAMPLES; i++) {
		if (isReceiving()) {
			break;
		}
		int16_t rssi = hwGetCurrentRSSI();
		if (rssi < _noise_floor + NOISE_FLOOR_SAMPLING_THRESHOLD) {
			sum += rssi;
			count++;
		}
	}

	if (count >= NUM_NOISE_FLOOR_SAMPLES / 2) {
		_noise_floor = sum / count;
		if (_noise_floor < -120) _noise_floor = -120;
		if (_noise_floor > -50) _noise_floor = -50;
		LOG_DBG("noise floor: %d dBm (%d samples)", _noise_floor, count);
	}
}

void LoRaRadioBase::resetAGC()
{
	hwResetAGC();
}

bool LoRaRadioBase::isReceiving()
{
	if (!_in_recv_mode || _tx_active) {
		return false;
	}
	if (hwIsPreambleDetected()) {
		return true;
	}
	return isChannelActive();
}

bool LoRaRadioBase::isChannelActive(int threshold)
{
	if (threshold == 0) {
		threshold = _calibration_threshold;
	}
	if (threshold == 0) {
		return false;
	}
	int16_t rssi = hwGetCurrentRSSI();
	return rssi > (_noise_floor + threshold);
}

/* ── Power saving ─────────────────────────────────────────────────────── */

void LoRaRadioBase::enableRxDutyCycle(bool enable)
{
	_rx_duty_cycle_enabled = enable;
	LOG_INF("RX duty cycle %s", enable ? "enabled" : "disabled");
	if (_in_recv_mode) {
		hwSetRxDutyCycle(enable);
	}
}

void LoRaRadioBase::setRxBoost(bool enable)
{
	_rx_boost_enabled = enable;
	LOG_INF("RX boost %s (+3dB sensitivity, +2mA)",
		enable ? "enabled" : "disabled");
	if (_in_recv_mode) {
		hwSetRxBoost(enable);
	}
}

} /* namespace mesh */

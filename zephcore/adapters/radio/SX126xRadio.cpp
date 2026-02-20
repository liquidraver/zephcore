/*
 * SPDX-License-Identifier: Apache-2.0
 * SX126x hardware hooks for LoRaRadioBase.
 */

#include "SX126xRadio.h"
#include <mesh/LoRaConfig.h>
#include <zephyr/kernel.h>

/*
 * Access loramac-node SX126x functions for advanced features.
 * These are defined in the Zephyr LoRa driver (sx126x.c/radio.c).
 */
extern "C" {
uint16_t SX126xGetIrqStatus(void);
int8_t SX126xGetRssiInst(void);
uint32_t SX126xGetRandom(void);

#define RADIO_CALIBRATEIMAGE 0x98
void SX126xWriteCommand(uint8_t opcode, uint8_t *buffer, uint16_t size);

/* Radio struct for direct access to SetRxDutyCycle (SX126x only) */
struct LoRaMacRadio_s {
	void (*Init)(void *events);
	int (*GetStatus)(void);
	void (*SetModem)(int modem);
	void (*SetChannel)(uint32_t freq);
	bool (*IsChannelFree)(uint32_t freq, uint32_t rxBandwidth, int16_t rssiThresh, uint32_t maxCarrierSenseTime);
	uint32_t (*Random)(void);
	void (*SetRxConfig)(int modem, uint32_t bandwidth, uint32_t datarate, uint8_t coderate,
			    uint32_t bandwidthAfc, uint16_t preambleLen, uint16_t symbTimeout,
			    bool fixLen, uint8_t payloadLen, bool crcOn, bool freqHopOn,
			    uint8_t hopPeriod, bool iqInverted, bool rxContinuous);
	void (*SetTxConfig)(int modem, int8_t power, uint32_t fdev, uint32_t bandwidth,
			    uint32_t datarate, uint8_t coderate, uint16_t preambleLen,
			    bool fixLen, bool crcOn, bool freqHopOn, uint8_t hopPeriod,
			    bool iqInverted, uint32_t timeout);
	bool (*CheckRfFrequency)(uint32_t frequency);
	uint32_t (*TimeOnAir)(int modem, uint32_t bandwidth, uint32_t datarate, uint8_t coderate,
			      uint16_t preambleLen, bool fixLen, uint8_t payloadLen, bool crcOn);
	void (*Send)(uint8_t *buffer, uint8_t size);
	void (*Sleep)(void);
	void (*Standby)(void);
	void (*Rx)(uint32_t timeout);
	void (*StartCad)(void);
	void (*SetTxContinuousWave)(uint32_t freq, int8_t power, uint16_t time);
	int16_t (*Rssi)(int modem);
	void (*Write)(uint32_t addr, uint8_t data);
	uint8_t (*Read)(uint32_t addr);
	void (*WriteBuffer)(uint32_t addr, uint8_t *buffer, uint8_t size);
	void (*ReadBuffer)(uint32_t addr, uint8_t *buffer, uint8_t size);
	void (*SetMaxPayloadLength)(int modem, uint8_t max);
	void (*SetPublicNetwork)(bool enable);
	uint32_t (*GetWakeupTime)(void);
	void (*IrqProcess)(void);
	void (*RxBoosted)(uint32_t timeout);
	void (*SetRxDutyCycle)(uint32_t rxTime, uint32_t sleepTime);
};
extern const struct LoRaMacRadio_s Radio __asm__("Radio");
}

/* IRQ status bits */
static const uint16_t kIrqHeaderValid = 0x0010;
static const uint16_t kIrqPreambleDetected = 0x0004;

/* RX gain register */
#define REG_RX_GAIN  0x08AC
#define RX_GAIN_POWER_SAVE  0x94
#define RX_GAIN_BOOSTED     0x96

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_lora, CONFIG_ZEPHCORE_LORA_LOG_LEVEL);

namespace mesh {

/**
 * SX126x-specific: Calibrate image rejection for a minimal 4 MHz range.
 */
static void calibrateImageMinimal(uint32_t freq_hz)
{
	uint16_t freq_mhz = freq_hz / 1000000;
	uint8_t cal_low = (uint8_t)(freq_mhz / 4);
	uint8_t cal_high = cal_low + 1;

	uint8_t calFreq[2] = { cal_low, cal_high };
	SX126xWriteCommand(RADIO_CALIBRATEIMAGE, calFreq, 2);

	LOG_DBG("SX126x image calibration: %u-%u MHz", cal_low * 4, cal_high * 4);
}

K_THREAD_STACK_DEFINE(sx126x_tx_wait_stack, TX_WAIT_THREAD_STACK_SIZE);

SX126xRadio::SX126xRadio(const struct device *lora_dev, MainBoard &board,
			  NodePrefs *prefs)
	: LoRaRadioBase(lora_dev, board, prefs)
{
}

void SX126xRadio::begin()
{
	startTxThread(sx126x_tx_wait_stack,
		      K_THREAD_STACK_SIZEOF(sx126x_tx_wait_stack));
	LoRaRadioBase::begin();
}

/* ── Hardware primitives ──────────────────────────────────────────────── */

void SX126xRadio::hwConfigure(const struct lora_modem_config &cfg)
{
	int ret = lora_config(_dev, const_cast<struct lora_modem_config *>(&cfg));
	if (ret < 0) {
		LOG_ERR("lora_config failed: %d", ret);
	} else {
		calibrateImageMinimal(cfg.frequency);
	}
}

void SX126xRadio::hwStartReceive()
{
	int ret = lora_recv_async(_dev, rxCallbackStatic, this);
	if (ret < 0) {
		LOG_ERR("lora_recv_async failed: %d", ret);
		_in_recv_mode = false;
		return;
	}
	_in_recv_mode = true;

	if (_rx_boost_enabled) {
		::Radio.Write(REG_RX_GAIN, RX_GAIN_BOOSTED);
	}
	applyRxDutyCycleIfEnabled();
}

void SX126xRadio::hwCancelReceive()
{
	lora_recv_async(_dev, NULL, NULL);
}

int SX126xRadio::hwSendAsync(uint8_t *buf, uint32_t len,
			     struct k_poll_signal *sig)
{
	return lora_send_async(_dev, buf, len, sig);
}

int16_t SX126xRadio::hwGetCurrentRSSI()
{
	return SX126xGetRssiInst();
}

bool SX126xRadio::hwIsPreambleDetected()
{
	uint16_t irq = SX126xGetIrqStatus();
	return (irq & kIrqHeaderValid) || (irq & kIrqPreambleDetected);
}

void SX126xRadio::hwSetRxBoost(bool enable)
{
	::Radio.Write(REG_RX_GAIN, enable ? RX_GAIN_BOOSTED : RX_GAIN_POWER_SAVE);
}

void SX126xRadio::hwSetRxDutyCycle(bool enable)
{
	if (enable) {
		applyRxDutyCycleIfEnabled();
	} else {
		::Radio.Rx(0);
		if (_rx_boost_enabled) {
			::Radio.Write(REG_RX_GAIN, RX_GAIN_BOOSTED);
		}
	}
}

void SX126xRadio::hwResetAGC()
{
	if (_in_recv_mode && !isReceiving()) {
		hwCancelReceive();
		startReceive();
	}
}

/* ── SX126x RX Duty Cycle — RadioLib Algorithm ───────────────────────── */

void SX126xRadio::applyRxDutyCycleIfEnabled()
{
	if (!_rx_duty_cycle_enabled || !_in_recv_mode) {
		return;
	}

	uint8_t sf = _prefs ? _prefs->sf : LoRaConfig::SPREADING_FACTOR;
	float bw_khz = _prefs ? _prefs->bw : (float)LoRaConfig::BANDWIDTH;
	uint16_t preamble_len = LoRaConfig::PREAMBLE_LEN;

	uint16_t min_symbols = (sf >= 7) ? RADIOLIB_MIN_SYMBOLS_SF7_PLUS
					 : RADIOLIB_MIN_SYMBOLS_SF6_LESS;

	int16_t sleep_symbols = (int16_t)preamble_len - (int16_t)min_symbols;
	if (sleep_symbols <= 0) {
		LOG_WRN("Preamble too short for duty cycle (need >%d, have %d)",
			min_symbols, preamble_len);
		_rx_duty_cycle_enabled = false;
		return;
	}

	uint32_t symbol_us = ((uint32_t)(1 << sf) * 1000) / (uint32_t)bw_khz;
	uint32_t sleep_period_us = sleep_symbols * symbol_us;

	uint32_t preamble_total_us = (preamble_len + 1) * symbol_us;
	int32_t wake_calc1 = ((int32_t)preamble_total_us -
			      ((int32_t)sleep_period_us - RADIOLIB_TCXO_DELAY_US)) / 2;
	uint32_t wake_calc2 = (min_symbols + 1) * symbol_us;

	uint32_t wake_period_us = (wake_calc1 > 0 && (uint32_t)wake_calc1 > wake_calc2)
				  ? (uint32_t)wake_calc1 : wake_calc2;

	/* SetRxDutyCycle takes times in 15.625us steps */
	uint32_t rx_time = (wake_period_us * 64) / 1000;
	uint32_t sleep_time = (sleep_period_us * 64) / 1000;

	if (rx_time < 64) rx_time = 64;
	if (sleep_time < 64) sleep_time = 64;

	::Radio.SetRxDutyCycle(rx_time, sleep_time);

	uint32_t rx_ms = (rx_time * 1000) / 64000;
	uint32_t sleep_ms = (sleep_time * 1000) / 64000;
	uint32_t total_ms = rx_ms + sleep_ms;
	uint32_t preamble_ms = (preamble_len * symbol_us) / 1000;

	uint32_t bw_int = (uint32_t)bw_khz;
	uint32_t bw_frac = (uint32_t)((bw_khz - bw_int) * 10);
	LOG_INF("RX duty cycle: SF%d BW%u.%u sym=%uus preamble=%ums",
		sf, bw_int, bw_frac, symbol_us, preamble_ms);
	LOG_INF("  -> rx=%ums sleep=%ums period=%ums (minSym=%d sleepSym=%d)",
		rx_ms, sleep_ms, total_ms, min_symbols, sleep_symbols);
}

} /* namespace mesh */

/*
 * SPDX-License-Identifier: Apache-2.0
 * Shared constants and utilities for all LoRa radio adapters.
 *
 * Anything duplicated between SX126xRadio and LR1110Radio belongs here.
 * Radio-specific constants (e.g. SX126x duty cycle math) stay in
 * their respective headers.
 */

#pragma once

#include <zephyr/drivers/lora.h>

/* --- Noise floor calibration --- */
#define NUM_NOISE_FLOOR_SAMPLES          64
#define NOISE_FLOOR_SAMPLING_THRESHOLD   14  /* only sample if rssi < floor + threshold */
#define DEFAULT_NOISE_FLOOR              0   /* first calibration accepts all samples */

/* --- RX ring buffer ---
 * Sized to be effectively drop-proof: even at the fastest LoRa settings
 * (SF7/BW500, ~5ms per packet), 32 slots buffer 160ms+ of back-to-back
 * arrivals.  The main loop drain takes microseconds per packet, so
 * overflow should never occur in practice.  Cost: 32 × 260 = ~8.3 KB. */
#define RX_RING_SIZE 32

/* --- TX wait thread --- */
#define TX_WAIT_THREAD_STACK_SIZE 1024
#define TX_WAIT_THREAD_PRIORITY   10  /* preemptible, lower than main */
#define TX_TIMEOUT_MS             5000   /* TX completion timeout */

/* --- SNR thresholds per spreading factor (SF7..SF12) --- */
inline constexpr float lora_snr_threshold[] = {
	-7.5f, -10.0f, -12.5f, -15.0f, -17.5f, -20.0f
};

/* --- Callback types (ISR-safe) --- */
typedef void (*RadioRxCallback)(void *user_data);
typedef void (*RadioTxDoneCallback)(void *user_data);

/* --- Zephyr enum mapping utilities --- */

/* Map kHz bandwidth value to Zephyr enum.
 * Input is (uint16_t)(float_bw) — truncated, e.g. 7.8→7, 10.4→10, 62.5→62.
 * Zephyr >=4.4 has narrow BWs (7-62 kHz); <=4.3 only has 125/250/500. */
static inline enum lora_signal_bandwidth bw_khz_to_enum(uint16_t bw_khz)
{
	switch (bw_khz) {
	case 7:   return BW_7_KHZ;
	case 10:  return BW_10_KHZ;
	case 15:  return BW_15_KHZ;
	case 20:  return BW_20_KHZ;
	case 31:  return BW_31_KHZ;
	case 41:  return BW_41_KHZ;
	case 62:  return BW_62_KHZ;
	case 125: return BW_125_KHZ;
	case 250: return BW_250_KHZ;
	case 500: return BW_500_KHZ;
	default:  return BW_125_KHZ;
	}
}

/* Map ZephCore CR (5-8) to Zephyr coding_rate enum (1-4) */
static inline enum lora_coding_rate cr_to_enum(uint8_t cr)
{
	switch (cr) {
	case 5: return CR_4_5;
	case 6: return CR_4_6;
	case 7: return CR_4_7;
	case 8: return CR_4_8;
	default: return CR_4_5;
	}
}

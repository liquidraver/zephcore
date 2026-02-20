/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore LoRa default parameters
 */

#pragma once

#include <stdint.h>

namespace mesh {

struct LoRaConfig {
	static constexpr uint32_t FREQ_HZ = 869618000;
	static constexpr uint16_t BANDWIDTH = 62;   /* BW_62_KHZ */
	static constexpr uint8_t SPREADING_FACTOR = 8;
	static constexpr uint8_t CODING_RATE = 8;   /* CR_4_8 */
	static constexpr uint16_t PREAMBLE_LEN = 16;
	static constexpr int8_t TX_POWER_DBM = 22;
};

} /* namespace mesh */

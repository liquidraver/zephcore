/*
 * SPDX-License-Identifier: Apache-2.0
 * Zephyr MainBoard base - nRF52840 DK and generic
 */

#pragma once

#include <mesh/Board.h>

namespace mesh {

class ZephyrBoard : public MainBoard {
public:
	uint16_t getBattMilliVolts() override;
	float getMCUTemperature() override;
	const char *getManufacturerName() const override;
	void reboot() override;
	void rebootToBootloader();        /* Reboot into UF2 mass storage bootloader */
	bool getBootloaderVersion(char *version, size_t max_len) override;
	bool startOTAUpdate(const char *id, char reply[]) override;  /* Reboot into BLE OTA DFU */
	void clearBootloaderMagic();      /* Clear stale GPREGRET values at startup */
	uint8_t getStartupReason() const override;
};

} /* namespace mesh */

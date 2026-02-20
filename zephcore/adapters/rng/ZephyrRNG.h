/*
 * SPDX-License-Identifier: Apache-2.0
 * Zephyr CSPRNG implementation
 */

#pragma once

#include <mesh/RNG.h>

namespace mesh {

class ZephyrRNG : public RNG {
public:
	void random(uint8_t *dest, size_t sz) override;
};

} /* namespace mesh */

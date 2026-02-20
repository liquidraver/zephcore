/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore RNG interface - matches Utils.h
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

namespace mesh {

class RNG {
public:
	virtual void random(uint8_t *dest, size_t sz) = 0;

	uint32_t nextInt(uint32_t _min, uint32_t _max);
};

} /* namespace mesh */

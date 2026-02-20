/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ZephyrRNG.h"
#include <zephyr/random/random.h>
#include <zephyr/sys/printk.h>

namespace mesh {

void ZephyrRNG::random(uint8_t *dest, size_t sz)
{
	int ret = sys_csrand_get(dest, sz);
	if (ret != 0) {
		printk("ZephyrRNG: CSPRNG failed (%d), using PRNG fallback\n", ret);
		sys_rand_get(dest, sz);
	}
}

} /* namespace mesh */

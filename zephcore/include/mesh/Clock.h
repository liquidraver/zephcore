/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore clock interfaces - matches Dispatcher.h / MeshCore.h
 */

#pragma once

#include <stdint.h>

namespace mesh {

class MillisecondClock {
public:
	virtual unsigned long getMillis() = 0;
};

} /* namespace mesh */

/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore constants and base types (Zephyr port)
 */

#pragma once

#include <stdint.h>
#include <math.h>

#define MAX_HASH_SIZE        8
#define PUB_KEY_SIZE        32
#define PRV_KEY_SIZE        64
#define SEED_SIZE           32
#define SIGNATURE_SIZE      64
#define MAX_ADVERT_DATA_SIZE  32
#define CIPHER_KEY_SIZE     16
#define CIPHER_BLOCK_SIZE   16
#define CIPHER_MAC_SIZE      2
#define PATH_HASH_SIZE       1

#define MAX_PACKET_PAYLOAD  184
#define MAX_PATH_SIZE        64
#define MAX_TRANS_UNIT      255

#define MESH_DEBUG_PRINT(...)
#define MESH_DEBUG_PRINTLN(...)

namespace mesh {

#define BD_STARTUP_NORMAL     0
#define BD_STARTUP_RX_PACKET  1

} /* namespace mesh */

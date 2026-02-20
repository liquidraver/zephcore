/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore Packet - fundamental transmission unit
 */

#pragma once

#include <mesh/MeshCore.h>
#include <stdint.h>
#include <string.h>

namespace mesh {

#define PH_ROUTE_MASK     0x03
#define PH_TYPE_SHIFT     2
#define PH_TYPE_MASK      0x0F
#define PH_VER_SHIFT      6
#define PH_VER_MASK       0x03

#define ROUTE_TYPE_TRANSPORT_FLOOD   0x00
#define ROUTE_TYPE_FLOOD             0x01
#define ROUTE_TYPE_DIRECT            0x02
#define ROUTE_TYPE_TRANSPORT_DIRECT  0x03

#define PAYLOAD_TYPE_REQ         0x00
#define PAYLOAD_TYPE_RESPONSE    0x01
#define PAYLOAD_TYPE_TXT_MSG     0x02
#define PAYLOAD_TYPE_ACK         0x03
#define PAYLOAD_TYPE_ADVERT      0x04
#define PAYLOAD_TYPE_GRP_TXT     0x05
#define PAYLOAD_TYPE_GRP_DATA    0x06
#define PAYLOAD_TYPE_ANON_REQ    0x07
#define PAYLOAD_TYPE_PATH        0x08
#define PAYLOAD_TYPE_TRACE       0x09
#define PAYLOAD_TYPE_MULTIPART   0x0A
#define PAYLOAD_TYPE_CONTROL     0x0B
#define PAYLOAD_TYPE_RAW_CUSTOM  0x0F

#define PAYLOAD_VER_1       0x00
#define PAYLOAD_VER_2       0x01
#define PAYLOAD_VER_3       0x02
#define PAYLOAD_VER_4       0x03

class Packet {
public:
	Packet();

	uint8_t header;
	uint16_t payload_len, path_len;
	uint16_t transport_codes[2];
	uint8_t path[MAX_PATH_SIZE];
	uint8_t payload[MAX_PACKET_PAYLOAD];
	int8_t _snr;

	void calculatePacketHash(uint8_t *dest_hash) const;
	uint8_t getRouteType() const { return header & PH_ROUTE_MASK; }
	bool isRouteFlood() const { return getRouteType() == ROUTE_TYPE_FLOOD || getRouteType() == ROUTE_TYPE_TRANSPORT_FLOOD; }
	bool isRouteDirect() const { return getRouteType() == ROUTE_TYPE_DIRECT || getRouteType() == ROUTE_TYPE_TRANSPORT_DIRECT; }
	bool hasTransportCodes() const { return getRouteType() == ROUTE_TYPE_TRANSPORT_FLOOD || getRouteType() == ROUTE_TYPE_TRANSPORT_DIRECT; }
	uint8_t getPayloadType() const { return (header >> PH_TYPE_SHIFT) & PH_TYPE_MASK; }
	uint8_t getPayloadVer() const { return (header >> PH_VER_SHIFT) & PH_VER_MASK; }
	void markDoNotRetransmit() { header = 0xFF; }
	bool isMarkedDoNotRetransmit() const { return header == 0xFF; }
	float getSNR() const { return ((float)_snr) / 4.0f; }
	int getRawLength() const;
	uint8_t writeTo(uint8_t dest[]) const;
	bool readFrom(const uint8_t src[], uint8_t len);
};

} /* namespace mesh */

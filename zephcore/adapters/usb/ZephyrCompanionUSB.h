/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore USB CDC Companion Transport
 *
 * V3-framed USB CDC for companion mode: ISR, ring buffer, frame parsing,
 * DTR monitoring, DFU trigger, and frame write.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize USB CDC companion transport.
 * Sets up ring buffer, UART ISR, and DTR polling.
 * Call after device_is_ready() for the CDC ACM device.
 *
 * @param mesh_events  Pointer to the k_event used by the mesh event loop
 * @param rx_work      Pointer to the rx_process work item (for BLE RX event)
 * @param mesh_event_ble_rx  Bitmask for MESH_EVENT_BLE_RX
 * @param board        Opaque pointer to ZephyrBoard (for DFU reboot)
 */
void zephcore_usb_companion_init(struct k_event *mesh_events,
				 struct k_work *rx_work,
				 uint32_t mesh_event_ble_rx,
				 void *board);

/**
 * Send a frame over USB with V3 length prefix.
 * @return number of payload bytes sent, or 0 on failure
 */
size_t zephcore_usb_companion_write_frame(const uint8_t *src, size_t len);

/**
 * Reset USB RX state (e.g., on BLE disconnect).
 */
void zephcore_usb_companion_reset_rx(void);

#ifdef __cplusplus
}
#endif

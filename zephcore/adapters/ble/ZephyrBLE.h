/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore BLE Adapter — NUS service, advertising, security, TX/RX
 */
#pragma once

#include <zephyr/kernel.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Callbacks from BLE adapter → main */
struct ble_callbacks {
	/* Called when a complete frame is received from the remote (RX).
	 * Called from system work queue context — must not block. */
	void (*on_rx_frame)(const uint8_t *data, uint16_t len);
	/* Called when TX queue is empty (can continue contact iteration) */
	void (*on_tx_idle)(void);
	/* Called on BLE connect (for UI notify + USB state clearing) */
	void (*on_connected)(void);
	/* Called on BLE disconnect */
	void (*on_disconnected)(void);
};

/* Interface type for BLE/USB coexistence */
enum zephcore_iface {
	ZEPHCORE_IFACE_NONE,
	ZEPHCORE_IFACE_BLE,
	ZEPHCORE_IFACE_USB,
};

/**
 * Register callbacks and auth handlers. Call before bt_enable().
 */
void zephcore_ble_init(const struct ble_callbacks *cbs);

/**
 * Called from bt_ready() after bt_enable succeeds.
 * Loads settings, builds advertising data, starts advertising.
 * @param device_name  Name to advertise (NULL = "MeshCore")
 */
void zephcore_ble_start(const char *device_name);

/**
 * Queue a frame for BLE TX.
 * @return number of bytes queued, or 0 on failure
 */
size_t zephcore_ble_send(const uint8_t *data, uint16_t len);

/**
 * Enable/disable BLE (advertising + connections).
 * When disabled, disconnects any active connection and stops advertising.
 */
void zephcore_ble_set_enabled(bool enable);

/**
 * Check if BLE is the active transport and ready to send.
 */
bool zephcore_ble_is_active(void);

/**
 * Check if BLE has an active connection (regardless of interface state).
 */
bool zephcore_ble_is_connected(void);

/**
 * Check if BLE TX is congested (queue full, overflow retrying).
 * Callers should stop sending until this clears.
 */
bool zephcore_ble_is_congested(void);

/**
 * Set the BLE passkey at runtime.
 */
void zephcore_ble_set_passkey(uint32_t passkey);

/**
 * Get the current BLE passkey.
 */
uint32_t zephcore_ble_get_passkey(void);

/**
 * Get/set active interface (for USB switching in main).
 */
enum zephcore_iface zephcore_ble_get_active_iface(void);
void zephcore_ble_set_active_iface(enum zephcore_iface iface);

/**
 * Get pointer to the recv_queue (k_msgq) for USB RX path sharing.
 * USB code in main needs to put frames directly into this queue.
 */
struct k_msgq *zephcore_ble_get_recv_queue(void);

/**
 * Get pointer to the send_queue for USB TX path.
 */
struct k_msgq *zephcore_ble_get_send_queue(void);

/**
 * Kick the TX drain work — call after putting frames in the send queue.
 */
void zephcore_ble_kick_tx(void);

/**
 * Disconnect the current BLE connection (if any).
 * Used by USB when it takes over as active interface.
 */
void zephcore_ble_disconnect(void);

/**
 * Apply deferred connection parameters.
 * Call after the initial app sync is complete (channels + contacts +
 * offline messages) so the param negotiation doesn't disrupt throughput
 * during the sync burst.
 */
void zephcore_ble_conn_params_ready(void);

#ifdef __cplusplus
}
#endif

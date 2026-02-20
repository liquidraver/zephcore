/*
 * ZephCore - UI Task
 * Copyright (c) 2025 ZephCore
 * SPDX-License-Identifier: Apache-2.0
 *
 * Wires together buttons, buzzer, and display.
 * Handles input events, page navigation, and notifications from mesh.
 *
 * All event-driven - no polling loops.
 */

#ifndef ZEPHCORE_UI_TASK_H
#define ZEPHCORE_UI_TASK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* UI notification events (from mesh/BLE layer) */
enum ui_event {
	UI_EVENT_NONE = 0,
	UI_EVENT_CONTACT_MSG,    /* New contact message received */
	UI_EVENT_CHANNEL_MSG,    /* New channel message received */
	UI_EVENT_ROOM_MSG,       /* New room message received */
	UI_EVENT_ACK,            /* ACK received */
	UI_EVENT_BLE_CONNECTED,  /* BLE peer connected */
	UI_EVENT_BLE_DISCONNECTED, /* BLE peer disconnected */
};

/**
 * Initialize the UI subsystem.
 * Sets up display, buzzer, and input event callbacks.
 * Safe to call even if no display/buzzer hardware is present.
 *
 * @return 0 on success
 */
int ui_init(void);

/**
 * Play the startup chime if buzzer is not muted.
 * Call from main() AFTER loadPrefs() so the persisted buzzer_quiet
 * setting is respected. No chime if user has muted the buzzer.
 */
void ui_play_startup_chime(void);

/**
 * Post a notification event to the UI.
 * Triggers buzzer melody and/or display wake depending on event type.
 * Safe to call from any thread context.
 *
 * @param event  The UI event type
 */
void ui_notify(enum ui_event event);

/**
 * Update the message count shown on the messages page.
 *
 * @param count  Current unread message count
 */
void ui_set_msg_count(uint16_t count);

/**
 * Update BLE connection status.
 *
 * @param connected  true if BLE peer is connected
 * @param name       BLE device name (can be NULL)
 */
void ui_set_ble_status(bool connected, const char *name);

/**
 * Update radio parameters for display.
 */
void ui_set_radio_params(uint32_t freq_hz, uint8_t sf, uint16_t bw_khz_x10,
			 uint8_t cr, int8_t tx_power, int16_t noise_floor);

/**
 * Update GPS data for display.
 */
void ui_set_gps_data(bool has_fix, uint8_t sats,
		     int32_t lat_mdeg, int32_t lon_mdeg, int32_t alt_mm);

/**
 * Update battery data for display.
 */
void ui_set_battery(uint16_t mv, uint8_t pct);

/**
 * Update RTC clock for top bar display.
 * @param epoch  Unix timestamp (0 = time not set, hides clock)
 */
void ui_set_clock(uint32_t epoch);

/**
 * Record a recently heard contact for the "recent" page.
 *
 * @param name   Contact name (truncated to 15 chars)
 * @param rssi   Signal strength in dBm
 * @param age_s  Seconds since contact was heard (computed from RTC)
 */
void ui_add_recent(const char *name, int16_t rssi, uint32_t age_s);

/**
 * Set the node name for the display top bar.
 */
void ui_set_node_name(const char *name);

/**
 * Clear and rebuild the recently heard contact list.
 * Call ui_clear_recent() then ui_add_recent() for each entry.
 */
void ui_clear_recent(void);

/**
 * Update sensor data for display.
 */
void ui_set_sensor_data(int16_t temp_c10, uint32_t pressure_pa,
			uint16_t humidity_rh10, uint16_t light_lux);

/**
 * Set whether GPS hardware was detected at boot.
 * If false, GPS page shows "GPS: not detected".
 */
void ui_set_gps_available(bool available);

/**
 * Set GPS enabled state (for display page).
 */
void ui_set_gps_enabled(bool enabled);

/**
 * Update GPS state machine info for display.
 * @param state        0=OFF, 1=STANDBY (sleeping), 2=ACQUIRING (searching)
 * @param last_fix_age_s  Seconds since last fix (UINT32_MAX = never)
 * @param next_search_s   Seconds until next search (0 = now or off)
 */
void ui_set_gps_state(uint8_t state, uint32_t last_fix_age_s, uint32_t next_search_s);

/**
 * Set BLE enabled state (for display page).
 */
void ui_set_ble_enabled(bool enabled);

/**
 * Set buzzer quiet state (for display page).
 */
void ui_set_buzzer_quiet(bool quiet);

/**
 * Set offgrid mode (client repeat) state for display page.
 */
void ui_set_offgrid_mode(bool enabled);

/**
 * Trigger a display refresh (for periodic updates from housekeeping).
 */
void ui_refresh_display(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHCORE_UI_TASK_H */

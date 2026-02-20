/*
 * SPDX-License-Identifier: Apache-2.0
 * Zephyr GPS Manager - GNSS power management, fix acquisition, constellation config
 *
 * Event-driven state machine with no polling loops:
 * - LoRa/BLE events trigger GPS enable/disable
 * - k_work_delayable handles standby/timeout timers
 * - GNSS callback fires on fix data from driver
 *
 * Power strategy:
 * - Direct GPIO toggle via gps-enable alias (all boards)
 * - T1000-E warm standby: VRTC stays powered during standby, preserving
 *   ephemeris/almanac/RTC in backup RAM for fast re-acquisition (3-8s vs 15-45s)
 * - Full power-off only on user-disable or System OFF
 */

#include "ZephyrGPSManager.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(zephcore_gps, CONFIG_ZEPHCORE_GPS_LOG_LEVEL);

/* ========== GNSS Support ========== */
#if DT_HAS_COMPAT_STATUS_OKAY(gnss_nmea_generic) || \
    DT_HAS_COMPAT_STATUS_OKAY(u_blox_m8) || \
    DT_HAS_COMPAT_STATUS_OKAY(u_blox_f9p) || \
    DT_HAS_COMPAT_STATUS_OKAY(quectel_lcx6g) || \
    DT_HAS_COMPAT_STATUS_OKAY(quectel_lc76g) || \
    DT_HAS_COMPAT_STATUS_OKAY(luatos_air530z)
#define HAS_GNSS 1
#include <zephyr/drivers/gnss.h>
#else
#define HAS_GNSS 0
#endif

/* ========== GPS Module Capability Flags ==========
 * GPS power management uses GPIO only (no PM_DEVICE):
 * - Wio Tracker L1 (L76K): FORCE_ON pin LOW = hardware standby (~360µA,
 *   Vcc stays on, ephemeris/almanac/RTC preserved, hot-start 1-2s)
 * - T1000-E (AG3335): GPS_EN LOW + VRTC HIGH = warm standby (ephemeris
 *   preserved via backup RAM, ~1-2µA VRTC current)
 * - All boards: gps-enable alias → GPIO power control
 *
 * PM_DEVICE is intentionally NOT used — the system-managed PM subsystem
 * auto-suspends devices during idle, calling modem_chat_run_script() from
 * an unexpected context which can deadlock the system. */

/* ========== GPS State - Power Management ========== */
#if HAS_GNSS
static struct gps_position current_pos;
static struct gnss_time current_utc;
static bool gps_enabled = false;
static bool gps_available = false;
static K_MUTEX_DEFINE(gps_mutex);
static gps_enable_callback_t gps_enable_cb = NULL;
static gps_fix_callback_t gps_fix_cb = NULL;
static gps_event_callback_t gps_event_cb = NULL;

/* Pending GPS actions — set by work handlers (system work queue),
 * consumed by gps_process_event() (main thread).
 * This avoids calling blocking GNSS APIs from the system work queue,
 * which deadlocks because modem_chat_run_script() blocks on a semaphore
 * that's signaled from the same work queue. */
#define GPS_ACTION_WAKE     BIT(0)  /* Wake from standby → start acquiring */
#define GPS_ACTION_TIMEOUT  BIT(1)  /* Acquisition timeout → go to standby */
#define GPS_ACTION_FIX_DONE BIT(2)  /* Got enough good fixes → go to standby */
static atomic_t pending_gps_actions;

/* GPS Power Management State Machine */
enum gps_state {
	GPS_STATE_OFF,          /* GPS disabled by user */
	GPS_STATE_STANDBY,      /* GPS enabled but sleeping (5 min cycle) */
	GPS_STATE_ACQUIRING,    /* GPS awake, waiting for fixes */
};

static enum gps_state gps_current_state = GPS_STATE_OFF;
static uint8_t consecutive_good_fixes = 0;
static bool first_fix_acquired = false;  /* True after first successful fix since enable */
static bool gps_time_synced = false;     /* True after GPS syncs RTC. Starts false at boot (RTC reset),
                                          * set true after 3 good fixes, cleared when GPS disabled. */
static int64_t last_fix_uptime_ms = 0;  /* k_uptime when last validated fix was acquired */
static int64_t standby_start_ms = 0;    /* k_uptime when standby started (for next-wake calc) */
static uint64_t standby_interval_ms = 0; /* How long standby lasts (for next-wake calc) */

#define GPS_GOOD_FIX_COUNT       3       /* Need 3 consecutive good fixes */
#define GPS_MIN_SATELLITES       4       /* Minimum satellites for valid fix */

/* Runtime-configurable intervals, initialized from Kconfig defaults */
static uint32_t gps_acquire_timeout_ms = CONFIG_ZEPHCORE_GPS_FIX_TIMEOUT_SEC * 1000U;
static uint32_t gps_wake_interval_ms   = CONFIG_ZEPHCORE_GPS_POLL_INTERVAL_SEC * 1000U;

/* Repeater mode intervals - GPS only for time sync */
#define GPS_REPEATER_SYNC_INTERVAL_MS  (48ULL * 60 * 60 * 1000)  /* 48 hours */
#define GPS_REPEATER_SYNC_TIMEOUT_MS   (5 * 60 * 1000)           /* 5 minutes */

static bool gps_repeater_mode = false;  /* True = repeater (time sync only), False = companion */

/* Forward declarations for work handlers and state functions */
static void gps_wake_work_fn(struct k_work *work);
static void gps_timeout_work_fn(struct k_work *work);
static void gps_go_to_standby(void);
static void gps_start_acquiring(void);

/* Delayable work for event-driven timers (no polling!) */
static K_WORK_DELAYABLE_DEFINE(gps_wake_work, gps_wake_work_fn);
static K_WORK_DELAYABLE_DEFINE(gps_timeout_work, gps_timeout_work_fn);

#else
static gps_enable_callback_t gps_enable_cb = NULL;
#endif

void gps_set_enable_callback(gps_enable_callback_t cb)
{
	gps_enable_cb = cb;
}

void gps_set_fix_callback(gps_fix_callback_t cb)
{
#if HAS_GNSS
	gps_fix_cb = cb;
#else
	ARG_UNUSED(cb);
#endif
}

void gps_set_event_callback(gps_event_callback_t cb)
{
#if HAS_GNSS
	gps_event_cb = cb;
#else
	ARG_UNUSED(cb);
#endif
}

#if HAS_GNSS

/* GNSS callback - called when new fix data is available */
static void gnss_data_cb(const struct device *dev, const struct gnss_data *data)
{
	ARG_UNUSED(dev);

	if (!gps_enabled) {
		/* GPS disabled — ignore stale NMEA from hardware power-down.
		 * The GNSS driver fires callbacks as long as the UART has data,
		 * even after we drive GPS_EN LOW (module drains its buffer). */
		return;
	}

	LOG_DBG("GNSS callback: fix=%d sats=%d state=%d",
		data->info.fix_status, data->info.satellites_cnt, gps_current_state);

	k_mutex_lock(&gps_mutex, K_FOREVER);

	if (data->info.fix_status >= GNSS_FIX_STATUS_GNSS_FIX) {
		current_pos.latitude_ndeg = data->nav_data.latitude;
		current_pos.longitude_ndeg = data->nav_data.longitude;
		current_pos.altitude_mm = data->nav_data.altitude;
		current_pos.satellites = data->info.satellites_cnt;
		current_pos.valid = true;
		current_pos.timestamp_ms = k_uptime_get();
		current_utc = data->utc;

		/* Fix validation during acquisition */
		if (gps_current_state == GPS_STATE_ACQUIRING) {
			if (data->info.satellites_cnt >= GPS_MIN_SATELLITES) {
				consecutive_good_fixes++;
				LOG_INF("GPS: Good fix %d/%d (sats=%d) lat=%lld lon=%lld",
					consecutive_good_fixes, GPS_GOOD_FIX_COUNT,
					data->info.satellites_cnt,
					current_pos.latitude_ndeg / 1000000,
					current_pos.longitude_ndeg / 1000000);

				if (consecutive_good_fixes >= GPS_GOOD_FIX_COUNT) {
					LOG_INF("GPS: Got %d good fixes, updating location/time",
						GPS_GOOD_FIX_COUNT);

					/* Mark first fix acquired (enables timeout for future cycles) */
					first_fix_acquired = true;

					/* Mark time as synced from GPS - blocks phone time sync */
					gps_time_synced = true;
					last_fix_uptime_ms = k_uptime_get();

					/* Cancel timeout */
					k_work_cancel_delayable(&gps_timeout_work);

					/* Notify fix callback with validated position */
					if (gps_fix_cb) {
						double lat = (double)data->nav_data.latitude / 1000000000.0;
						double lon = (double)data->nav_data.longitude / 1000000000.0;
						k_mutex_unlock(&gps_mutex);
						gps_fix_cb(lat, lon, gps_get_utc_time());
					} else {
						k_mutex_unlock(&gps_mutex);
					}

					/* Defer standby to main thread — we're on the system
					 * workqueue here (GNSS callback), can't call PM suspend
					 * (modem_chat_run_script deadlocks on same workqueue). */
					atomic_or(&pending_gps_actions, GPS_ACTION_FIX_DONE);
					if (gps_event_cb) {
						gps_event_cb();
					}
					return;
				}
			} else {
				/* Reset counter on bad fix (< 4 satellites) */
				if (consecutive_good_fixes > 0) {
					LOG_DBG("GPS: Poor fix (sats=%d), resetting counter",
						data->info.satellites_cnt);
				}
				consecutive_good_fixes = 0;
			}
		}
	} else {
		current_pos.valid = false;
		/* Reset counter on no fix */
		if (gps_current_state == GPS_STATE_ACQUIRING && consecutive_good_fixes > 0) {
			LOG_DBG("GPS: No fix, resetting counter");
			consecutive_good_fixes = 0;
		}
	}

	k_mutex_unlock(&gps_mutex);
}

/* Register GNSS callback for all GNSS devices */
GNSS_DATA_CALLBACK_DEFINE(NULL, gnss_data_cb);

/* Find and initialize GNSS device */
static const struct device *gnss_dev = NULL;

/* Multi-constellation configuration — runs ONCE at boot.
 * modem_chat_run_script() blocks on a semaphore signaled from the system
 * work queue. Calling it after a GPIO power cycle can deadlock because:
 * 1. The L76K needs ~300ms to boot after power restore
 * 2. Meanwhile the modem_chat may be processing stale UART data
 * 3. The script completion callback competes with NMEA processing
 *
 * Safe to call at boot because the driver init already ran and the chip
 * is powered and outputting NMEA. After power cycles, the L76K retains
 * constellation + fix rate settings in internal flash (PCAS commands
 * persist). So we only need to configure once. */
static bool gnss_configured = false;

static void gnss_configure(void)
{
	if (gnss_configured || gnss_dev == NULL) {
		return;
	}

	/* Enable all available constellation systems for faster TTFF.
	 * Try GPS+GLONASS+Galileo+BeiDou first (AG3335 supports all).
	 * Fall back to GPS+GLONASS+BeiDou if Galileo not supported (L76KB). */
	gnss_systems_t systems = GNSS_SYSTEM_GPS | GNSS_SYSTEM_GLONASS |
				 GNSS_SYSTEM_GALILEO | GNSS_SYSTEM_BEIDOU;
	int ret = gnss_set_enabled_systems(gnss_dev, systems);
	if (ret == -EINVAL) {
		/* Some systems not supported — try without Galileo */
		systems = GNSS_SYSTEM_GPS | GNSS_SYSTEM_GLONASS | GNSS_SYSTEM_BEIDOU;
		ret = gnss_set_enabled_systems(gnss_dev, systems);
	}
	if (ret == 0) {
		LOG_INF("GPS: Multi-constellation enabled");
	} else if (ret == -ENOSYS || ret == -ENOTSUP) {
		LOG_INF("GPS: Constellation config not supported by driver");
	} else {
		LOG_WRN("GPS: Failed to set constellations: %d", ret);
		/* Will retry on next power-on cycle */
		return;
	}

	/* Set 1Hz fix rate (explicit, don't rely on chip defaults) */
	ret = gnss_set_fix_rate(gnss_dev, 1000);
	if (ret == 0) {
		LOG_INF("GPS: Fix rate set to 1Hz");
	} else if (ret != -ENOSYS && ret != -ENOTSUP) {
		LOG_WRN("GPS: Failed to set fix rate: %d", ret);
	}

	gnss_configured = true;
}

#endif /* HAS_GNSS - GPS power GPIO section is unconditional (needed for shutdown) */

/* ========== GPS Power GPIO Control ==========
 * These are unconditional (not gated by HAS_GNSS) because
 * gps_power_off_for_shutdown() must be available for System OFF
 * even on boards without a GNSS driver.
 *
 * IMPORTANT: Do NOT touch GPIO during init! The GNSS driver needs the GPS
 * to be powered and outputting NMEA for the modem pipe to work.
 * We only configure GPIO lazily on first power-off request.
 *
 * Board-specific pins (defined in board overlays as gps-enable alias):
 * - T1000-E: P1.11 (GPS_EN), P0.8 (GPS_VRTC_EN), P1.15 (GPS_RESET), P1.12 (GPS_SLEEP_INT)
 * - Wio Tracker L1: P1.09 (GPS power, shared with luatos,air530z on-off-gpios)
 */
#if DT_NODE_EXISTS(DT_ALIAS(gps_enable))
static const struct gpio_dt_spec gps_enable_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(gps_enable), gpios);
#define HAS_GPS_POWER_CONTROL 1
#else
#define HAS_GPS_POWER_CONTROL 0
#endif

/* T1000-E specific GPS control pins */
#if DT_NODE_EXISTS(DT_ALIAS(gps_vrtc_enable))
static const struct gpio_dt_spec gps_vrtc_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(gps_vrtc_enable), gpios);
#define HAS_GPS_VRTC 1
#else
#define HAS_GPS_VRTC 0
#endif

#if DT_NODE_EXISTS(DT_ALIAS(gps_reset))
static const struct gpio_dt_spec gps_reset_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(gps_reset), gpios);
#define HAS_GPS_RESET 1
#else
#define HAS_GPS_RESET 0
#endif

#if DT_NODE_EXISTS(DT_ALIAS(gps_sleep_int))
static const struct gpio_dt_spec gps_sleep_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(gps_sleep_int), gpios);
#define HAS_GPS_SLEEP 1
#else
#define HAS_GPS_SLEEP 0
#endif

/* T1000-E has extra GPS control pins that require a specific init sequence */
#define HAS_T1000_GPS_CONTROL (HAS_GPS_VRTC || HAS_GPS_RESET || HAS_GPS_SLEEP)

#if HAS_GPS_POWER_CONTROL
static bool gps_gpio_configured = false;
#endif

/* GPS power control with warm standby support.
 * @param on        true = power on, false = power off
 * @param keep_vrtc When powering off: true = keep VRTC alive (warm standby,
 *                  preserves ephemeris/almanac/RTC for fast re-acquisition),
 *                  false = full power-off (cold start on next wake).
 *                  Only relevant on T1000-E (HAS_GPS_VRTC); ignored on other boards. */
static void gps_power_control(bool on, bool keep_vrtc = false)
{
#if HAS_GPS_POWER_CONTROL
	/* Direct GPIO power control — works on all boards.
	 * We toggle the GPS power pin ourselves rather than using driver PM
	 * (driver PM can hang on modem_pipe_close / modem_chat_run_script).
	 * The GNSS driver's modem pipe stays open; NMEA data simply stops
	 * when power is cut and resumes when power is restored. */
	if (on) {
#if HAS_T1000_GPS_CONTROL
		/* T1000-E power-on sequence (from Arduino target.cpp start_gps())
		 * Must follow this exact order with delays:
		 * 1. GPS_EN HIGH, delay 10ms
		 * 2. GPS_VRTC_EN HIGH, delay 10ms (critical - RTC power)
		 * 3. GPS_RESET HIGH, delay 10ms, then LOW
		 * 4. GPS_SLEEP_INT HIGH
		 */
		if (gpio_is_ready_dt(&gps_enable_gpio)) {
			gpio_pin_configure_dt(&gps_enable_gpio, GPIO_OUTPUT_HIGH);
		}
		k_msleep(10);

#if HAS_GPS_VRTC
		if (gpio_is_ready_dt(&gps_vrtc_gpio)) {
			gpio_pin_configure_dt(&gps_vrtc_gpio, GPIO_OUTPUT_HIGH);
		}
		k_msleep(10);
#endif

#if HAS_GPS_RESET
		if (gpio_is_ready_dt(&gps_reset_gpio)) {
			gpio_pin_configure_dt(&gps_reset_gpio, GPIO_OUTPUT_HIGH);
			k_msleep(10);
			gpio_pin_set_dt(&gps_reset_gpio, 0);  /* Release reset */
		}
#endif

#if HAS_GPS_SLEEP
		if (gpio_is_ready_dt(&gps_sleep_gpio)) {
			gpio_pin_configure_dt(&gps_sleep_gpio, GPIO_OUTPUT_HIGH);
		}
#endif
		gps_gpio_configured = true;
		LOG_INF("GPS power ON (T1000-E sequence)");
#else
		/* Simple boards - just GPS_EN */
		if (!gps_gpio_configured) {
			if (gpio_is_ready_dt(&gps_enable_gpio)) {
				gpio_pin_configure_dt(&gps_enable_gpio, GPIO_OUTPUT_HIGH);
				gps_gpio_configured = true;
				LOG_INF("GPS power GPIO configured, set HIGH");
			} else {
				LOG_WRN("GPS power GPIO not ready");
				return;
			}
		} else {
			gpio_pin_set_dt(&gps_enable_gpio, 1);
			LOG_INF("GPS power ON");
		}
#endif
	} else {
		/* Power off sequence */
#if HAS_GPS_VRTC
		if (!keep_vrtc) {
			/* Full power-off: VRTC off too (cold start on next wake) */
			if (gpio_is_ready_dt(&gps_vrtc_gpio)) {
				if (!gps_gpio_configured) {
					gpio_pin_configure_dt(&gps_vrtc_gpio, GPIO_OUTPUT_LOW);
				} else {
					gpio_pin_set_dt(&gps_vrtc_gpio, 0);
				}
			}
		}
		/* else: warm standby — VRTC stays HIGH, preserving
		 * ephemeris/almanac/RTC for fast re-acquisition (~1-2 µA) */
#endif
		if (gpio_is_ready_dt(&gps_enable_gpio)) {
			if (!gps_gpio_configured) {
				gpio_pin_configure_dt(&gps_enable_gpio, GPIO_OUTPUT_LOW);
				gps_gpio_configured = true;
			} else {
				gpio_pin_set_dt(&gps_enable_gpio, 0);
			}
		}
#if HAS_GPS_VRTC
		LOG_INF("GPS power OFF (%s)", keep_vrtc ?
			"standby — VRTC retained" : "full");
#else
		LOG_INF("GPS power OFF");
#endif
	}
#else
	ARG_UNUSED(keep_vrtc);
#endif
}

/* Drive all GPS power-enable GPIOs LOW for System OFF.
 * Uses gpio_pin_configure_dt() so pins are properly set even if
 * gps_power_control() was never called (GPIO not yet configured). */
void gps_power_off_for_shutdown(void)
{
#if HAS_GPS_POWER_CONTROL
	if (gpio_is_ready_dt(&gps_enable_gpio)) {
		gpio_pin_configure_dt(&gps_enable_gpio, GPIO_OUTPUT_LOW);
	}
#endif
#if HAS_GPS_VRTC
	if (gpio_is_ready_dt(&gps_vrtc_gpio)) {
		gpio_pin_configure_dt(&gps_vrtc_gpio, GPIO_OUTPUT_LOW);
	}
#endif
#if HAS_GPS_RESET
	if (gpio_is_ready_dt(&gps_reset_gpio)) {
		gpio_pin_configure_dt(&gps_reset_gpio, GPIO_OUTPUT_LOW);
	}
#endif
#if HAS_GPS_SLEEP
	if (gpio_is_ready_dt(&gps_sleep_gpio)) {
		gpio_pin_configure_dt(&gps_sleep_gpio, GPIO_OUTPUT_LOW);
	}
#endif
}

#if HAS_GNSS  /* Resume GNSS-specific code */

/* Go to standby and schedule next wake.
 * GPIO power control only — keep VRTC for warm start on T1000-E,
 * FORCE_ON pin LOW for L76K hardware standby. */
static void gps_go_to_standby(void)
{
	uint64_t wake_interval = gps_repeater_mode ?
		GPS_REPEATER_SYNC_INTERVAL_MS : gps_wake_interval_ms;

	if (gps_repeater_mode) {
		LOG_INF("GPS: Going to standby for 48 hours (repeater mode)");
	} else {
		LOG_INF("GPS: Going to standby for %d minutes", (int)(wake_interval / 60000));
	}
	gps_current_state = GPS_STATE_STANDBY;
	consecutive_good_fixes = 0;

	/* Record standby timing for UI (next-wake calculation) */
	standby_start_ms = k_uptime_get();
	standby_interval_ms = wake_interval;

	/* GPIO power-off — keep VRTC for warm start on T1000-E */
	gps_power_control(false, true);

	/* NOTE: gnss_configured stays true — L76K retains PCAS settings in
	 * flash across power cycles. Re-running gnss_configure() after GPIO
	 * wake would call modem_chat_run_script() before the chip has booted,
	 * risking a deadlock (modem_chat blocks on system work queue). */

	/* Schedule next wake (event-driven, no polling!) */
	k_work_schedule(&gps_wake_work, K_MSEC(wake_interval));
}

/* Wake GPS and start acquiring — GPIO power-on.
 * Does NOT call gnss_configure() — constellation/fix-rate settings persist
 * in L76K flash across power cycles. Calling modem_chat_run_script() here
 * would deadlock: the chip needs ~300ms to boot after GPIO power restore,
 * but modem_chat blocks the calling thread waiting for the system work
 * queue which may be processing stale UART data. */
static void gps_start_acquiring(void)
{
	LOG_INF("GPS: Waking for %s", gps_repeater_mode ? "time sync" : "position fix");
	gps_current_state = GPS_STATE_ACQUIRING;
	consecutive_good_fixes = 0;

	gps_power_control(true);

	/* Start timeout timer */
	if (gps_repeater_mode) {
		/* Repeater mode: always use 5 min timeout for time sync */
		k_work_schedule(&gps_timeout_work, K_MSEC(GPS_REPEATER_SYNC_TIMEOUT_MS));
	} else if (first_fix_acquired) {
		/* Companion mode: use short timeout after first fix acquired */
		k_work_schedule(&gps_timeout_work, K_MSEC(gps_acquire_timeout_ms));
	} else {
		LOG_INF("GPS: First fix mode - no timeout");
	}
}

/* Work handler: wake GPS for fix.
 * Runs on system work queue — MUST NOT call blocking GNSS APIs directly!
 * modem_chat_run_script() blocks on a semaphore signaled from this same
 * work queue, causing a deadlock. Instead, set a flag and signal the
 * main thread to do the actual wake via gps_process_event(). */
static void gps_wake_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!gps_enabled || gps_current_state != GPS_STATE_STANDBY) {
		return;
	}

	atomic_or(&pending_gps_actions, GPS_ACTION_WAKE);
	if (gps_event_cb) {
		gps_event_cb();
	}
}

/* Work handler: timeout waiting for fix.
 * Runs on system work queue — MUST NOT call blocking GNSS APIs directly!
 * Same deadlock risk as gps_wake_work_fn. Defer to main thread. */
static void gps_timeout_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (gps_current_state != GPS_STATE_ACQUIRING) {
		return;
	}

	LOG_WRN("GPS: Timeout after %d/%d fixes, deferring standby to main thread",
		consecutive_good_fixes, GPS_GOOD_FIX_COUNT);

	atomic_or(&pending_gps_actions, GPS_ACTION_TIMEOUT);
	if (gps_event_cb) {
		gps_event_cb();
	}
}

static int gnss_init(void)
{
	/* Try to find a GNSS device - prefer chip-specific drivers
	 * (they support constellation config, fix rate, etc.) over
	 * the generic NMEA parser which is passive only.
	 * Power control is done lazily in gps_enable(). */
#if DT_HAS_COMPAT_STATUS_OKAY(quectel_lc76g)
	gnss_dev = DEVICE_DT_GET_ANY(quectel_lc76g);
#elif DT_HAS_COMPAT_STATUS_OKAY(luatos_air530z)
	gnss_dev = DEVICE_DT_GET_ANY(luatos_air530z);
#elif DT_HAS_COMPAT_STATUS_OKAY(quectel_lcx6g)
	gnss_dev = DEVICE_DT_GET_ANY(quectel_lcx6g);
#elif DT_HAS_COMPAT_STATUS_OKAY(u_blox_m8)
	gnss_dev = DEVICE_DT_GET_ANY(u_blox_m8);
#elif DT_HAS_COMPAT_STATUS_OKAY(u_blox_f9p)
	gnss_dev = DEVICE_DT_GET_ANY(u_blox_f9p);
#elif DT_HAS_COMPAT_STATUS_OKAY(gnss_nmea_generic)
	gnss_dev = DEVICE_DT_GET_ANY(gnss_nmea_generic);
#endif

	if (gnss_dev == NULL) {
		LOG_WRN("No GNSS device found in device tree");
		return -ENODEV;
	}

	if (!device_is_ready(gnss_dev)) {
		LOG_ERR("GNSS device %s not ready", gnss_dev->name);
		return -ENODEV;
	}

	LOG_INF("GNSS device %s initialized", gnss_dev->name);
	gps_available = true;
	return 0;
}
#endif /* HAS_GNSS */

/* ========== Public API ========== */

int gps_manager_init(void)
{
#if HAS_GNSS
	gnss_init();

	/* Configure constellations + fix rate NOW while chip is powered
	 * and the modem pipe is open (driver init already ran).
	 * This is the ONLY safe place to call modem_chat_run_script() —
	 * after power cycles the chip needs ~300ms boot time and calling
	 * modem_chat from the main thread risks deadlock. L76K retains
	 * PCAS settings in flash, so one-time config at boot is enough. */
	gnss_configure();
#endif
	return 0;
}

bool gps_is_available(void)
{
#if HAS_GNSS
	return gps_available;
#else
	return false;
#endif
}

bool gps_is_enabled(void)
{
#if HAS_GNSS
	return gps_enabled;
#else
	return false;
#endif
}

void gps_ensure_power_state(bool should_be_enabled)
{
#if HAS_GNSS
	if (!gps_available) {
		return;
	}

	/* At boot, GPS hardware is powered (bootloader/pull-up).
	 * If it should be disabled, explicitly power it off now. */
	if (!should_be_enabled) {
		LOG_INF("GPS: Powering off at boot (disabled in prefs)");
		gps_power_control(false);
		gps_current_state = GPS_STATE_OFF;
	}
#else
	ARG_UNUSED(should_be_enabled);
#endif
}

void gps_set_repeater_mode(bool repeater)
{
#if HAS_GNSS
	if (!gps_available) {
		return;
	}

	gps_repeater_mode = repeater;

	if (repeater) {
		LOG_INF("GPS: Repeater mode - starting initial time sync, then every 48h");

		gps_enabled = true;  /* Logically enabled */

		/* Start acquiring immediately for initial time sync at boot.
		 * GPS hardware is already powered from bootloader, so we just
		 * start the acquisition state machine. */
		gps_start_acquiring();
	} else {
		LOG_INF("GPS: Companion mode");
	}
#else
	ARG_UNUSED(repeater);
#endif
}

void gps_enable(bool enable)
{
#if HAS_GNSS
	if (!gps_available) {
		LOG_WRN("GPS not available");
		return;
	}

	if (enable == gps_enabled) {
		return;
	}

	gps_enabled = enable;

	if (enable) {
		LOG_INF("GPS enabled - starting acquisition");

		/* Start acquiring immediately (no delay for first wake) */
		gps_current_state = GPS_STATE_ACQUIRING;
		consecutive_good_fixes = 0;

		/* Power on GPS - uses lazy GPIO init */
		gps_power_control(true);

		/* gnss_configure() runs once at boot (see gps_manager_init path).
		 * L76K retains PCAS settings in flash across power cycles.
		 * Do NOT call modem_chat_run_script() here — the chip needs
		 * ~300ms to boot after GPIO power restore and calling it
		 * immediately deadlocks the main thread. */

		/* No timeout for first fix after enable - wait as long as needed */
		if (first_fix_acquired) {
			k_work_schedule(&gps_timeout_work, K_MSEC(gps_acquire_timeout_ms));
		} else {
			LOG_INF("GPS: First fix mode - no timeout until first successful fix");
		}
	} else {
		LOG_INF("GPS disabled - canceling timers and powering off");

		/* Cancel any pending work */
		k_work_cancel_delayable(&gps_wake_work);
		k_work_cancel_delayable(&gps_timeout_work);

		/* Power off GPS */
		gps_power_control(false);
		gps_current_state = GPS_STATE_OFF;
		consecutive_good_fixes = 0;

		/* Clear time sync flag - time will drift, allow phone sync again */
		gps_time_synced = false;
	}

	/* Notify callback (for persistence in main.cpp) */
	if (gps_enable_cb) {
		gps_enable_cb(enable);
	}
#else
	ARG_UNUSED(enable);
#endif
}

void gps_get_position(struct gps_position *pos)
{
#if HAS_GNSS
	k_mutex_lock(&gps_mutex, K_FOREVER);
	*pos = current_pos;
	k_mutex_unlock(&gps_mutex);
#else
	memset(pos, 0, sizeof(*pos));
#endif
}

uint32_t gps_get_poll_interval_sec(void)
{
#if HAS_GNSS
	return gps_wake_interval_ms / 1000U;
#else
	return CONFIG_ZEPHCORE_GPS_POLL_INTERVAL_SEC;
#endif
}

void gps_set_poll_interval_sec(uint32_t interval)
{
#if HAS_GNSS
	if (interval < 10) interval = 10;
	if (interval > 86400) interval = 86400;
	gps_wake_interval_ms = interval * 1000U;
	LOG_INF("GPS poll interval set to %u seconds", interval);
#else
	ARG_UNUSED(interval);
#endif
}

int64_t gps_get_utc_time(void)
{
#if HAS_GNSS
	k_mutex_lock(&gps_mutex, K_FOREVER);
	if (!current_pos.valid) {
		k_mutex_unlock(&gps_mutex);
		return 0;
	}

	struct gnss_time t = current_utc;
	k_mutex_unlock(&gps_mutex);

	int year = 2000 + t.century_year;
	int days = 0;

	for (int y = 1970; y < year; y++) {
		days += (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)) ? 366 : 365;
	}

	static const int month_days[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	for (int m = 1; m < t.month; m++) {
		days += month_days[m];
		if (m == 2 && (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0))) {
			days++;
		}
	}

	days += t.month_day - 1;

	int64_t timestamp = (int64_t)days * 86400;
	timestamp += t.hour * 3600;
	timestamp += t.minute * 60;
	timestamp += t.millisecond / 1000;

	return timestamp;
#else
	return 0;
#endif
}

bool gps_has_time_sync(void)
{
#if HAS_GNSS
	/* Returns true if GPS has recently synced the RTC.
	 * Expires after 2 hours without a fix so the phone can re-sync
	 * (e.g. node moved indoors, GPS lost sky, RTC drifting). */
	if (!gps_time_synced) {
		return false;
	}
	int64_t age_ms = k_uptime_get() - last_fix_uptime_ms;

	if (age_ms > (2 * 60 * 60 * 1000LL)) {
		gps_time_synced = false;
		return false;
	}
	return true;
#else
	return false;
#endif
}

bool gps_get_last_known_position(struct gps_position *pos)
{
#if HAS_GNSS
	k_mutex_lock(&gps_mutex, K_FOREVER);
	if (current_pos.valid) {
		*pos = current_pos;
		k_mutex_unlock(&gps_mutex);
		return true;
	}
	k_mutex_unlock(&gps_mutex);
#endif
	memset(pos, 0, sizeof(*pos));
	return false;
}

void gps_request_fresh_fix(void)
{
#if HAS_GNSS
	if (!gps_available || !gps_enabled) {
		return;
	}

	if (gps_current_state == GPS_STATE_STANDBY) {
		LOG_INF("GPS: Fresh fix requested, waking early");
		/* Cancel scheduled wake and wake immediately */
		k_work_cancel_delayable(&gps_wake_work);
		gps_start_acquiring();
	} else if (gps_current_state == GPS_STATE_ACQUIRING) {
		LOG_DBG("GPS: Fresh fix requested but already acquiring");
	}
#endif
}

void gps_get_state_info(struct gps_state_info *info)
{
	memset(info, 0, sizeof(*info));
#if HAS_GNSS
	info->state = (uint8_t)gps_current_state;
	info->satellites = current_pos.satellites;

	if (last_fix_uptime_ms > 0) {
		/* Seconds since last validated fix */
		info->last_fix_age_s = (uint32_t)((k_uptime_get() - last_fix_uptime_ms) / 1000);
	} else {
		info->last_fix_age_s = UINT32_MAX;  /* No fix yet */
	}

	if (gps_current_state == GPS_STATE_STANDBY && standby_interval_ms > 0) {
		int64_t wake_at = standby_start_ms + (int64_t)standby_interval_ms;
		int64_t remaining = wake_at - k_uptime_get();

		info->next_search_s = (remaining > 0) ? (uint32_t)(remaining / 1000) : 0;
	} else if (gps_current_state == GPS_STATE_ACQUIRING) {
		info->next_search_s = 0;  /* Searching right now */
	}
#endif
}

/* Process pending GPS state transitions — called from main thread.
 * Work handlers on the system work queue set flags + signal the main
 * thread via gps_event_cb(). The main thread then calls this function,
 * which safely executes blocking GNSS configuration (modem_chat_run_script
 * blocks on a semaphore signaled from the system work queue — calling it
 * FROM the work queue deadlocks). */
void gps_process_event(void)
{
#if HAS_GNSS
	uint32_t actions = (uint32_t)atomic_clear(&pending_gps_actions);

	if (actions == 0) {
		return;
	}

	/* Wake takes priority — if both wake and timeout/fix-done are pending
	 * (shouldn't happen, but be safe), wake wins. */
	if (actions & GPS_ACTION_WAKE) {
		if (gps_enabled && gps_current_state == GPS_STATE_STANDBY) {
			gps_start_acquiring();
		}
	} else if (actions & (GPS_ACTION_TIMEOUT | GPS_ACTION_FIX_DONE)) {
		if (gps_current_state == GPS_STATE_ACQUIRING) {
			gps_go_to_standby();
		}
	}
#endif
}

/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore BLE Adapter — NUS service, advertising, security, TX/RX
 *
 * Extracted from main_companion.cpp for independent BLE debug logging.
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_ble, CONFIG_ZEPHCORE_BLE_LOG_LEVEL);

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/bluetooth/services/nus/inst.h>
#include <zephyr/settings/settings.h>

#include "ZephyrBLE.h"

/* MAX_FRAME_SIZE from CompanionMesh.h — keep in sync */
#ifndef MAX_FRAME_SIZE
#define MAX_FRAME_SIZE 172
#endif

/* ========== Constants ========== */

#define DEVICE_NAME_MAX 24
#define FRAME_QUEUE_SIZE CONFIG_ZEPHCORE_BLE_QUEUE_SIZE
#define BLE_TX_POWER 4
#define BLE_TX_RETRY_MS 20

/* BLE connection parameters */
#define BLE_DEFAULT_MIN_INTERVAL CONFIG_ZEPHCORE_BLE_CONN_MIN_INTERVAL
#define BLE_DEFAULT_MAX_INTERVAL CONFIG_ZEPHCORE_BLE_CONN_MAX_INTERVAL
#define BLE_DEFAULT_LATENCY      CONFIG_ZEPHCORE_BLE_CONN_LATENCY
#define BLE_DEFAULT_TIMEOUT      CONFIG_ZEPHCORE_BLE_CONN_TIMEOUT

/* TX timeout watchdog - reset ble_tx_in_progress if callback never fires */
#define BLE_TX_TIMEOUT_MS 2000

/* Advertising intervals (Apple Accessory Design Guidelines) */
#define BT_ADV_INTERVAL_FAST     CONFIG_ZEPHCORE_BLE_ADV_FAST_INTERVAL
#define BT_ADV_INTERVAL_SLOW     CONFIG_ZEPHCORE_BLE_ADV_SLOW_INTERVAL
#define BT_ADV_FAST_TIMEOUT_SEC  CONFIG_ZEPHCORE_BLE_ADV_FAST_TIMEOUT

/* ========== Frame type for queues ========== */

struct frame {
	uint16_t len;
	uint8_t buf[MAX_FRAME_SIZE];
};

/* ========== Static state ========== */

/* Callbacks to main */
static const struct ble_callbacks *ble_cbs;

/* Advertising data */
static char device_name[DEVICE_NAME_MAX];
static const uint8_t ad_flags = BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR;
static const int8_t ad_tx_power = BLE_TX_POWER;
static const uint8_t nus_uuid[] = { BT_UUID_NUS_SRV_VAL };
static struct bt_data ad[3];
static struct bt_data sd[1];
static size_t ad_len;
static size_t sd_len;

/* Queues — ISR-safe, no mutex needed */
K_MSGQ_DEFINE(ble_send_queue, sizeof(struct frame), FRAME_QUEUE_SIZE, 4);
K_MSGQ_DEFINE(ble_recv_queue, sizeof(struct frame), FRAME_QUEUE_SIZE, 4);

/* TX retry buffer - used when BLE returns -ENOMEM/-EAGAIN */
static struct frame tx_retry_frame;
static bool tx_retry_pending = false;

/* Connection state */
static struct bt_conn *current_conn;
static bool nus_notif_enabled;
static bool ble_tx_ready = false;
static bool ble_tx_in_progress = false;
static int64_t ble_tx_start_time = 0;

/* Active interface tracking */
static enum zephcore_iface active_iface = ZEPHCORE_IFACE_NONE;

/* Advertising state */
static bool adv_switching = false;
static bool adv_is_slow = false;

/* Runtime BLE passkey */
static uint32_t ble_passkey = CONFIG_ZEPHCORE_BLE_PASSKEY;

/* ========== Forward declarations ========== */

static void ble_tx_complete_cb(struct bt_conn *conn, void *user_data);
static int secure_nus_send(struct bt_conn *conn, const void *data, uint16_t len);
static void secure_nus_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);
static ssize_t secure_nus_rx_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				   const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
static void start_adv(void);
static void kick_tx_drain(void);

/* ========== GATT Service ========== */

/*
 * Secure NUS service with MITM-authenticated permissions.
 * Unlike the default NUS service, this requires PIN pairing before
 * any NUS communication is allowed (matches Arduino's SECMODE_ENC_WITH_MITM).
 */
BT_GATT_SERVICE_DEFINE(secure_nus_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_NUS_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_NUS_TX_CHAR,
		BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_NONE,
		NULL, NULL, NULL),
	BT_GATT_CCC(secure_nus_ccc_changed,
		BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN),
	BT_GATT_CHARACTERISTIC(BT_UUID_NUS_RX_CHAR,
		BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
		BT_GATT_PERM_WRITE_AUTHEN,
		NULL, secure_nus_rx_write, NULL),
);

static sys_slist_t secure_nus_cbs_list = SYS_SLIST_STATIC_INIT(&secure_nus_cbs_list);
STRUCT_SECTION_ITERABLE(bt_nus_inst, secure_nus) = {
	.svc = &secure_nus_svc,
	.cbs = &secure_nus_cbs_list,
};

/* ========== Work items ========== */

static void tx_drain_work_fn(struct k_work *work);
static void adv_slow_work_fn(struct k_work *work);

K_WORK_DELAYABLE_DEFINE(tx_drain_work, tx_drain_work_fn);
K_WORK_DELAYABLE_DEFINE(adv_slow_work, adv_slow_work_fn);

/* ========== TX completion callback ========== */

/*
 * TX completion callback - chains to next frame (event-driven like Arduino)
 * This is called by bt_gatt_notify_cb when the notification is sent over the air.
 */
static void ble_tx_complete_cb(struct bt_conn *conn, void *user_data)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(user_data);

	ble_tx_in_progress = false;

	/* Chain to next frame immediately via work queue */
	k_work_schedule(&tx_drain_work, K_NO_WAIT);
}

/* Helper function to send via our secure NUS TX characteristic with callback */
static int secure_nus_send(struct bt_conn *conn, const void *data, uint16_t len)
{
	struct bt_gatt_notify_params params = {
		.attr = &secure_nus_svc.attrs[1],  /* TX characteristic */
		.data = data,
		.len = len,
		.func = ble_tx_complete_cb,
		.user_data = NULL,
	};

	return bt_gatt_notify_cb(conn, &params);
}

/* ========== Device name and advertising data ========== */

static void build_device_name_and_adv(const char *name_from_prefs)
{
	if (name_from_prefs && name_from_prefs[0]) {
		size_t name_len = strnlen(name_from_prefs, sizeof(device_name) - 1);
		memcpy(device_name, name_from_prefs, name_len);
		device_name[name_len] = '\0';

		/* Apple BLE Accessory Design Guidelines: device name must not
		 * contain ':' or ';' characters. Replace with '-'.
		 */
		for (size_t i = 0; device_name[i]; i++) {
			if (device_name[i] == ':' || device_name[i] == ';') {
				device_name[i] = '-';
			}
		}
	} else {
		/* Fallback - should never happen since prefs.node_name has default */
		snprintf(device_name, sizeof(device_name), "MeshCore");
	}

	bt_set_name(device_name);

	ad[0].type = BT_DATA_FLAGS;
	ad[0].data_len = 1;
	ad[0].data = &ad_flags;
	ad[1].type = BT_DATA_TX_POWER;
	ad[1].data_len = 1;
	ad[1].data = (const uint8_t *)&ad_tx_power;
	ad[2].type = BT_DATA_UUID128_ALL;
	ad[2].data_len = sizeof(nus_uuid);
	ad[2].data = nus_uuid;
	ad_len = 3;

	sd[0].type = BT_DATA_NAME_COMPLETE;
	sd[0].data_len = (uint8_t)strlen(device_name);
	sd[0].data = (const uint8_t *)device_name;
	sd_len = 1;

	LOG_DBG("%s", device_name);
}

/* ========== Connection callbacks ========== */

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	if (err) {
		LOG_WRN("connection failed: %s err 0x%02x", addr, err);
		return;
	}
	LOG_DBG("%s", addr);
	current_conn = bt_conn_ref(conn);

	/* Cancel slow advertising work - we're connected now */
	k_work_cancel_delayable(&adv_slow_work);
	adv_is_slow = false;

	/* Request max data length (251 bytes) - must be before MTU exchange per BLE spec.
	 * Apple Accessory Design Guidelines: DLE before MTU handshake for best performance.
	 */
#if defined(CONFIG_BT_USER_DATA_LEN_UPDATE)
	{
		struct bt_conn_le_data_len_param data_len_param = {
			.tx_max_len = BT_GAP_DATA_LEN_MAX,
			.tx_max_time = BT_GAP_DATA_TIME_MAX,
		};
		int dle_err = bt_conn_le_data_len_update(conn, &data_len_param);
		if (dle_err) {
			LOG_WRN("Failed to request data length update: %d", dle_err);
		} else {
			LOG_INF("Requested max data length (251 bytes)");
		}
	}
#endif

	/* Proactively request authenticated encryption (MITM).
	 * - Bonded device: re-encrypts with stored keys (instant, no user interaction)
	 * - New device: triggers passkey pairing dialog on the central
	 *
	 * L3 = authenticated pairing with MITM protection. iOS/Android will
	 * negotiate Secure Connections (LESC) automatically when supported.
	 * L4 (SC-only) is NOT used because Windows never shows the passkey
	 * dialog for SC pairing — it only works with Legacy passkey entry.
	 *
	 * This is needed because some BLE stacks (Windows) don't automatically
	 * initiate pairing when they get "Insufficient Authentication" from a
	 * protected characteristic. By requesting security here, all platforms
	 * (iOS, Android, Windows) get the pairing prompt immediately.
	 *
	 * security_changed() callback handles the rest once encryption is up. */
	int sec_err = bt_conn_set_security(conn, BT_SECURITY_L3);
	if (sec_err && sec_err != -EALREADY) {
		LOG_WRN("Failed to request security: %d", sec_err);
	}

	/* Notify main of BLE connection */
	if (ble_cbs && ble_cbs->on_connected) {
		ble_cbs->on_connected();
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_DBG("%s reason 0x%02x", addr, reason);

	if (conn == current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	nus_notif_enabled = false;

	/* Reset BLE TX state */
	ble_tx_in_progress = false;
	ble_tx_ready = false;

	/* Clear interface state if BLE was active */
	if (active_iface == ZEPHCORE_IFACE_BLE) {
		active_iface = ZEPHCORE_IFACE_NONE;
		LOG_INF("active_iface = IFACE_NONE");
	}

	/* Clear queues and retry state */
	k_msgq_purge(&ble_send_queue);
	k_msgq_purge(&ble_recv_queue);
	tx_retry_pending = false;

	k_work_cancel_delayable(&tx_drain_work);

	/* Notify main of BLE disconnection */
	if (ble_cbs && ble_cbs->on_disconnected) {
		ble_cbs->on_disconnected();
	}
}

/* Flag to suppress recycled() callback during adv mode switch */
static void recycled(void)
{
	if (adv_switching || adv_is_slow) {
		LOG_DBG("suppressed (switching=%d slow=%d)",
			adv_switching, adv_is_slow);
		return;
	}
	LOG_DBG("restart advertising");
	start_adv();
}

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	if (err) {
		LOG_WRN("security failed: %s level %u err %d", addr, level, err);
		return;
	}
	LOG_INF("%s level %u", addr, level);

	/* For bonded reconnects, pairing_complete is NOT called - only security_changed.
	 * Enable TX when we have sufficient security (level 2+ = encrypted).
	 * This handles both fresh pairing (pairing_complete also sets it) and
	 * reconnects with existing bond.
	 */
	if (level >= BT_SECURITY_L2 && !ble_tx_ready) {
		LOG_INF("security established, enabling TX");
		ble_tx_ready = true;
		active_iface = ZEPHCORE_IFACE_BLE;

		/* Request our preferred connection parameters.
		 * This is the single place for conn param requests -
		 * fires for both fresh pairing and bonded reconnects. */
		struct bt_le_conn_param conn_param = {
			.interval_min = BLE_DEFAULT_MIN_INTERVAL,
			.interval_max = BLE_DEFAULT_MAX_INTERVAL,
			.latency = BLE_DEFAULT_LATENCY,
			.timeout = BLE_DEFAULT_TIMEOUT,
		};
		int param_err = bt_conn_le_param_update(conn, &conn_param);
		if (param_err) {
			LOG_WRN("Failed to request conn param update: %d", param_err);
		} else {
			LOG_INF("Requested conn params: %d-%dms interval, latency=%d",
				BLE_DEFAULT_MIN_INTERVAL * 5 / 4,
				BLE_DEFAULT_MAX_INTERVAL * 5 / 4,
				BLE_DEFAULT_LATENCY);
		}

		/* Don't kick TX here — MTU exchange may not be done yet.
		 * TX starts when client subscribes to CCC (secure_nus_ccc_changed),
		 * which is the last step and implies MTU is negotiated. */
	}
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	return true;
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
			    uint16_t latency, uint16_t timeout)
{
	ARG_UNUSED(conn);
	LOG_INF("BLE conn params updated: interval=%dms latency=%d timeout=%dms",
		interval * 5 / 4, latency, timeout * 10);
}

#if defined(CONFIG_BT_USER_DATA_LEN_UPDATE)
static void le_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info)
{
	ARG_UNUSED(conn);
	LOG_INF("BLE data length updated: TX=%u/%uus RX=%u/%uus",
		info->tx_max_len, info->tx_max_time,
		info->rx_max_len, info->rx_max_time);
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.recycled = recycled,
	.le_param_req = le_param_req,
	.le_param_updated = le_param_updated,
	.security_changed = security_changed,
#if defined(CONFIG_BT_USER_DATA_LEN_UPDATE)
	.le_data_len_updated = le_data_len_updated,
#endif
};

/* ========== Authentication callbacks ========== */

static uint32_t auth_app_passkey(struct bt_conn *conn)
{
	return ble_passkey;
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	ARG_UNUSED(conn);
	LOG_WRN("pairing cancelled");
}

static struct bt_conn_auth_cb auth_cb = {
	.passkey_display = auth_passkey_display,
	.cancel = auth_cancel,
	.app_passkey = auth_app_passkey,
};

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	ARG_UNUSED(conn);
	LOG_DBG("bonded=%d", bonded);

	/* Switch to BLE interface (fresh pairing only).
	 * Conn params and TX enable are handled in security_changed(),
	 * which fires for both fresh pairing and bonded reconnects. */
	if (active_iface == ZEPHCORE_IFACE_USB) {
		LOG_INF("switching from USB to BLE");
		/* Main handles USB state clearing via on_connected callback */
	}
	active_iface = ZEPHCORE_IFACE_BLE;
	LOG_INF("active_iface = IFACE_BLE");
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	LOG_WRN("pairing failed: reason %d", reason);
	bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
}

static struct bt_conn_auth_info_cb auth_info_cb = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed,
};

/* ========== TX drain work ========== */

static void kick_tx_drain(void)
{
	k_work_schedule(&tx_drain_work, K_NO_WAIT);
}

static void tx_drain_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	struct frame f;
	int err;

	/* USB TX path is handled in main — only BLE TX here */
	if (active_iface == ZEPHCORE_IFACE_USB) {
		/* Let main handle USB TX — just signal tx_idle if queue empty */
		if (k_msgq_num_used_get(&ble_send_queue) == 0) {
			if (ble_cbs && ble_cbs->on_tx_idle) {
				ble_cbs->on_tx_idle();
			}
		}
		return;
	}

	/*
	 * BLE TX path - Event-driven (like Arduino's HVN_TX_COMPLETE)
	 * Uses bt_gatt_notify_cb() callback to chain TX without polling.
	 * Re-entrancy guard prevents concurrent notify calls.
	 *
	 * IMPORTANT: Must wait for ble_tx_ready before sending. This is set in
	 * security_changed() after encryption is established. Sending before the
	 * connection is fully secured causes "No ATT channel for MTU" errors.
	 */
	if (!current_conn || !nus_notif_enabled || !ble_tx_ready) {
		LOG_DBG("tx_drain[BLE]: not ready (conn=%p notif=%d ready=%d)",
			current_conn, nus_notif_enabled, ble_tx_ready);
		return;
	}

	/* Re-entrancy guard - only one TX in flight at a time */
	if (ble_tx_in_progress) {
		/* TX timeout watchdog - if callback never fired, reset state */
		if ((k_uptime_get() - ble_tx_start_time) > BLE_TX_TIMEOUT_MS) {
			LOG_WRN("tx_drain[BLE]: TX timeout, resetting state");
			ble_tx_in_progress = false;
			/* Fall through to try next TX */
		} else {
			LOG_DBG("tx_drain[BLE]: TX in progress, callback will chain");
			return;
		}
	}

	/* Check retry buffer first */
	if (tx_retry_pending) {
		LOG_INF("tx_drain[BLE]: retrying len=%u hdr=0x%02x", (unsigned)tx_retry_frame.len, tx_retry_frame.buf[0]);
		ble_tx_in_progress = true;
		ble_tx_start_time = k_uptime_get();
		err = secure_nus_send(current_conn, tx_retry_frame.buf, tx_retry_frame.len);
		if (err == 0) {
			tx_retry_pending = false;
			LOG_INF("tx_drain[BLE]: retry success");
			return;  /* Callback will chain to next */
		} else if (err == -EAGAIN || err == -ENOMEM) {
			ble_tx_in_progress = false;
			LOG_DBG("tx_drain[BLE]: retry still busy, wait %dms", BLE_TX_RETRY_MS);
			k_work_schedule(&tx_drain_work, K_MSEC(BLE_TX_RETRY_MS));
			return;
		} else {
			ble_tx_in_progress = false;
			tx_retry_pending = false;
			LOG_WRN("tx_drain[BLE]: retry failed err=%d, dropped", err);
			/* Fall through to try next frame */
		}
	}

	/* Get next frame from queue */
	if (k_msgq_get(&ble_send_queue, &f, K_NO_WAIT) != 0) {
		/* TX queue empty - signal idle */
		if (ble_cbs && ble_cbs->on_tx_idle) {
			ble_cbs->on_tx_idle();
		}
		return;
	}

	LOG_DBG("tx_drain[BLE]: sending len=%u hdr=0x%02x queue=%u",
		(unsigned)f.len, f.buf[0], k_msgq_num_used_get(&ble_send_queue));

	/* Mark TX in progress before calling notify_cb */
	ble_tx_in_progress = true;
	ble_tx_start_time = k_uptime_get();
	err = secure_nus_send(current_conn, f.buf, f.len);

	if (err == 0) {
		/* Success - callback will chain to next */
		LOG_DBG("tx_drain[BLE]: queued for TX");
		return;
	} else if (err == -EAGAIN || err == -ENOMEM) {
		/* BLE buffer full - save for retry */
		ble_tx_in_progress = false;
		tx_retry_frame = f;
		tx_retry_pending = true;
		LOG_DBG("tx_drain[BLE]: BLE busy (err=%d), saved for retry", err);
		k_work_schedule(&tx_drain_work, K_MSEC(BLE_TX_RETRY_MS));
		return;
	} else {
		/* Other error - drop frame */
		ble_tx_in_progress = false;
		LOG_WRN("tx_drain[BLE]: send failed err=%d, dropped frame", err);
		return;
	}
}

/* ========== NUS service callbacks ========== */

static void secure_nus_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	bool enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_DBG("enabled=%d", enabled);
	nus_notif_enabled = enabled;
	if (enabled) {
		kick_tx_drain();
	}
}

static ssize_t secure_nus_rx_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				   const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(offset);
	ARG_UNUSED(flags);

	if (len == 0 || len > MAX_FRAME_SIZE) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	const uint8_t *data = (const uint8_t *)buf;
	uint8_t cmd = data[0];

	LOG_DBG("len=%u cmd=0x%02x", len, cmd);

	/* Notify main via callback */
	if (ble_cbs && ble_cbs->on_rx_frame) {
		ble_cbs->on_rx_frame(data, len);
	}

	return len;
}

/* ========== Advertising ========== */

static void start_adv(void)
{
	/* Start with fast 20ms advertising for quick discovery */
	struct bt_le_adv_param adv_param = {
		.id = BT_ID_DEFAULT,
		.options = BT_LE_ADV_OPT_CONN,
		.interval_min = BT_ADV_INTERVAL_FAST,
		.interval_max = BT_ADV_INTERVAL_FAST,
	};

	adv_is_slow = false;
	int err = bt_le_adv_start(&adv_param, ad, ad_len, sd, sd_len);
	if (err && err != -EALREADY) {
		LOG_ERR("adv start failed: %d", err);
	} else {
		LOG_INF("BLE advertising: fast mode (20ms) for %ds", BT_ADV_FAST_TIMEOUT_SEC);
		/* Schedule switch to slow mode after timeout */
		k_work_schedule(&adv_slow_work, K_SECONDS(BT_ADV_FAST_TIMEOUT_SEC));
	}
}

static void adv_slow_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	/* Only switch if not connected */
	if (current_conn) {
		return;
	}

	/* Stop current advertising and restart with slow interval.
	 * Set flag to prevent recycled() from calling start_adv() again. */
	adv_switching = true;
	bt_le_adv_stop();

	struct bt_le_adv_param adv_param = {
		.id = BT_ID_DEFAULT,
		.options = BT_LE_ADV_OPT_CONN,
		.interval_min = BT_ADV_INTERVAL_SLOW,
		.interval_max = BT_ADV_INTERVAL_SLOW,
	};

	int err = bt_le_adv_start(&adv_param, ad, ad_len, sd, sd_len);
	adv_switching = false;
	if (err && err != -EALREADY) {
		LOG_ERR("slow adv start failed: %d", err);
	} else {
		adv_is_slow = true;
		LOG_INF("BLE advertising: slow mode (546ms)");
	}
}

/* ========== Public API ========== */

void zephcore_ble_init(const struct ble_callbacks *cbs)
{
	ble_cbs = cbs;
	bt_conn_auth_cb_register(&auth_cb);
	bt_conn_auth_info_cb_register(&auth_info_cb);
}

void zephcore_ble_start(const char *name)
{
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	build_device_name_and_adv(name);
	LOG_DBG("init complete, starting adv");
	start_adv();
}

size_t zephcore_ble_send(const uint8_t *data, uint16_t len)
{
	if (len == 0 || len > MAX_FRAME_SIZE) {
		LOG_WRN("invalid len=%u", (unsigned)len);
		return 0;
	}

	/* Don't queue frames if no active transport */
	if (active_iface == ZEPHCORE_IFACE_BLE && !current_conn) {
		LOG_DBG("no BLE conn, dropping len=%u hdr=0x%02x",
			(unsigned)len, data[0]);
		return 0;
	}
	if (active_iface == ZEPHCORE_IFACE_NONE) {
		LOG_DBG("no active iface, dropping len=%u hdr=0x%02x",
			(unsigned)len, data[0]);
		return 0;
	}

	struct frame f;
	f.len = len;
	memcpy(f.buf, data, len);

	if (k_msgq_put(&ble_send_queue, &f, K_NO_WAIT) != 0) {
		LOG_WRN("queue full!");
		return 0;
	}

	LOG_DBG("queued len=%u hdr=0x%02x queue=%u",
		(unsigned)len, data[0], k_msgq_num_used_get(&ble_send_queue));

	kick_tx_drain();
	return len;
}

void zephcore_ble_set_enabled(bool enable)
{
	if (!enable) {
		/* Disconnect current connection if any */
		if (current_conn) {
			bt_conn_disconnect(current_conn,
					   BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		}
		/* Stop advertising */
		adv_switching = true;
		bt_le_adv_stop();
		adv_switching = false;
		adv_is_slow = false;
		k_work_cancel_delayable(&adv_slow_work);
		LOG_INF("BLE disabled");
	} else {
		/* Re-enable advertising */
		start_adv();
		LOG_INF("BLE enabled");
	}
}

bool zephcore_ble_is_active(void)
{
	return active_iface == ZEPHCORE_IFACE_BLE && current_conn != NULL && ble_tx_ready;
}

bool zephcore_ble_is_connected(void)
{
	return current_conn != NULL;
}

void zephcore_ble_set_passkey(uint32_t passkey)
{
	if (passkey >= 100000 && passkey <= 999999) {
		ble_passkey = passkey;
	} else {
		ble_passkey = CONFIG_ZEPHCORE_BLE_PASSKEY;
	}
	LOG_INF("BLE passkey updated to %06u (effective on next pairing)", ble_passkey);
}

uint32_t zephcore_ble_get_passkey(void)
{
	return ble_passkey;
}

enum zephcore_iface zephcore_ble_get_active_iface(void)
{
	return active_iface;
}

void zephcore_ble_set_active_iface(enum zephcore_iface iface)
{
	active_iface = iface;
}

struct k_msgq *zephcore_ble_get_recv_queue(void)
{
	return &ble_recv_queue;
}

struct k_msgq *zephcore_ble_get_send_queue(void)
{
	return &ble_send_queue;
}

void zephcore_ble_kick_tx(void)
{
	kick_tx_drain();
}

void zephcore_ble_disconnect(void)
{
	if (current_conn) {
		bt_conn_disconnect(current_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}
}

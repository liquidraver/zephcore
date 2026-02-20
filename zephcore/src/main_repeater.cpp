/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore - Repeater (USB CLI, Event-Driven)
 *
 * This is the main entry point for the repeater role.
 * Repeaters use USB serial CLI for configuration (no BLE).
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_repeater_main, CONFIG_ZEPHCORE_LORA_LOG_LEVEL);

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/drivers/hwinfo.h>
#include "oled_power.h"

/* USB CDC with 1200 baud touch detection (when not using auto-init) */
#if !IS_ENABLED(CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT)
#include <ZephyrRepeaterUSB.h>
#endif

#include <app/RepeaterDataStore.h>
#include <app/RepeaterMesh.h>
#include <adapters/clock/ZephyrRTCClock.h>
#include <ZephyrSensorManager.h>

/* Radio + mesh includes (shared header selects LR1110 or SX126x) */
#include <mesh/RadioIncludes.h>

/* LED configuration */
#if DT_NODE_HAS_PROP(DT_ALIAS(led0), gpios)
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#endif
#if DT_NODE_HAS_PROP(DT_ALIAS(led1), gpios)
#define LED1_NODE DT_ALIAS(led1)
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
#endif

/* USB CLI configuration */
#define USB_RING_BUF_SIZE 512
#define CLI_LINE_BUF_SIZE 256

/*
 * Event-driven mesh loop - replaces 50ms polling with true event signaling.
 * Events are signaled from ISR/callbacks, mesh loop wakes immediately.
 */
#define MESH_EVENT_LORA_RX       BIT(0)  /* LoRa packet received */
#define MESH_EVENT_LORA_TX_DONE  BIT(1)  /* LoRa TX complete */
#define MESH_EVENT_CLI_RX        BIT(2)  /* CLI command received */
#define MESH_EVENT_HOUSEKEEPING  BIT(3)  /* Periodic housekeeping (noise floor, etc.) */
#define MESH_EVENT_ALL           (MESH_EVENT_LORA_RX | MESH_EVENT_LORA_TX_DONE | MESH_EVENT_CLI_RX | MESH_EVENT_HOUSEKEEPING)

/* Housekeeping interval - infrequent to preserve power savings */
#define HOUSEKEEPING_INTERVAL_MS CONFIG_ZEPHCORE_HOUSEKEEPING_INTERVAL_MS

/* Event object for mesh loop */
static struct k_event mesh_events;

/* USB CDC state */
static const struct device *usb_dev;
static uint8_t usb_ring_buf_data[USB_RING_BUF_SIZE];
static struct ring_buf usb_ring_buf;
static char cli_line_buf[CLI_LINE_BUF_SIZE];
static char cli_reply_buf[256];
static uint16_t cli_line_idx;

/* Work items for event-driven processing */
static void cli_rx_work_fn(struct k_work *work);
static void housekeeping_timer_fn(struct k_timer *timer);
K_WORK_DEFINE(cli_rx_work, cli_rx_work_fn);

/* Housekeeping timer for periodic tasks (noise floor calibration, etc.) */
K_TIMER_DEFINE(housekeeping_timer, housekeeping_timer_fn, NULL);

/* Forward declarations */
#ifdef ZEPHCORE_LORA
static RepeaterMesh *repeater_mesh_ptr;
#endif

/* Print string to USB serial */
static void cli_print(const char *str)
{
	if (!usb_dev) return;
	while (*str) {
		uart_poll_out(usb_dev, *str++);
	}
}

/* USB CDC UART interrupt callback */
static void cli_uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			uint8_t buf[64];
			int recv_len = uart_fifo_read(dev, buf, sizeof(buf));
			if (recv_len > 0) {
				ring_buf_put(&usb_ring_buf, buf, recv_len);
				k_work_submit(&cli_rx_work);
			}
		}
	}
}

/* CLI RX work - processes line-based CLI commands
 * Matches Arduino behavior: echo each char, then "  -> reply" on enter
 */
static void cli_rx_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	uint8_t byte;

	while (ring_buf_get(&usb_ring_buf, &byte, 1) == 1) {
		/* Process command on \r OR \n (support echo from Linux) */
		if (byte == '\r' || byte == '\n') {
			if (cli_line_idx > 0) {
				cli_line_buf[cli_line_idx] = '\0';

				/* Debug: log received command */
				LOG_INF("CLI cmd len=%d: %.40s%s", cli_line_idx,
					cli_line_buf, cli_line_idx > 40 ? "..." : "");

				/* Process CLI command */
#ifdef ZEPHCORE_LORA
				if (repeater_mesh_ptr) {
					cli_reply_buf[0] = '\0';
					repeater_mesh_ptr->handleCommand(0, cli_line_buf, cli_reply_buf);
					if (cli_reply_buf[0] != '\0') {
						/* Arduino format: newline, then "  -> reply" */
						cli_print("\r\n  -> ");
						cli_print(cli_reply_buf);
					}
				}
#endif
				cli_line_idx = 0;
			}
			/* New line for next command */
			cli_print("\r\n");
		} else if (byte == 0x7F || byte == 0x08) {
			/* Backspace - echo backspace sequence */
			if (cli_line_idx > 0) {
				cli_line_idx--;
				if (usb_dev) {
					uart_poll_out(usb_dev, '\b');
					uart_poll_out(usb_dev, ' ');
					uart_poll_out(usb_dev, '\b');
				}
			}
		} else if (cli_line_idx < sizeof(cli_line_buf) - 1) {
			/* Echo character back (like Arduino) */
			if (usb_dev) {
				uart_poll_out(usb_dev, byte);
			}
			cli_line_buf[cli_line_idx++] = (char)byte;
		}
	}
}

/* Housekeeping timer callback - signals event to wake mesh loop periodically */
static void housekeeping_timer_fn(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_event_post(&mesh_events, MESH_EVENT_HOUSEKEEPING);
}

#ifdef ZEPHCORE_LORA
/* LoRa RX callback - called from ISR context when packet received */
static void lora_rx_callback(void *user_data)
{
	ARG_UNUSED(user_data);
	k_event_post(&mesh_events, MESH_EVENT_LORA_RX);
}

/* LoRa TX complete callback */
static void lora_tx_done_callback(void *user_data)
{
	ARG_UNUSED(user_data);
	k_event_post(&mesh_events, MESH_EVENT_LORA_TX_DONE);
}
#endif

/* Global instances */
static mesh::ZephyrRTCClock rtc_clock;
static RepeaterDataStore data_store;

#ifdef ZEPHCORE_LORA
static mesh::ZephyrBoard zephyr_board;

/* Radio prefs — initialized with defaults in main(), updated after mesh.begin()
 * loads persisted prefs. Passed to radio adapter at static construction time. */
static NodePrefs radio_prefs;

#if IS_ENABLED(CONFIG_ZEPHCORE_RADIO_LR1110)
/* LR1110 via Zephyr LoRa driver */
static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
static mesh::LR1110Radio lora_radio(lora_dev, zephyr_board, &radio_prefs);
#else
/* SX126x via Zephyr LoRa driver */
static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
static mesh::SX126xRadio lora_radio(lora_dev, zephyr_board, &radio_prefs);
#endif

static mesh::ZephyrMillisecondClock ms_clock;
static mesh::ZephyrRNG zephyr_rng;
static mesh::SimpleMeshTables mesh_tables;

/* RepeaterMesh requires: board, radio, ms_clock, rng, rtc, tables */
static RepeaterMesh repeater_mesh(zephyr_board, lora_radio, ms_clock, zephyr_rng, rtc_clock, mesh_tables);
#endif

/* Repeater event loop */
static void repeater_event_loop(void)
{
	LOG_INF("starting event-driven loop");

	/* Print startup banner (no prompt - Arduino style) */
	cli_print("\r\n=== ZephCore Repeater ===\r\n");

	/* Start housekeeping timer for periodic maintenance tasks */
	k_timer_start(&housekeeping_timer, K_MSEC(HOUSEKEEPING_INTERVAL_MS),
		      K_MSEC(HOUSEKEEPING_INTERVAL_MS));

	for (;;) {
		/* Wait for any mesh event - blocks until signaled */
		uint32_t events = k_event_wait(&mesh_events, MESH_EVENT_ALL, false, K_FOREVER);
		k_event_clear(&mesh_events, events);

#ifdef ZEPHCORE_LORA
		if (repeater_mesh_ptr) {
			repeater_mesh_ptr->loop();
		}
#endif
	}
}

int main(void)
{
	/* Initialize radio prefs with safe defaults before anything else */
	initNodePrefs(&radio_prefs);
	strcpy(radio_prefs.node_name, "Repeater");

	/* Wait for USB CDC to enumerate before any logging */
	k_sleep(K_MSEC(2000));
	LOG_INF("=== ZephCore Repeater starting ===");

	/* Configure LEDs */
#if DT_NODE_HAS_PROP(DT_ALIAS(led0), gpios)
	if (gpio_is_ready_dt(&led0)) {
		gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
	}
#endif
#if DT_NODE_HAS_PROP(DT_ALIAS(led1), gpios)
	if (gpio_is_ready_dt(&led1)) {
		gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
	}
#endif

	/* Initialize repeater data store */
	if (!data_store.begin()) {
		LOG_ERR("RepeaterDataStore init failed");
	}

	/* Initialize sensor manager */
	sensor_manager_init();

	/* Set GPS to repeater mode: power off now, wake every 48h for time sync only.
	 * This prevents GPS from draining power on boards that have it (e.g., Wio Tracker). */
	if (gps_is_available()) {
		gps_set_repeater_mode(true);
	}

	/* Put OLED display to sleep */
	oled_sleep();

	/* Log environment sensor availability */
	if (env_sensors_available()) {
		LOG_INF("Environment sensors available");
	}

#ifdef ZEPHCORE_LORA
	repeater_mesh_ptr = &repeater_mesh;

	/* Initialize mesh event object BEFORE begin() — radio callbacks post events */
	k_event_init(&mesh_events);

	/* Set LoRa callbacks for event-driven packet processing */
	lora_radio.setRxCallback(lora_rx_callback, nullptr);
	lora_radio.setTxDoneCallback(lora_tx_done_callback, nullptr);

	/* Load or generate identity BEFORE begin() */
	mesh::LocalIdentity self_identity;
	if (!data_store.loadIdentity(self_identity)) {
		LOG_INF("No identity found, generating new keypair...");
		self_identity = mesh::LocalIdentity(&zephyr_rng);
		/* Ensure pub_key[0] is not reserved (0x00 or 0xFF) */
		int count = 0;
		while (count < 10 && (self_identity.pub_key[0] == 0x00 || self_identity.pub_key[0] == 0xFF)) {
			self_identity = mesh::LocalIdentity(&zephyr_rng);
			count++;
		}
		data_store.saveIdentity(self_identity);
		LOG_INF("New identity saved");
	}
	repeater_mesh.self_id = self_identity;

	/* Log repeater ID (first 8 bytes of public key) */
	LOG_INF("Repeater ID: %02x%02x%02x%02x%02x%02x%02x%02x...",
		self_identity.pub_key[0], self_identity.pub_key[1],
		self_identity.pub_key[2], self_identity.pub_key[3],
		self_identity.pub_key[4], self_identity.pub_key[5],
		self_identity.pub_key[6], self_identity.pub_key[7]);

	/* Start mesh with data store - this loads prefs, ACL, regions */
	repeater_mesh.begin(&data_store);

	/* Generate default node name from hardware device ID if not set */
	NodePrefs* prefs = repeater_mesh.getNodePrefs();
	if (strlen(prefs->node_name) == 0 || strcmp(prefs->node_name, "Repeater") == 0) {
		uint8_t dev_id[8];
		ssize_t id_len = hwinfo_get_device_id(dev_id, sizeof(dev_id));
		if (id_len >= 4) {
			snprintf(prefs->node_name, sizeof(prefs->node_name),
				 "Repeater-%02X%02X%02X%02X", dev_id[0], dev_id[1], dev_id[2], dev_id[3]);
		}
	}

	/* Apply RX boost setting from prefs (overrides radio default if user changed it) */
	lora_radio.setRxBoost(prefs->rx_boost != 0);

	/* Send initial zero-hop advertisement after boot */
	LOG_INF("Sending initial advertisement...");
	repeater_mesh.sendSelfAdvertisement(500, false);  /* 500ms delay, zero-hop */
#endif

	/* Initialize USB CDC for CLI with 1200 baud touch detection */
#if !IS_ENABLED(CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT)
	zephcore_usbd_init();
#endif
	usb_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
	if (device_is_ready(usb_dev)) {
		LOG_INF("USB CDC device ready: %s", usb_dev->name);
		ring_buf_init(&usb_ring_buf, sizeof(usb_ring_buf_data), usb_ring_buf_data);

		/* Set up UART interrupt callback */
		uart_irq_callback_set(usb_dev, cli_uart_isr);
		uart_irq_rx_enable(usb_dev);
	} else {
		LOG_ERR("USB CDC device not ready");
		usb_dev = NULL;
	}

	/*
	 * Event-driven architecture: main thread runs repeater event loop.
	 * No BLE - all configuration via USB serial CLI.
	 */
#ifdef ZEPHCORE_LORA
	repeater_event_loop();  /* Never returns */
#else
	for (;;) {
		k_sleep(K_FOREVER);
	}
#endif

	return 0;
}

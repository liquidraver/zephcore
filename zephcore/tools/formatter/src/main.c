/*
 * ZephCore Universal Flash Formatter
 *
 * Erases all filesystem partitions (NVS + LittleFS + QSPI if present)
 * and reboots into Adafruit UF2 DFU mode for clean firmware installation.
 *
 * Partition addresses come from the board's devicetree overlay — no
 * hardcoded addresses, works for both s140 v6.1.1 and v7.3.0 boards.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/reboot.h>

#if defined(CONFIG_SOC_SERIES_NRF52X) || defined(CONFIG_SOC_SERIES_NRF52)
#include <hal/nrf_power.h>
#endif

/* Adafruit UF2 bootloader magic — enter mass storage DFU mode */
#define BOOTLOADER_DFU_UF2_MAGIC  0x57

/* ── LED feedback (optional) ─────────────────────────────────── */

#if DT_NODE_EXISTS(DT_ALIAS(led0))
#define HAS_LED 1
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static void led_init(void)  { gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE); }
static void led_on(void)    { gpio_pin_set_dt(&led, 1); }
static void led_off(void)   { gpio_pin_set_dt(&led, 0); }
#else
#define HAS_LED 0
static void led_init(void)  {}
static void led_on(void)    {}
static void led_off(void)   {}
#endif

/* ── Partition erase helper ──────────────────────────────────── */

static int erase_partition(uint8_t id, const char *name)
{
	const struct flash_area *fa;
	int rc;

	rc = flash_area_open(id, &fa);
	if (rc) {
		printk("  %s: open failed (rc %d) — skipped\n", name, rc);
		return rc;
	}

	printk("  %s: erasing 0x%lx - 0x%lx (%u KB)...",
	       name,
	       (unsigned long)fa->fa_off,
	       (unsigned long)(fa->fa_off + fa->fa_size),
	       (unsigned)(fa->fa_size / 1024));

	rc = flash_area_erase(fa, 0, fa->fa_size);
	if (rc) {
		printk(" FAILED (rc %d)\n", rc);
	} else {
		printk(" OK\n");
	}

	flash_area_close(fa);
	return rc;
}

/* ── Main ────────────────────────────────────────────────────── */

int main(void)
{
	int errors = 0;

	/* Brief delay for USB CDC to enumerate */
	k_msleep(2000);

	printk("\n");
	printk("=== ZephCore Flash Formatter ===\n");
	printk("\n");

	led_init();
	led_on();

	/* ── Erase internal NVS partition (BLE bonds + settings) ── */
	if (erase_partition(FIXED_PARTITION_ID(storage_partition), "NVS (storage)")) {
		errors++;
	}

	/* ── Erase internal LittleFS partition (identity, contacts, channels) ── */
	if (erase_partition(FIXED_PARTITION_ID(lfs_partition), "LittleFS (lfs)")) {
		errors++;
	}

	/* ── Erase external QSPI flash (if present in devicetree) ── */
#if DT_NODE_EXISTS(DT_NODELABEL(qspi_storage_partition))
	if (erase_partition(FIXED_PARTITION_ID(qspi_storage_partition), "QSPI external")) {
		errors++;
	}
#else
	printk("  QSPI: not present on this board — skipped\n");
#endif

	led_off();

	/* ── Summary ── */
	printk("\n");
	if (errors) {
		printk("Completed with %d error(s).\n", errors);
	} else {
		printk("All partitions erased successfully.\n");
	}

	printk("Rebooting into UF2 DFU mode...\n");
	printk("You can now drag-drop your new firmware UF2.\n");
	k_msleep(500);

	/* ── Reboot to UF2 bootloader ── */
#if defined(CONFIG_SOC_SERIES_NRF52X) || defined(CONFIG_SOC_SERIES_NRF52)
	nrf_power_gpregret_set(NRF_POWER, 0, BOOTLOADER_DFU_UF2_MAGIC);
#endif
	sys_reboot(SYS_REBOOT_COLD);

	return 0; /* never reached */
}

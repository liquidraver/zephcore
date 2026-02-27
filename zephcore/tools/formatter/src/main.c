/*
 * ZephCore Universal Flash Formatter
 *
 * Erases all filesystem partitions and reboots into Adafruit UF2 DFU mode
 * for clean firmware installation.
 *
 * nRF52 builds use hardcoded flash addresses (identical across all boards)
 * so a single UF2 works on every board with the same SoftDevice version.
 * QSPI is compile-time conditional — included only when building for a
 * QSPI-capable board target (pin config is board-specific).
 *
 * Non-nRF52 builds use DTS FIXED_PARTITION_EXISTS guards as before.
 *
 * Build:
 *   SD v7:        west build -b t1000_e/nrf52840        zephcore/tools/formatter --pristine
 *   SD v6:        west build -b rak4631/nrf52840         zephcore/tools/formatter --pristine
 *   SD v7 + QSPI: west build -b wio_tracker_l1/nrf52840 zephcore/tools/formatter --pristine
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
#define IS_NRF52 1
#else
#define IS_NRF52 0
#endif

/* Adafruit UF2 bootloader magic — enter mass storage DFU mode */
#define BOOTLOADER_DFU_UF2_MAGIC  0x57

/*
 * nRF52840 Arduino MeshCore partition addresses.
 * These are IDENTICAL across all nRF52 boards (SD v6 and v7):
 *   ExtraFS    @ 0xD4000 (100KB) — contacts, channels, blobs
 *   InternalFS @ 0xED000 (28KB)  — prefs, identity, BLE settings
 *
 * Only the SoftDevice/app boundary differs (v6=0x26000, v7=0x27000),
 * which doesn't affect the formatter since we don't touch app flash.
 */
#define NRF52_EXTRAFS_OFF       0xD4000
#define NRF52_EXTRAFS_SIZE      0x19000   /* 100KB */
#define NRF52_INTERNALFS_OFF    0xED000
#define NRF52_INTERNALFS_SIZE   0x7000    /* 28KB */

/* ── LED feedback (optional — may not match actual board) ──── */

#if DT_NODE_EXISTS(DT_ALIAS(led0)) && !IS_NRF52
/* Only use DTS LED on non-nRF52 (board-specific builds).
 * On nRF52 universal builds, skip LED to avoid toggling
 * wrong pins on boards other than the build target. */
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

/* ── Flash erase helpers ───────────────────────────────────── */

#if IS_NRF52
/**
 * Erase a region of internal flash by absolute address.
 * Works on any nRF52840 board regardless of DTS.
 */
static int erase_region(const struct device *dev, off_t offset, size_t size,
			const char *name)
{
	printk("  %s: erasing 0x%lx - 0x%lx (%u KB)...",
	       name, (unsigned long)offset,
	       (unsigned long)(offset + size),
	       (unsigned)(size / 1024));

	int rc = flash_erase(dev, offset, size);
	if (rc) {
		printk(" FAILED (rc %d)\n", rc);
	} else {
		printk(" OK\n");
	}
	return rc;
}
#endif /* IS_NRF52 */

#if !IS_NRF52
/**
 * Erase a DTS partition by flash_area ID (non-nRF52 boards).
 */
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
#endif /* !IS_NRF52 */

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

#if IS_NRF52
	/*
	 * nRF52 universal path: hardcoded addresses, no DTS dependency.
	 * This binary works on ANY nRF52840 board with the same SoftDevice
	 * version (determines UF2 load address, not erase targets).
	 */
	const struct device *flash_dev = DEVICE_DT_GET(DT_NODELABEL(flash_controller));

	if (!device_is_ready(flash_dev)) {
		printk("  ERROR: flash device not ready!\n");
		errors++;
	} else {
		if (erase_region(flash_dev, NRF52_EXTRAFS_OFF,
				 NRF52_EXTRAFS_SIZE, "ExtraFS (contacts/channels)")) {
			errors++;
		}
		if (erase_region(flash_dev, NRF52_INTERNALFS_OFF,
				 NRF52_INTERNALFS_SIZE, "InternalFS (prefs/identity)")) {
			errors++;
		}
	}

	/* QSPI: try to erase if present via flash_area (compiled in by DTS).
	 * On boards without QSPI, FIXED_PARTITION_EXISTS is false at compile
	 * time so this block is excluded — no runtime probe needed. */
#if FIXED_PARTITION_EXISTS(qspi_storage_partition)
	{
		const struct flash_area *fa;
		int rc = flash_area_open(FIXED_PARTITION_ID(qspi_storage_partition), &fa);
		if (rc == 0) {
			printk("  QSPI: erasing 0x%lx (%u KB, may take a while)...",
			       (unsigned long)fa->fa_off,
			       (unsigned)(fa->fa_size / 1024));
			rc = flash_area_erase(fa, 0, fa->fa_size);
			printk(rc ? " FAILED (rc %d)\n" : " OK\n", rc);
			if (rc) errors++;
			flash_area_close(fa);
		} else {
			printk("  QSPI: not accessible (rc %d) — skipped\n", rc);
		}
	}
#else
	printk("  QSPI: not present in build target — skipped\n");
#endif

#else /* !IS_NRF52 */
	/*
	 * Non-nRF52 path: use DTS partitions (board-specific builds).
	 */
#if FIXED_PARTITION_EXISTS(storage_partition)
	if (erase_partition(FIXED_PARTITION_ID(storage_partition), "NVS (storage)")) {
		errors++;
	}
#endif

#if FIXED_PARTITION_EXISTS(lfs_partition)
	if (erase_partition(FIXED_PARTITION_ID(lfs_partition), "LittleFS (lfs)")) {
		errors++;
	}
#endif

#if FIXED_PARTITION_EXISTS(qspi_storage_partition)
	if (erase_partition(FIXED_PARTITION_ID(qspi_storage_partition), "QSPI external")) {
		errors++;
	}
#else
	printk("  QSPI: not present on this board — skipped\n");
#endif

#endif /* IS_NRF52 */

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
#if IS_NRF52
	nrf_power_gpregret_set(NRF_POWER, 0, BOOTLOADER_DFU_UF2_MAGIC);
#endif
	sys_reboot(SYS_REBOOT_COLD);

	return 0; /* never reached */
}

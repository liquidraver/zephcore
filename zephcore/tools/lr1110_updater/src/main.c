/*
 * ZephCore LR1110 Firmware Updater
 * SPDX-License-Identifier: Apache-2.0
 *
 * Standalone tool that updates the LR1110 radio firmware to v0x0401.
 * Flash this UF2 first, let it update the radio, then flash ZephCore main firmware.
 *
 * Firmware image source: https://github.com/Lora-net/radio_firmware_images
 * Bootloader protocol source: https://github.com/Lora-net/lr1110_driver
 * License: Clear BSD (Semtech Corporation 2021-2023)
 *
 * Update sequence (matches Semtech's official lr1110_updater_tool):
 *   1. Hardware reset LR1110
 *   2. Read version (GetVersion 0x0101) — type tells us firmware vs bootloader
 *   3. If already running 0x0401 → skip, reboot to UF2 DFU
 *   4. Reboot into bootloader mode (Reboot 0x8005 with stay=true)
 *   5. Verify we're in bootloader (type == 0xDF)
 *   6. Erase flash (EraseFlash 0x8000) — ~2.5 seconds
 *   7. Write firmware (WriteFlashEncrypted 0x8003) — 61320 words in 64-word chunks
 *   8. Reboot into firmware (Reboot 0x8005 with stay=false)
 *   9. Verify new firmware version == 0x0401
 *  10. Reboot MCU into UF2 DFU mode for main firmware flash
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/reboot.h>

#include "lr11xx_hal_updater.h"
#include "lr1110_bootloader.h"
#include "lr1110_transceiver_0401.h"

#if defined(CONFIG_SOC_SERIES_NRF52X) || defined(CONFIG_SOC_SERIES_NRF52)
#include <hal/nrf_power.h>
#endif

/* Adafruit UF2 bootloader magic — enter mass storage DFU mode */
#define BOOTLOADER_DFU_UF2_MAGIC 0x57

/* Target firmware version */
#define TARGET_FW_VERSION LR11XX_FIRMWARE_VERSION  /* 0x0401 */

/* LR1110 type field values */
#define LR1110_TYPE_TRANSCEIVER  0x01
#define LR1110_TYPE_BOOTLOADER   0xDF

/* ── LED feedback (optional) ─────────────────────────────────── */

#if DT_NODE_EXISTS(DT_ALIAS(led0))
#define HAS_LED 1
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static void led_init(void)  { gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE); }
static void led_on(void)    { gpio_pin_set_dt(&led, 1); }
static void led_off(void)   { gpio_pin_set_dt(&led, 0); }
static void led_toggle(void) { gpio_pin_toggle_dt(&led); }
#else
#define HAS_LED 0
static void led_init(void)  {}
static void led_on(void)    {}
static void led_off(void)   {}
static void led_toggle(void) {}
#endif

/* ── Helpers ──────────────────────────────────────────────────── */

static void reboot_to_uf2(void)
{
	printk("\nRebooting into UF2 DFU mode...\n");
	printk("You can now drag-drop ZephCore firmware UF2.\n");
	k_msleep(500);

#if defined(CONFIG_SOC_SERIES_NRF52X) || defined(CONFIG_SOC_SERIES_NRF52)
	nrf_power_gpregret_set(NRF_POWER, 0, BOOTLOADER_DFU_UF2_MAGIC);
#endif
	sys_reboot(SYS_REBOOT_COLD);
}

static void fatal_error(const char *msg)
{
	printk("\n!!! FATAL: %s\n", msg);
	printk("Please power cycle the device and try again.\n");
	led_off();

	/* Blink LED rapidly to indicate error */
	while (1) {
		led_toggle();
		k_msleep(200);
	}
}

/* ── Main ─────────────────────────────────────────────────────── */

int main(void)
{
	lr1110_bootloader_version_t version = { 0 };
	void *ctx;
	lr1110_status_t rc;

	/* Brief delay for USB CDC to enumerate */
	k_msleep(2000);

	printk("\n");
	printk("============================================\n");
	printk("  ZephCore LR1110 Firmware Updater\n");
	printk("  Target: transceiver FW 0x%04X\n", TARGET_FW_VERSION);
	printk("  Image:  %u words (%u KB)\n",
	       LR11XX_FIRMWARE_IMAGE_SIZE,
	       (LR11XX_FIRMWARE_IMAGE_SIZE * 4) / 1024);
	printk("============================================\n");
	printk("\n");

	led_init();
	led_on();

	/* ── Step 1: Initialize HAL ── */
	printk("[1/10] Initializing SPI and GPIOs...\n");
	if (lr1110_updater_hal_init() != 0) {
		fatal_error("HAL init failed");
	}
	ctx = lr1110_updater_get_context();

	/* ── Step 2: Hardware reset ── */
	printk("[2/10] Hardware reset LR1110...\n");
	if (lr1110_updater_hw_reset() != 0) {
		fatal_error("Hardware reset failed (BUSY stuck)");
	}

	/* ── Step 3: Read current version ── */
	printk("[3/10] Reading current firmware version...\n");
	rc = lr1110_bootloader_get_version(ctx, &version);
	if (rc != LR1110_STATUS_OK) {
		fatal_error("GetVersion failed");
	}

	printk("  HW   = 0x%02X\n", version.hw);
	printk("  TYPE = 0x%02X", version.type);
	if (version.type == LR1110_TYPE_TRANSCEIVER) {
		printk(" (transceiver firmware)\n");
	} else if (version.type == LR1110_TYPE_BOOTLOADER) {
		printk(" (bootloader — no firmware loaded)\n");
	} else {
		printk(" (unknown)\n");
	}
	printk("  FW   = 0x%04X\n", version.fw);

	/* ── Step 4: Check if update needed ── */
	if (version.type == LR1110_TYPE_TRANSCEIVER && version.fw == TARGET_FW_VERSION) {
		printk("\nAlready running target firmware 0x%04X — no update needed!\n",
		       TARGET_FW_VERSION);
		led_off();
		reboot_to_uf2();
		return 0;
	}

	if (version.type == LR1110_TYPE_TRANSCEIVER) {
		printk("\n[4/10] Current FW 0x%04X → updating to 0x%04X\n",
		       version.fw, TARGET_FW_VERSION);

		/* Force into bootloader mode via hardware reset with BUSY held LOW.
		 * This is the official Semtech approach (lr1110_updater_tool).
		 * Cannot use the software reboot command (0x8005) because that's
		 * a bootloader-mode opcode — we're in firmware mode (0x0118). */
		printk("[5/10] Forcing LR1110 into bootloader mode...\n");
		if (lr1110_updater_reset_to_bootloader() != 0) {
			fatal_error("Failed to enter bootloader mode");
		}
	} else if (version.type == LR1110_TYPE_BOOTLOADER) {
		printk("\n[4/10] Already in bootloader mode — proceeding with flash\n");
		printk("[5/10] (skipped — already in bootloader)\n");
	} else {
		fatal_error("Unknown chip type — cannot proceed");
	}

	/* ── Step 5: Verify bootloader mode ── */
	rc = lr1110_bootloader_get_version(ctx, &version);
	if (rc != LR1110_STATUS_OK) {
		fatal_error("GetVersion in bootloader failed");
	}

	printk("  Bootloader: HW=0x%02X TYPE=0x%02X FW=0x%04X\n",
	       version.hw, version.type, version.fw);

	if (version.type != LR1110_TYPE_BOOTLOADER) {
		fatal_error("Not in bootloader mode after reboot");
	}

	/* ── Step 6: Read chip identity ── */
	{
		lr1110_bootloader_pin_t pin = { 0 };
		lr1110_bootloader_chip_eui_t chip_eui = { 0 };
		lr1110_bootloader_join_eui_t join_eui = { 0 };

		lr1110_bootloader_read_pin(ctx, pin);
		lr1110_bootloader_read_chip_eui(ctx, chip_eui);
		lr1110_bootloader_read_join_eui(ctx, join_eui);

		printk("  PIN     = 0x%02X%02X%02X%02X\n",
		       pin[0], pin[1], pin[2], pin[3]);
		printk("  ChipEUI = 0x%02X%02X%02X%02X%02X%02X%02X%02X\n",
		       chip_eui[0], chip_eui[1], chip_eui[2], chip_eui[3],
		       chip_eui[4], chip_eui[5], chip_eui[6], chip_eui[7]);
		printk("  JoinEUI = 0x%02X%02X%02X%02X%02X%02X%02X%02X\n",
		       join_eui[0], join_eui[1], join_eui[2], join_eui[3],
		       join_eui[4], join_eui[5], join_eui[6], join_eui[7]);
	}

	/* ── Step 7: Erase flash ── */
	printk("\n[6/10] Erasing LR1110 flash (~2.5 seconds)...\n");
	led_toggle();

	rc = lr1110_bootloader_erase_flash(ctx);
	if (rc != LR1110_STATUS_OK) {
		fatal_error("Flash erase failed");
	}
	printk("  Flash erase complete!\n");

	/* ── Step 8: Write firmware image ── */
	printk("[7/10] Writing firmware (%u words = %u KB)...\n",
	       LR11XX_FIRMWARE_IMAGE_SIZE,
	       (LR11XX_FIRMWARE_IMAGE_SIZE * 4) / 1024);

	/* Progress tracking */
	uint32_t total = LR11XX_FIRMWARE_IMAGE_SIZE;
	uint32_t chunk_size = 64; /* 64 uint32_t words per write */
	uint32_t num_chunks = (total + chunk_size - 1) / chunk_size;
	uint32_t progress_step = num_chunks / 10; /* Print every 10% */
	if (progress_step == 0) progress_step = 1;

	uint32_t remaining = total;
	uint32_t offset = 0;
	uint32_t chunk_idx = 0;

	while (remaining > 0) {
		uint8_t this_chunk = (remaining > chunk_size)
			? (uint8_t)chunk_size : (uint8_t)remaining;

		rc = lr1110_bootloader_write_flash_encrypted(
			ctx, offset,
			&lr11xx_firmware_image[chunk_idx * chunk_size],
			this_chunk);

		if (rc != LR1110_STATUS_OK) {
			printk("\n");
			printk("ERROR: Write failed at offset 0x%08X (chunk %u/%u)\n",
			       offset, chunk_idx + 1, num_chunks);
			fatal_error("Firmware write failed");
		}

		/* Progress feedback */
		if ((chunk_idx % progress_step) == 0) {
			uint32_t pct = (chunk_idx * 100) / num_chunks;
			printk("  %3u%%  (%u / %u words)\n",
			       pct, total - remaining + this_chunk, total);
			led_toggle();
		}

		offset += this_chunk * sizeof(uint32_t);
		remaining -= this_chunk;
		chunk_idx++;
	}

	printk("  100%%  (%u / %u words)\n", total, total);
	printk("  Firmware write complete!\n");

	/* ── Step 9: Reboot into firmware ── */
	printk("\n[8/10] Rebooting LR1110 into new firmware...\n");
	lr1110_bootloader_reboot(ctx, false);  /* stay_in_bootloader = false */

	/* Wait for firmware boot (273ms typical) */
	k_msleep(500);

	/* Re-reset to ensure clean state */
	if (lr1110_updater_hw_reset() != 0) {
		fatal_error("Reset after firmware flash failed");
	}

	/* ── Step 10: Verify new firmware ── */
	printk("[9/10] Verifying new firmware version...\n");
	rc = lr1110_bootloader_get_version(ctx, &version);
	if (rc != LR1110_STATUS_OK) {
		fatal_error("GetVersion after flash failed");
	}

	printk("  HW   = 0x%02X\n", version.hw);
	printk("  TYPE = 0x%02X", version.type);
	if (version.type == LR1110_TYPE_TRANSCEIVER) {
		printk(" (transceiver firmware)\n");
	} else if (version.type == LR1110_TYPE_BOOTLOADER) {
		printk(" (bootloader — firmware not running!)\n");
	} else {
		printk(" (unknown)\n");
	}
	printk("  FW   = 0x%04X\n", version.fw);

	if (version.type == LR1110_TYPE_TRANSCEIVER && version.fw == TARGET_FW_VERSION) {
		printk("\n============================================\n");
		printk("  UPDATE SUCCESSFUL!\n");
		printk("  LR1110 firmware: 0x%04X\n", version.fw);
		printk("============================================\n");
		led_on();
	} else {
		printk("\nWARNING: Expected FW 0x%04X but got TYPE=0x%02X FW=0x%04X\n",
		       TARGET_FW_VERSION, version.type, version.fw);
		printk("The update may have failed. Try again.\n");
		led_off();
	}

	/* ── Reboot to UF2 DFU ── */
	printk("\n[10/10] Done! Rebooting to UF2 DFU mode...\n");
	reboot_to_uf2();

	return 0; /* never reached */
}

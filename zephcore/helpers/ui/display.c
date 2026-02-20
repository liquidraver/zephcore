/*
 * ZephCore - Display Abstraction (CFB)
 * Copyright (c) 2025 ZephCore
 * SPDX-License-Identifier: Apache-2.0
 *
 * Wraps Zephyr's Character Framebuffer (CFB) subsystem.
 * Auto-detects any Zephyr-supported display from devicetree:
 *   1. "zephyr,display" chosen node (standard — works for any display)
 *   2. Legacy nodelabels: sh1106, ssd1306 (backwards compat)
 *
 * Resolution is queried from the driver at runtime — no hardcoded
 * dimensions.  Layout code should use mc_display_width/height().
 *
 * Auto-off timer turns display off after CONFIG_ZEPHCORE_UI_DISPLAY_AUTO_OFF_MS.
 */

#include "display.h"
#include "doom_game.h"

#include <zephyr/device.h>
#include <zephyr/display/cfb.h>
#include <zephyr/drivers/display.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <stdio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mc_display, CONFIG_ZEPHCORE_BOARD_LOG_LEVEL);

/* ========== State ========== */

static const struct device *disp_dev;
static bool disp_on;
static bool disp_initialized;

/* Runtime display geometry (queried from driver) */
static uint16_t disp_width;
static uint16_t disp_height;
static uint8_t  font_w;
static uint8_t  font_h;
static bool     is_epd;       /* true for e-paper displays */

/* Auto-off work */
static struct k_work_delayable auto_off_work;

static void auto_off_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	/* Don't blank display while Doom easter egg is playing */
	if (doom_game_is_running()) {
		return;
	}
	if (disp_on) {
		mc_display_off();
	}
}

/* ========== Early blanking ==========
 * OLED controllers (SSD1306, SH1106) turn the display ON during driver init,
 * showing stale VRAM from before reset.  Our mc_display_init() runs much later
 * (after BLE, LoRa, etc.), so there's a visible garbage flash.
 *
 * Fix: SYS_INIT hook runs right after the driver, sending "Display OFF" before
 * main() starts.  This is harmless for non-OLED displays (blanking is a no-op
 * or already blanked). */
static int display_early_blank(void)
{
	const struct device *dev = NULL;

	/* Try standard chosen node first */
#if DT_HAS_CHOSEN(zephyr_display)
	dev = DEVICE_DT_GET_OR_NULL(DT_CHOSEN(zephyr_display));
#endif
	/* Legacy nodelabel fallback */
	if (!dev) {
		dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(sh1106));
	}
	if (!dev) {
		dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(ssd1306));
	}
	if (dev && device_is_ready(dev)) {
		display_blanking_on(dev);
	}
	return 0;
}
SYS_INIT(display_early_blank, APPLICATION, 99);

/* ========== Public API ========== */

int mc_display_init(void)
{
	/* Find display device from devicetree.
	 * Priority: zephyr,display chosen > sh1106 nodelabel > ssd1306 nodelabel.
	 * This supports any Zephyr display driver (SSD1306, SH1106, ST7735,
	 * ILI9341, SSD1681 e-ink, etc.) via the standard chosen mechanism. */
#if DT_HAS_CHOSEN(zephyr_display)
	disp_dev = DEVICE_DT_GET_OR_NULL(DT_CHOSEN(zephyr_display));
#endif
	if (!disp_dev) {
		disp_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(sh1106));
	}
	if (!disp_dev) {
		disp_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(ssd1306));
	}

	if (!disp_dev || !device_is_ready(disp_dev)) {
		LOG_INF("no display found - display disabled");
		return -ENODEV;
	}

	/* Query actual resolution from display driver */
	struct display_capabilities caps;

	display_get_capabilities(disp_dev, &caps);
	disp_width = caps.x_resolution;
	disp_height = caps.y_resolution;
	is_epd = (caps.screen_info & SCREEN_INFO_EPD) != 0;

	LOG_INF("display: %ux%u%s", disp_width, disp_height,
		is_epd ? " (e-paper)" : "");

	/* Blank display before CFB init to hide stale VRAM */
	display_blanking_on(disp_dev);

	/* Initialize CFB */
	int ret = cfb_framebuffer_init(disp_dev);

	if (ret) {
		LOG_ERR("CFB init failed: %d", ret);
		return ret;
	}

	/* Select smallest font — scan all registered fonts and pick the one
	 * with the smallest height for best text density.  Our custom 6x8
	 * Latin-1 font will typically win. */
	int num_fonts = cfb_get_numof_fonts(disp_dev);

	LOG_INF("display: %d fonts available", num_fonts);

	int best_idx = 0;
	uint8_t best_h = 255;

	for (int i = 0; i < num_fonts; i++) {
		uint8_t fw = 0, fh = 0;

		cfb_get_font_size(disp_dev, i, &fw, &fh);
		LOG_INF("  font[%d]: %ux%u", i, fw, fh);
		if (fh < best_h) {
			best_h = fh;
			best_idx = i;
		}
	}

	cfb_framebuffer_set_font(disp_dev, best_idx);
	cfb_get_font_size(disp_dev, best_idx, &font_w, &font_h);
	LOG_INF("display: selected font[%d] (%ux%u)", best_idx, font_w, font_h);

	/* CFB inversion no longer needed — Zephyr commit 2374ef62f97 fixed
	 * the MONO10/MONO01 polarity logic in cfb_framebuffer_finalize().
	 * SSD1306 OLED reports MONO01 by default, which CFB now handles
	 * correctly (white pixels on black background) without manual invert. */

	/* Push a blank frame to clear stale VRAM, then unblank. */
	cfb_framebuffer_clear(disp_dev, false);
	cfb_framebuffer_finalize(disp_dev);

	display_blanking_off(disp_dev);
	disp_on = true;
	disp_initialized = true;

	/* Set up auto-off timer */
	k_work_init_delayable(&auto_off_work, auto_off_handler);

	LOG_INF("display initialized (%ux%u, font %ux%u)",
		disp_width, disp_height, font_w, font_h);
	return 0;
}

uint16_t mc_display_width(void)
{
	return disp_width;
}

uint16_t mc_display_height(void)
{
	return disp_height;
}

uint8_t mc_display_font_width(void)
{
	return font_w;
}

uint8_t mc_display_font_height(void)
{
	return font_h;
}

void mc_display_on(void)
{
	if (!disp_initialized) {
		return;
	}

	if (!disp_on) {
		display_blanking_off(disp_dev);
		disp_on = true;
	}

	mc_display_reset_auto_off();
}

void mc_display_off(void)
{
	if (!disp_initialized) {
		return;
	}

	if (disp_on) {
		display_blanking_on(disp_dev);
		disp_on = false;
	}
}

bool mc_display_is_on(void)
{
	return disp_on;
}

void mc_display_clear(void)
{
	if (!disp_initialized) {
		return;
	}

	cfb_framebuffer_clear(disp_dev, false);
}

void mc_display_text(int x, int y, const char *text, bool invert)
{
	if (!disp_initialized || !text) {
		return;
	}

	if (invert) {
		cfb_framebuffer_invert(disp_dev);
	}

	cfb_print(disp_dev, text, x, y);

	if (invert) {
		cfb_framebuffer_invert(disp_dev);
	}
}

void mc_display_fill_rect(int x, int y, int w, int h)
{
	if (!disp_initialized) {
		return;
	}

	/* CFB doesn't have a native fill_rect, so we draw line by line */
	for (int row = y; row < y + h && row < disp_height; row++) {
		struct cfb_position start = { .x = x, .y = row };
		struct cfb_position end = { .x = x + w - 1, .y = row };
		cfb_draw_line(disp_dev, &start, &end);
	}
}

void mc_display_hline(int x, int y, int w)
{
	if (!disp_initialized) {
		return;
	}

	struct cfb_position start = { .x = x, .y = y };
	struct cfb_position end = { .x = x + w - 1, .y = y };
	cfb_draw_line(disp_dev, &start, &end);
}

void mc_display_xbm(int x, int y, const uint8_t *data, int w, int h)
{
	if (!disp_initialized || !data) {
		return;
	}

	/* Adafruit drawBitmap format (MSB first): row-major, bit 7 = leftmost.
	 * This matches the Arduino MeshCore logo data from icons.h.
	 * Each row is padded to byte boundary: bytes_per_row = (w+7)/8 */
	int bytes_per_row = (w + 7) / 8;

	for (int row = 0; row < h; row++) {
		for (int col = 0; col < w; col++) {
			int byte_idx = row * bytes_per_row + col / 8;
			int bit_idx = 7 - (col % 8);  /* MSB first */

			if (data[byte_idx] & (1 << bit_idx)) {
				struct cfb_position pos = {
					.x = (int16_t)(x + col),
					.y = (int16_t)(y + row)
				};
				cfb_draw_point(disp_dev, &pos);
			}
		}
	}
}

void mc_display_finalize(void)
{
	if (!disp_initialized) {
		return;
	}

	/* Don't let CFB overwrite display while Doom is rendering directly */
	if (doom_game_is_running()) {
		return;
	}

	cfb_framebuffer_finalize(disp_dev);
}

void mc_display_reset_auto_off(void)
{
	if (!disp_initialized) {
		return;
	}

#ifdef CONFIG_ZEPHCORE_UI_DISPLAY_AUTO_OFF_MS
	uint32_t timeout = CONFIG_ZEPHCORE_UI_DISPLAY_AUTO_OFF_MS;

	if (timeout > 0) {
		k_work_reschedule(&auto_off_work, K_MSEC(timeout));
	}
#endif
}

const struct device *mc_display_get_device(void)
{
	return disp_initialized ? disp_dev : NULL;
}

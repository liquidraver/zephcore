/*
 * SPDX-License-Identifier: Apache-2.0
 * Minimal LR11xx HAL for firmware updater.
 *
 * Implements the Semtech lr11xx_hal_* interface used by lr1110_bootloader.c.
 * Stripped down: no DIO1 interrupt, no work queue, no sleep tracking.
 * Just SPI + GPIO for bootloader commands.
 */

#include "lr11xx_hal_updater.h"
#include "lr11xx_hal.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lr1110_hal, LOG_LEVEL_INF);

/* ── Hardware from devicetree ──────────────────────────────── */

#define LR1110_NODE DT_NODELABEL(lora)

#if !DT_NODE_EXISTS(LR1110_NODE)
#error "No 'lora' node found in devicetree — is this an LR1110 board?"
#endif

/* SPI bus device */
static const struct device *spi_dev = DEVICE_DT_GET(DT_BUS(LR1110_NODE));

/* SPI config — manual CS (we toggle NSS via GPIO) */
static struct spi_config spi_cfg = {
	.frequency = DT_PROP(LR1110_NODE, spi_max_frequency),
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
};

/* GPIO pins */
static const struct gpio_dt_spec pin_nss   = GPIO_DT_SPEC_GET(DT_BUS(LR1110_NODE), cs_gpios);
static const struct gpio_dt_spec pin_reset = GPIO_DT_SPEC_GET(LR1110_NODE, reset_gpios);
static const struct gpio_dt_spec pin_busy  = GPIO_DT_SPEC_GET(LR1110_NODE, busy_gpios);

/* BUSY timeout — 3 seconds (flash erase can take ~2.5s) */
#define BUSY_TIMEOUT_MS 3000

/* Extended BUSY timeout for flash erase */
#define ERASE_BUSY_TIMEOUT_MS 5000

/* ── Context (opaque pointer for Semtech driver) ──────────── */

/* The Semtech driver passes 'context' to every HAL function.
 * We use a dummy static — all state is in file-scope globals. */
static int dummy_context;

void *lr1110_updater_get_context(void)
{
	return &dummy_context;
}

/* ── BUSY wait ────────────────────────────────────────────── */

static int wait_on_busy(uint32_t timeout_ms)
{
	int64_t start = k_uptime_get();

	while (gpio_pin_get_dt(&pin_busy)) {
		if ((k_uptime_get() - start) > timeout_ms) {
			printk("ERROR: BUSY timeout after %u ms\n", timeout_ms);
			return -ETIMEDOUT;
		}
		k_busy_wait(100); /* 100us */
	}
	return 0;
}

/* ── Public init/reset ────────────────────────────────────── */

int lr1110_updater_hal_init(void)
{
	int ret;

	if (!device_is_ready(spi_dev)) {
		printk("ERROR: SPI device not ready\n");
		return -ENODEV;
	}

	/* NSS — output, inactive (HIGH = deselected) */
	ret = gpio_pin_configure_dt(&pin_nss, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		printk("ERROR: NSS config failed: %d\n", ret);
		return ret;
	}

	/* RESET — output, inactive (HIGH = not in reset) */
	ret = gpio_pin_configure_dt(&pin_reset, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		printk("ERROR: RESET config failed: %d\n", ret);
		return ret;
	}

	/* BUSY — input */
	ret = gpio_pin_configure_dt(&pin_busy, GPIO_INPUT);
	if (ret < 0) {
		printk("ERROR: BUSY config failed: %d\n", ret);
		return ret;
	}

	printk("LR1110 HAL initialized (SPI @ %u Hz)\n", spi_cfg.frequency);
	return 0;
}

int lr1110_updater_hw_reset(void)
{
	printk("Resetting LR1110...\n");

	/* Assert reset (active-low: logical 1 = physical LOW = reset active) */
	gpio_pin_set_dt(&pin_reset, 1);
	k_msleep(10);

	/* Release reset */
	gpio_pin_set_dt(&pin_reset, 0);

	/* After reset, the LR1110 boots into bootloader if flash is empty,
	 * or into firmware if flash has valid content.
	 * Firmware boot takes up to 273ms (datasheet). */
	k_msleep(300);

	int ret = wait_on_busy(BUSY_TIMEOUT_MS);
	if (ret) {
		printk("ERROR: BUSY stuck after reset\n");
	} else {
		printk("LR1110 reset complete, BUSY=low\n");
	}
	return ret;
}

int lr1110_updater_reset_to_bootloader(void)
{
	printk("Resetting LR1110 into bootloader (BUSY held LOW)...\n");

	/* Semtech lr1110_updater_tool pattern:
	 * 1. Drive BUSY LOW as output during reset
	 * 2. Pulse RESET
	 * 3. Wait 500ms
	 * 4. Release BUSY back to input
	 * 5. Wait 100ms + BUSY low
	 *
	 * When BUSY is held LOW by the host during reset, the LR1110
	 * enters bootloader mode instead of executing flash firmware. */

	/* Drive BUSY to physical LOW (pin_busy has GPIO_ACTIVE_HIGH,
	 * so we use raw GPIO to be explicit about physical level) */
	gpio_pin_configure(pin_busy.port, pin_busy.pin,
			   GPIO_OUTPUT_LOW);

	/* Assert reset */
	gpio_pin_set_dt(&pin_reset, 1);
	k_msleep(10);

	/* Release reset — chip starts booting, sees BUSY held LOW → bootloader */
	gpio_pin_set_dt(&pin_reset, 0);
	k_msleep(500);

	/* Release BUSY back to input */
	gpio_pin_configure_dt(&pin_busy, GPIO_INPUT);
	k_msleep(100);

	int ret = wait_on_busy(BUSY_TIMEOUT_MS);
	if (ret) {
		printk("ERROR: BUSY stuck after bootloader reset\n");
	} else {
		printk("LR1110 in bootloader mode, BUSY=low\n");
	}
	return ret;
}

/* ── Semtech HAL interface ────────────────────────────────── */

lr11xx_hal_status_t lr11xx_hal_write(const void *context, const uint8_t *command,
				     const uint16_t command_length,
				     const uint8_t *data, const uint16_t data_length)
{
	(void)context;
	int ret;

	/* Wait for device ready */
	if (wait_on_busy(BUSY_TIMEOUT_MS)) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	const struct spi_buf tx_bufs[] = {
		{ .buf = (uint8_t *)command, .len = command_length },
		{ .buf = (uint8_t *)data, .len = data_length },
	};
	const struct spi_buf_set tx = {
		.buffers = tx_bufs,
		.count = (data_length > 0) ? 2 : 1,
	};

	gpio_pin_set_dt(&pin_nss, 1); /* Assert NSS (LOW) */
	ret = spi_write(spi_dev, &spi_cfg, &tx);
	gpio_pin_set_dt(&pin_nss, 0); /* Deassert NSS (HIGH) */

	if (ret < 0) {
		printk("ERROR: SPI write failed: %d\n", ret);
		return LR11XX_HAL_STATUS_ERROR;
	}

	/* For flash erase command (0x8000), BUSY can stay high for ~2.5 seconds */
	uint16_t opcode = 0;
	if (command_length >= 2) {
		opcode = ((uint16_t)command[0] << 8) | command[1];
	}
	uint32_t timeout = (opcode == 0x8000) ? ERASE_BUSY_TIMEOUT_MS : BUSY_TIMEOUT_MS;

	if (wait_on_busy(timeout)) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_read(const void *context, const uint8_t *command,
				    const uint16_t command_length,
				    uint8_t *data, const uint16_t data_length)
{
	(void)context;
	int ret;

	/* Wait for device ready */
	if (wait_on_busy(BUSY_TIMEOUT_MS)) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	/* Step 1: Write command */
	const struct spi_buf tx_buf = { .buf = (uint8_t *)command, .len = command_length };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

	gpio_pin_set_dt(&pin_nss, 1);
	ret = spi_write(spi_dev, &spi_cfg, &tx);
	gpio_pin_set_dt(&pin_nss, 0);

	if (ret < 0) {
		printk("ERROR: SPI write (cmd) failed: %d\n", ret);
		return LR11XX_HAL_STATUS_ERROR;
	}

	if (data_length == 0) {
		return (wait_on_busy(BUSY_TIMEOUT_MS) == 0)
			? LR11XX_HAL_STATUS_OK : LR11XX_HAL_STATUS_ERROR;
	}

	/* Step 2: Wait for device ready, then read response */
	if (wait_on_busy(BUSY_TIMEOUT_MS)) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	/* LR11xx returns 1 dummy byte + data */
	uint8_t dummy;
	const struct spi_buf rx_bufs[] = {
		{ .buf = &dummy, .len = 1 },
		{ .buf = data, .len = data_length },
	};
	const struct spi_buf_set rx = { .buffers = rx_bufs, .count = 2 };

	gpio_pin_set_dt(&pin_nss, 1);
	ret = spi_read(spi_dev, &spi_cfg, &rx);
	gpio_pin_set_dt(&pin_nss, 0);

	if (ret < 0) {
		printk("ERROR: SPI read failed: %d\n", ret);
		return LR11XX_HAL_STATUS_ERROR;
	}

	return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_direct_read(const void *context, uint8_t *data,
					   const uint16_t data_length)
{
	(void)context;
	int ret;

	if (wait_on_busy(BUSY_TIMEOUT_MS)) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	const struct spi_buf rx_buf = { .buf = data, .len = data_length };
	const struct spi_buf_set rx = { .buffers = &rx_buf, .count = 1 };

	gpio_pin_set_dt(&pin_nss, 1);
	ret = spi_read(spi_dev, &spi_cfg, &rx);
	gpio_pin_set_dt(&pin_nss, 0);

	if (ret < 0) {
		printk("ERROR: SPI direct read failed: %d\n", ret);
		return LR11XX_HAL_STATUS_ERROR;
	}

	return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_reset(const void *context)
{
	(void)context;
	return (lr1110_updater_hw_reset() == 0)
		? LR11XX_HAL_STATUS_OK : LR11XX_HAL_STATUS_ERROR;
}

lr11xx_hal_status_t lr11xx_hal_wakeup(const void *context)
{
	(void)context;
	return (wait_on_busy(BUSY_TIMEOUT_MS) == 0)
		? LR11XX_HAL_STATUS_OK : LR11XX_HAL_STATUS_ERROR;
}

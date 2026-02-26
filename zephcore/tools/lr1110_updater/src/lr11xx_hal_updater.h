/*
 * SPDX-License-Identifier: Apache-2.0
 * Minimal LR11xx HAL for firmware updater — no IRQ, no DIO1, just SPI+GPIO.
 *
 * Provides the lr11xx_hal_write/read/direct_read/reset interface that the
 * Semtech bootloader driver (lr1110_bootloader.c) needs.
 */

#ifndef LR11XX_HAL_UPDATER_H
#define LR11XX_HAL_UPDATER_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

/**
 * @brief Initialize SPI and GPIOs for LR1110 communication.
 *
 * Reads pin configuration from the devicetree "semtech,lr1110" node on SPI1.
 *
 * @return 0 on success, negative errno on failure
 */
int lr1110_updater_hal_init(void);

/**
 * @brief Hardware reset the LR1110 (pulse RESET, wait for BUSY low).
 *
 * After reset the LR1110 boots into firmware (if flash valid) or bootloader.
 *
 * @return 0 on success, negative errno on failure
 */
int lr1110_updater_hw_reset(void);

/**
 * @brief Force LR1110 into bootloader mode via hardware reset.
 *
 * Holds BUSY LOW as output during RESET pulse — this forces the LR1110
 * into bootloader mode regardless of flash content. This is the official
 * Semtech approach (lr1110_updater_tool).
 *
 * @return 0 on success, negative errno on failure
 */
int lr1110_updater_reset_to_bootloader(void);

/**
 * @brief Get the opaque HAL context pointer for Semtech driver calls.
 *
 * This pointer is passed as 'context' to lr1110_bootloader_*() functions.
 */
void *lr1110_updater_get_context(void);

#endif /* LR11XX_HAL_UPDATER_H */

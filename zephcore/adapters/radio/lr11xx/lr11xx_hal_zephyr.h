/*
 * SPDX-License-Identifier: Apache-2.0
 * LR11xx HAL implementation for Zephyr - ZephCore
 *
 * Simplified HAL that uses direct SPI/GPIO access without Zephyr device model.
 * Based on IRNAS/SWDR001-Zephyr but adapted for ZephCore's simpler needs.
 */

#ifndef LR11XX_HAL_ZEPHYR_H
#define LR11XX_HAL_ZEPHYR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include "lr11xx_hal.h"

/**
 * @brief LR11xx HAL context for Zephyr
 *
 * This structure is passed as the 'context' parameter to all lr11xx_hal_* functions.
 * It contains all the hardware configuration needed to communicate with the radio.
 *
 * CRITICAL: All SPI operations are protected by spi_mutex. The LR1110 radio is
 * accessed from two threads:
 *   1. Main thread: mesh event loop (noise floor calibration, TX, reconfigure)
 *   2. System work queue: DIO1 interrupt handler (RX packet processing)
 * Without the mutex, concurrent SPI access corrupts the LR1110 command/response
 * protocol, causing the BUSY pin to get stuck HIGH permanently.
 */
struct lr11xx_hal_context {
    /* SPI device */
    const struct device *spi_dev;
    struct spi_config spi_cfg;

    /* GPIO pins */
    struct gpio_dt_spec nss;    /* Chip select (directly controlled, not via SPI CS) */
    struct gpio_dt_spec reset;  /* Reset pin (active low) */
    struct gpio_dt_spec busy;   /* Busy pin (high = busy) */
    struct gpio_dt_spec dio1;   /* DIO1 interrupt pin */

    /* Optional pins */
    struct gpio_dt_spec dio2;   /* DIO2 (often RF switch control) */
    struct gpio_dt_spec rxen;   /* RX enable (for external PA/LNA) */
    struct gpio_dt_spec txen;   /* TX enable (for external PA/LNA) */

    /* TCXO config */
    uint16_t tcxo_voltage_mv;   /* 0 = no TCXO, else voltage in mV (e.g., 1600 = 1.6V) */
    uint32_t tcxo_startup_us;   /* TCXO startup time in microseconds */

    /* State tracking */
    volatile bool radio_is_sleeping;
};

/**
 * @brief Initialize the HAL context GPIOs
 *
 * Must be called before any other HAL functions.
 *
 * @param ctx HAL context with gpio specs already filled in
 * @return 0 on success, negative errno on failure
 */
int lr11xx_hal_init(struct lr11xx_hal_context *ctx);

/**
 * @brief GPIO callback type for DIO1 interrupt
 */
typedef void (*lr11xx_dio1_callback_t)(void *user_data);

/**
 * @brief Set DIO1 interrupt callback
 *
 * @param ctx HAL context
 * @param cb Callback function (called from ISR context)
 * @param user_data User data passed to callback
 */
void lr11xx_hal_set_dio1_callback(struct lr11xx_hal_context *ctx,
                                   lr11xx_dio1_callback_t cb, void *user_data);

/**
 * @brief Enable DIO1 interrupt
 */
void lr11xx_hal_enable_dio1_irq(struct lr11xx_hal_context *ctx);

/**
 * @brief Disable DIO1 interrupt
 */
void lr11xx_hal_disable_dio1_irq(struct lr11xx_hal_context *ctx);

#ifdef __cplusplus
}
#endif

#endif /* LR11XX_HAL_ZEPHYR_H */

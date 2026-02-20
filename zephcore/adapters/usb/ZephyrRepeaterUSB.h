/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore - USB CDC ACM with 1200 baud touch detection (Repeater)
 *
 * Replaces Zephyr's CDC_ACM_SERIAL_INITIALIZE_AT_BOOT with custom init
 * that includes a message callback for detecting 1200 baud touch to
 * enter UF2 bootloader mode.
 */

#ifndef ZEPHCORE_ZEPHYR_REPEATER_USB_H
#define ZEPHCORE_ZEPHYR_REPEATER_USB_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize USB CDC ACM with 1200 baud touch detection.
 *
 * This replaces Zephyr's default CDC_ACM_SERIAL_INITIALIZE_AT_BOOT
 * initialization with our own that includes a message callback for
 * detecting 1200 baud touch to enter bootloader mode.
 *
 * @return 0 on success, negative error code on failure.
 */
int zephcore_usbd_init(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHCORE_ZEPHYR_REPEATER_USB_H */

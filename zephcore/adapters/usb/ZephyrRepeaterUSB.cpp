/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore - USB CDC ACM with 1200 baud touch detection (Repeater)
 *
 * This module initializes USB CDC ACM with a message callback to detect
 * 1200 baud touch for entering UF2 bootloader mode.
 */

#include <stdint.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/hwinfo.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_usb_repeater, CONFIG_ZEPHCORE_BOARD_LOG_LEVEL);

#include <adapters/board/ZephyrBoard.h>

#include "ZephyrRepeaterUSB.h"

/*
 * USB Device Configuration
 * Since we're not using CDC_ACM_SERIAL_INITIALIZE_AT_BOOT, we define our own values.
 */
#define ZEPHCORE_USB_VID        0x2fe3  /* Zephyr Project VID */
#define ZEPHCORE_USB_PID        0x0004  /* CDC ACM PID */
#define ZEPHCORE_USB_MAX_POWER  125     /* 250mA in 2mA units */

/*
 * USB Device Context - based on Zephyr's cdc_acm_serial.c
 * but with message callback for 1200 baud touch detection.
 */
USBD_DEVICE_DEFINE(zephcore_usbd,
		   DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
		   ZEPHCORE_USB_VID, ZEPHCORE_USB_PID);

USBD_DESC_LANG_DEFINE(zephcore_lang);
USBD_DESC_MANUFACTURER_DEFINE(zephcore_mfr, "ZephCore");
USBD_DESC_PRODUCT_DEFINE(zephcore_product, "ZephCore Repeater");
USBD_DESC_SERIAL_NUMBER_DEFINE(zephcore_sn);

USBD_DESC_CONFIG_DEFINE(zephcore_fs_cfg_desc, "FS Configuration");
USBD_DESC_CONFIG_DEFINE(zephcore_hs_cfg_desc, "HS Configuration");

USBD_CONFIGURATION_DEFINE(zephcore_fs_config,
			  0, /* attributes: bus-powered */
			  ZEPHCORE_USB_MAX_POWER, &zephcore_fs_cfg_desc);

USBD_CONFIGURATION_DEFINE(zephcore_hs_config,
			  0, /* attributes: bus-powered */
			  ZEPHCORE_USB_MAX_POWER, &zephcore_hs_cfg_desc);

/* ZephyrBoard instance for bootloader entry */
static mesh::ZephyrBoard usb_board;

/*
 * USB message callback - detect 1200 baud touch for bootloader entry.
 * Arduino uses this method: when host opens port at 1200 baud, enter DFU.
 */
static void usbd_msg_callback(struct usbd_context *const ctx, const struct usbd_msg *msg)
{
	if (msg->type == USBD_MSG_CDC_ACM_LINE_CODING) {
		uint32_t baudrate = 0;
		int ret = uart_line_ctrl_get(msg->dev, UART_LINE_CTRL_BAUD_RATE, &baudrate);
		if (ret == 0) {
			LOG_DBG("CDC ACM baud rate change: %u", baudrate);
			if (baudrate == 1200) {
				LOG_INF("1200 baud touch detected - entering bootloader");
				/* Small delay to let USB disconnect cleanly */
				k_msleep(100);
				usb_board.rebootToBootloader();
				/* Never returns */
			}
		}
	}
}

static int register_cdc_acm(struct usbd_context *const uds_ctx,
			    const enum usbd_speed speed)
{
	struct usbd_config_node *cfg_nd;
	int err;

	if (speed == USBD_SPEED_HS) {
		cfg_nd = &zephcore_hs_config;
	} else {
		cfg_nd = &zephcore_fs_config;
	}

	err = usbd_add_configuration(uds_ctx, speed, cfg_nd);
	if (err) {
		LOG_ERR("Failed to add configuration");
		return err;
	}

	err = usbd_register_class(&zephcore_usbd, "cdc_acm_0", speed, 1);
	if (err) {
		LOG_ERR("Failed to register CDC ACM class");
		return err;
	}

	return usbd_device_set_code_triple(uds_ctx, speed,
					   USB_BCC_MISCELLANEOUS, 0x02, 0x01);
}

extern "C" int zephcore_usbd_init(void)
{
	int err;

	LOG_INF("Initializing ZephCore USB CDC with 1200 baud touch detection");

	/* Add string descriptors */
	err = usbd_add_descriptor(&zephcore_usbd, &zephcore_lang);
	if (err) {
		LOG_ERR("Failed to add language descriptor (%d)", err);
		return err;
	}

	err = usbd_add_descriptor(&zephcore_usbd, &zephcore_mfr);
	if (err) {
		LOG_ERR("Failed to add manufacturer descriptor (%d)", err);
		return err;
	}

	err = usbd_add_descriptor(&zephcore_usbd, &zephcore_product);
	if (err) {
		LOG_ERR("Failed to add product descriptor (%d)", err);
		return err;
	}

	err = usbd_add_descriptor(&zephcore_usbd, &zephcore_sn);
	if (err) {
		LOG_ERR("Failed to add serial number descriptor (%d)", err);
		return err;
	}

	/* Register configurations */
	if (USBD_SUPPORTS_HIGH_SPEED &&
	    usbd_caps_speed(&zephcore_usbd) == USBD_SPEED_HS) {
		err = register_cdc_acm(&zephcore_usbd, USBD_SPEED_HS);
		if (err) {
			return err;
		}
	}

	err = register_cdc_acm(&zephcore_usbd, USBD_SPEED_FS);
	if (err) {
		return err;
	}

	/* Register message callback for 1200 baud detection */
	err = usbd_msg_register_cb(&zephcore_usbd, usbd_msg_callback);
	if (err) {
		LOG_ERR("Failed to register message callback (%d)", err);
		return err;
	}

	/* Initialize USB device */
	err = usbd_init(&zephcore_usbd);
	if (err) {
		LOG_ERR("Failed to initialize USB device (%d)", err);
		return err;
	}

	/* Enable USB device */
	err = usbd_enable(&zephcore_usbd);
	if (err) {
		LOG_ERR("Failed to enable USB device (%d)", err);
		return err;
	}

	LOG_INF("ZephCore USB CDC initialized");
	return 0;
}

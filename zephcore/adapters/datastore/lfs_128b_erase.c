/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Custom LittleFS erase callback for 128-byte blocks on 4KB-erase flash.
 *
 * Arduino MeshCore uses LFS with block_size=128 on nRF52840 which has
 * 4096-byte erase granularity.  The standard Zephyr erase callback calls
 * flash_area_flatten(fa, offset, 128) which fails because the flash driver
 * requires page-aligned (4096-byte) erases.
 *
 * This callback implements read-modify-erase-write:
 *   1. Read the entire 4KB page containing the 128B block
 *   2. Memset the 128B region to 0xFF (erased state)
 *   3. Hardware-erase the 4KB page
 *   4. Write back the modified page
 */

#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/flash.h>
#include <lfs.h>
#include <string.h>

#define HW_ERASE_SIZE 4096

/* Static 4KB buffer for read-modify-erase-write.
 * Both ExtraFS and InternalFS mounts are serialized by the Zephyr FS mutex,
 * so this is never accessed concurrently. */
static uint8_t page_buf[HW_ERASE_SIZE];

int lfs_128b_erase(const struct lfs_config *c, lfs_block_t block)
{
	const struct flash_area *fa = (const struct flash_area *)c->context;
	int rc;

	/* Offset of this LFS block within the flash area */
	size_t block_offset = block * c->block_size;

	/* Align down to the 4KB page boundary (within flash area) */
	size_t page_offset = block_offset & ~((size_t)HW_ERASE_SIZE - 1);
	size_t offset_in_page = block_offset - page_offset;

	/* 1. Read entire 4KB page */
	rc = flash_area_read(fa, page_offset, page_buf, HW_ERASE_SIZE);
	if (rc < 0) {
		return LFS_ERR_IO;
	}

	/* 2. Set the 128B block region to erased state (0xFF) */
	memset(&page_buf[offset_in_page], 0xFF, c->block_size);

	/* 3. Hardware-erase the 4KB page */
	rc = flash_area_flatten(fa, page_offset, HW_ERASE_SIZE);
	if (rc < 0) {
		return LFS_ERR_IO;
	}

	/* 4. Write back the modified page */
	rc = flash_area_write(fa, page_offset, page_buf, HW_ERASE_SIZE);
	if (rc < 0) {
		return LFS_ERR_IO;
	}

	return LFS_ERR_OK;
}

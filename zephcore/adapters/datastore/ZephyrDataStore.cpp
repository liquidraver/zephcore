/*
 * SPDX-License-Identifier: Apache-2.0
 * Zephyr DataStore - LittleFS-backed persistence with optional QSPI flash
 */

#include "ZephyrDataStore.h"
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/device.h>
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_datastore, CONFIG_ZEPHCORE_DATASTORE_LOG_LEVEL);

#define MAX_ADVERT_PKT_LEN (2 + 32 + PUB_KEY_SIZE + 4 + SIGNATURE_SIZE + MAX_ADVERT_DATA_SIZE)

struct BlobRec {
	uint32_t timestamp;
	uint8_t key[7];
	uint8_t len;
	uint8_t data[MAX_ADVERT_PKT_LEN];
};

/* Track mount status - filesystems are automounted via DTS fstab */
static bool lfs_mounted;
static bool ext_lfs_mounted;

/* Check if a filesystem is mounted using fs_statvfs */
static bool is_mounted(const char *mount_point)
{
	struct fs_statvfs stat;
	return fs_statvfs(mount_point, &stat) == 0;
}

bool ZephyrDataStore::mount()
{
	if (lfs_mounted) {
		return true;
	}

	/* Check if internal LFS was automounted */
	if (is_mounted(mountPoint())) {
		lfs_mounted = true;
		LOG_INF("Internal LittleFS at %s (automounted)", mountPoint());
	} else {
		LOG_ERR("Internal LittleFS NOT mounted at %s - check DTS fstab!", mountPoint());
		return false;
	}

	/* Check if external QSPI was automounted */
	if (is_mounted(extMountPoint())) {
		ext_lfs_mounted = true;
		LOG_INF("External QSPI LittleFS at %s (automounted, 100 blobs)", extMountPoint());
	} else {
		ext_lfs_mounted = false;
		LOG_WRN("External QSPI NOT mounted at %s - using internal only (20 blobs)", extMountPoint());
	}

	return true;
}

void ZephyrDataStore::unmount()
{
	/* With automount, filesystems are managed by Zephyr - just clear our flags */
	lfs_mounted = false;
	ext_lfs_mounted = false;
}

ZephyrDataStore::ZephyrDataStore(mesh::RTCClock &clock)
	: _clock(&clock), _has_ext_fs(false)
{
}

void ZephyrDataStore::begin()
{
	_has_ext_fs = ext_lfs_mounted;
	LOG_INF("_has_ext_fs=%d (ext_lfs_mounted=%d)", _has_ext_fs ? 1 : 0, ext_lfs_mounted ? 1 : 0);
	LOG_INF("contacts path=%s, channels path=%s", contactsFile(), channelsFile());

	if (_has_ext_fs) {
		migrateToExternalFS();
	}

	/* Clean up stale .tmp files from interrupted saves.
	 * If a .tmp file exists, the save was interrupted before the
	 * atomic rename — the original file is still intact. */
	cleanStaleTmpFiles();

	checkAdvBlobFile();
}

void ZephyrDataStore::cleanStaleTmpFiles()
{
	/* If a .tmp file exists, a save was interrupted before the atomic
	 * rename completed.  Since we never unlink the original before
	 * rename, the original file is guaranteed intact.  The .tmp may
	 * be partial (crash during write) or complete (crash between
	 * close and rename).  Either way, delete it — we keep the last
	 * successfully-committed version.  Worst case: we lose the one
	 * save that was in progress, but never lose ALL data. */
	const char *paths[] = { contactsFile(), channelsFile(), advBlobsFile() };
	char tmp_path[48];
	for (size_t i = 0; i < ARRAY_SIZE(paths); i++) {
		snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", paths[i]);
		struct fs_dirent ent;
		if (fs_stat(tmp_path, &ent) == 0) {
			LOG_INF("PREVIOUS REBOOT CORRUPTED FS! "
				"Deleting temp file: %s (%zu bytes)",
				tmp_path, ent.size);
			fs_unlink(tmp_path);
		}
	}
}

bool ZephyrDataStore::exists(const char *path)
{
	struct fs_dirent ent;
	return fs_stat(path, &ent) == 0;
}

bool ZephyrDataStore::removeFile(const char *path)
{
	return fs_unlink(path) == 0;
}

bool ZephyrDataStore::openRead(const char *path, uint8_t *buf, size_t buf_sz, size_t &out_len)
{
	struct fs_file_t file;
	fs_file_t_init(&file);
	int rc = fs_open(&file, path, FS_O_READ);
	if (rc < 0) {
		return false;
	}
	ssize_t n = fs_read(&file, buf, buf_sz);
	fs_close(&file);
	if (n < 0) {
		return false;
	}
	out_len = (size_t)n;
	return true;
}

bool ZephyrDataStore::openWrite(const char *path, const uint8_t *buf, size_t len)
{
	fs_unlink(path);

	struct fs_file_t file;
	fs_file_t_init(&file);
	int rc = fs_open(&file, path, FS_O_CREATE | FS_O_WRITE);
	if (rc < 0) {
		LOG_ERR("openWrite: fs_open(%s) failed: %d", path, rc);
		return false;
	}
	ssize_t n = fs_write(&file, buf, len);
	fs_close(&file);
	return n >= 0 && (size_t)n == len;
}

bool ZephyrDataStore::copyFile(const char *src, const char *dst)
{
	struct fs_file_t src_file, dst_file;
	fs_file_t_init(&src_file);
	fs_file_t_init(&dst_file);

	if (fs_open(&src_file, src, FS_O_READ) < 0) {
		return false;
	}

	fs_unlink(dst);
	if (fs_open(&dst_file, dst, FS_O_CREATE | FS_O_WRITE) < 0) {
		fs_close(&src_file);
		return false;
	}

	uint8_t buf[64];
	ssize_t n;
	bool ok = true;
	while ((n = fs_read(&src_file, buf, sizeof(buf))) > 0) {
		if (fs_write(&dst_file, buf, n) != n) {
			ok = false;
			break;
		}
	}

	fs_close(&src_file);
	fs_close(&dst_file);
	return ok && n >= 0;
}

void ZephyrDataStore::migrateToExternalFS()
{
	/* Migrate contacts from internal to external if not present */
	if (!exists(EXT_CONTACTS_FILE) && exists(INT_CONTACTS_FILE)) {
		LOG_INF("Migrating contacts to external storage");
		if (copyFile(INT_CONTACTS_FILE, EXT_CONTACTS_FILE)) {
			removeFile(INT_CONTACTS_FILE);
		}
	}

	/* Migrate channels from internal to external if not present */
	if (!exists(EXT_CHANNELS_FILE) && exists(INT_CHANNELS_FILE)) {
		LOG_INF("Migrating channels to external storage");
		if (copyFile(INT_CHANNELS_FILE, EXT_CHANNELS_FILE)) {
			removeFile(INT_CHANNELS_FILE);
		}
	}

	/* Migrate adv_blobs - need special handling for size change */
	if (!exists(EXT_ADV_BLOBS_FILE) && exists(INT_ADV_BLOBS_FILE)) {
		LOG_INF("Migrating adv_blobs to external storage (20 -> 100 slots)");
		if (copyFile(INT_ADV_BLOBS_FILE, EXT_ADV_BLOBS_FILE)) {
			removeFile(INT_ADV_BLOBS_FILE);
			/* Extend blob file to 100 records */
			struct fs_file_t file;
			fs_file_t_init(&file);
			if (fs_open(&file, EXT_ADV_BLOBS_FILE, FS_O_RDWR) == 0) {
				fs_seek(&file, 0, FS_SEEK_END);
				BlobRec zeroes;
				memset(&zeroes, 0, sizeof(zeroes));
				for (int i = 20; i < 100; i++) {
					fs_write(&file, &zeroes, sizeof(zeroes));
				}
				fs_close(&file);
			}
		}
	}

	/* Clean up old files on internal if they exist on external */
	if (exists(EXT_CONTACTS_FILE) && exists(INT_CONTACTS_FILE)) {
		removeFile(INT_CONTACTS_FILE);
	}
	if (exists(EXT_CHANNELS_FILE) && exists(INT_CHANNELS_FILE)) {
		removeFile(INT_CHANNELS_FILE);
	}
	if (exists(EXT_ADV_BLOBS_FILE) && exists(INT_ADV_BLOBS_FILE)) {
		removeFile(INT_ADV_BLOBS_FILE);
	}
}

void ZephyrDataStore::checkAdvBlobFile()
{
	const char *path = advBlobsFile();
	if (exists(path)) {
		return;
	}
	BlobRec zeroes;
	memset(&zeroes, 0, sizeof(zeroes));
	struct fs_file_t file;
	fs_file_t_init(&file);
	int rc = fs_open(&file, path, FS_O_CREATE | FS_O_WRITE);
	if (rc < 0) {
		LOG_ERR("Failed to create adv_blobs file: %d", rc);
		return;
	}
	int count = maxBlobRecs();
	for (int i = 0; i < count; i++) {
		fs_write(&file, &zeroes, sizeof(zeroes));
	}
	fs_close(&file);
	LOG_INF("Created adv_blobs with %d slots", count);
}

bool ZephyrDataStore::formatFileSystem()
{
	LOG_INF("formatFileSystem: starting...");
	unmount();

	const struct flash_area *fap;
	int rc;

	/* Format internal LittleFS partition */
	LOG_INF("formatFileSystem: opening lfs_partition...");
	rc = flash_area_open(FIXED_PARTITION_ID(lfs_partition), &fap);
	if (rc < 0) {
		LOG_ERR("Failed to open lfs_partition: rc=%d", rc);
		return false;
	}
	LOG_INF("formatFileSystem: lfs_partition opened, size=%u, erasing...", (unsigned)fap->fa_size);
	rc = flash_area_flatten(fap, 0, fap->fa_size);
	flash_area_close(fap);
	if (rc < 0) {
		LOG_ERR("Failed to format lfs_partition: rc=%d", rc);
		return false;
	}
	LOG_INF("Formatted internal LittleFS (92KB)");

	/* Format NVS storage partition (BLE bonds) */
	LOG_INF("formatFileSystem: opening storage_partition (NVS/bonds)...");
	rc = flash_area_open(FIXED_PARTITION_ID(storage_partition), &fap);
	if (rc < 0) {
		LOG_ERR("Failed to open storage_partition: rc=%d", rc);
		return false;
	}
	LOG_INF("formatFileSystem: storage_partition opened, size=%u, erasing...", (unsigned)fap->fa_size);
	rc = flash_area_flatten(fap, 0, fap->fa_size);
	flash_area_close(fap);
	if (rc < 0) {
		LOG_ERR("Failed to format storage_partition: rc=%d", rc);
		return false;
	}
	LOG_INF("Formatted NVS storage (8KB) - BLE bonds cleared");

	/* NOTE: We intentionally do NOT erase the bootloader area (0xED000-end).
	 * Storage partition (0xD4000-0xD6000) contains BLE bonds and IS erased. */

#if FIXED_PARTITION_EXISTS(qspi_storage_partition)
	/* Format external QSPI flash if present */
	LOG_INF("formatFileSystem: opening qspi_storage_partition...");
	rc = flash_area_open(FIXED_PARTITION_ID(qspi_storage_partition), &fap);
	if (rc == 0) {
		LOG_INF("formatFileSystem: QSPI partition opened, size=%u, erasing (this may take a while)...", (unsigned)fap->fa_size);
		rc = flash_area_flatten(fap, 0, fap->fa_size);
		flash_area_close(fap);
		if (rc < 0) {
			LOG_ERR("Failed to format external QSPI storage: rc=%d", rc);
		} else {
			LOG_INF("Formatted external QSPI storage");
		}
	} else {
		LOG_WRN("formatFileSystem: couldn't open qspi_storage_partition: rc=%d (may not be present)", rc);
	}
#else
	LOG_INF("formatFileSystem: no qspi_storage_partition defined, skipping QSPI");
#endif

	LOG_INF("formatFileSystem: calling mount()...");
	bool mounted = mount();
	LOG_INF("formatFileSystem: mount() returned %d", mounted ? 1 : 0);
	return mounted;
}

bool ZephyrDataStore::loadMainIdentity(mesh::LocalIdentity &identity)
{
	uint8_t buf[PRV_KEY_SIZE + PUB_KEY_SIZE + 32];
	size_t len = 0;
	if (!openRead(MAIN_ID_FILE, buf, sizeof(buf), len) || len < PRV_KEY_SIZE + PUB_KEY_SIZE) {
		return false;
	}
	return identity.readFrom(buf, len);
}

bool ZephyrDataStore::saveMainIdentity(const mesh::LocalIdentity &identity)
{
	uint8_t buf[PRV_KEY_SIZE + PUB_KEY_SIZE + 32];
	size_t n = identity.writeTo(buf, sizeof(buf));
	if (n == 0) {
		return false;
	}
	return openWrite(MAIN_ID_FILE, buf, n);
}

void ZephyrDataStore::loadPrefs(NodePrefs &prefs)
{
	bool prefs_exists = exists(PREFS_FILE);
	LOG_INF("loadPrefs: exists(%s)=%d", PREFS_FILE, prefs_exists ? 1 : 0);
	const char *path = prefs_exists ? PREFS_FILE : "/lfs/node_prefs";
	LOG_INF("loadPrefs: trying path=%s", path);
	uint8_t buf[256];
	size_t len = 0;
	if (!openRead(path, buf, sizeof(buf), len)) {
		LOG_WRN("loadPrefs: file not found or read failed");
		return;
	}
	if (len < 88) {
		LOG_WRN("loadPrefs: file too small (%d bytes, need 88)", (int)len);
		return;
	}
	LOG_INF("loadPrefs: loaded %d bytes from %s", (int)len, path);
	size_t off = 0;
	memcpy(&prefs.airtime_factor, &buf[off], sizeof(float));
	off += 4;
	/* Migrate old AF multiplier (0-9) to duty cycle percentage (0-99) */
	if (prefs.airtime_factor > 0.0f && prefs.airtime_factor <= 9.0f) {
		prefs.airtime_factor *= 10.0f;
	}
	memcpy(prefs.node_name, &buf[off], 32);
	off += 36;
	memcpy(&prefs.node_lat, &buf[off], sizeof(double));
	off += 8;
	memcpy(&prefs.node_lon, &buf[off], sizeof(double));
	off += 8;
	memcpy(&prefs.freq, &buf[off], sizeof(float));
	off += 4;
	prefs.sf = buf[off++];
	prefs.cr = buf[off++];
	off++;
	prefs.manual_add_contacts = buf[off++];
	memcpy(&prefs.bw, &buf[off], sizeof(float));
	off += 4;
	prefs.tx_power_dbm = buf[off++];
	prefs.telemetry_mode_base = buf[off++];
	prefs.telemetry_mode_loc = buf[off++];
	prefs.telemetry_mode_env = buf[off++];
	memcpy(&prefs.rx_delay_base, &buf[off], sizeof(float));
	off += 4;
	prefs.advert_loc_policy = buf[off++];
	prefs.multi_acks = buf[off++];
	off += 2;
	memcpy(&prefs.ble_pin, &buf[off], sizeof(uint32_t));
	off += 4;
	prefs.buzzer_quiet = buf[off++];
	prefs.gps_enabled = buf[off++];
	memcpy(&prefs.gps_interval, &buf[off], sizeof(uint32_t));
	off += 4;
	prefs.autoadd_config = buf[off++];

	/* rx_boost - added later, may not exist in old files */
	if (off < len) {
		prefs.rx_boost = buf[off++];
	} else {
		prefs.rx_boost = 1;  /* Default to boosted for better sensitivity */
	}

	/* client_repeat (offgrid mode) - added v9, may not exist in old files */
	if (off < len) {
		prefs.client_repeat = buf[off++];
	} else {
		prefs.client_repeat = 0;  /* Default: disabled (companion only) */
	}

	if (strcmp(path, "/lfs/node_prefs") == 0) {
		savePrefs(prefs);
		removeFile("/lfs/node_prefs");
	}
}

void ZephyrDataStore::savePrefs(const NodePrefs &prefs)
{
	uint8_t buf[256];
	uint8_t pad[8] = {0};
	size_t off = 0;
	memcpy(&buf[off], &prefs.airtime_factor, sizeof(float));
	off += 4;
	memcpy(&buf[off], prefs.node_name, 32);
	off += 32;
	memcpy(&buf[off], pad, 4);
	off += 4;
	memcpy(&buf[off], &prefs.node_lat, sizeof(double));
	off += 8;
	memcpy(&buf[off], &prefs.node_lon, sizeof(double));
	off += 8;
	memcpy(&buf[off], &prefs.freq, sizeof(float));
	off += 4;
	buf[off++] = prefs.sf;
	buf[off++] = prefs.cr;
	buf[off++] = 0;
	buf[off++] = prefs.manual_add_contacts;
	memcpy(&buf[off], &prefs.bw, sizeof(float));
	off += 4;
	buf[off++] = prefs.tx_power_dbm;
	buf[off++] = prefs.telemetry_mode_base;
	buf[off++] = prefs.telemetry_mode_loc;
	buf[off++] = prefs.telemetry_mode_env;
	memcpy(&buf[off], &prefs.rx_delay_base, sizeof(float));
	off += 4;
	buf[off++] = prefs.advert_loc_policy;
	buf[off++] = prefs.multi_acks;
	memcpy(&buf[off], pad, 2);
	off += 2;
	memcpy(&buf[off], &prefs.ble_pin, sizeof(uint32_t));
	off += 4;
	buf[off++] = prefs.buzzer_quiet;
	buf[off++] = prefs.gps_enabled;
	memcpy(&buf[off], &prefs.gps_interval, sizeof(uint32_t));
	off += 4;
	buf[off++] = prefs.autoadd_config;
	buf[off++] = prefs.rx_boost;
	buf[off++] = prefs.client_repeat;
	bool ok = openWrite(PREFS_FILE, buf, off);
	LOG_INF("savePrefs: wrote %s, ok=%d (%d bytes), name='%.16s'", PREFS_FILE, ok ? 1 : 0, (int)off, prefs.node_name);
}

void ZephyrDataStore::loadContacts(DataStoreHost *host)
{
	const char *path = contactsFile();
	LOG_INF("loadContacts: path=%s (_has_ext_fs=%d)", path, _has_ext_fs ? 1 : 0);

	/* Check if file exists and its size */
	struct fs_dirent ent;
	if (fs_stat(path, &ent) == 0) {
		LOG_INF("loadContacts: file exists, size=%zu bytes", ent.size);
	} else {
		LOG_WRN("loadContacts: file does not exist at %s", path);
	}

	struct fs_file_t file;
	fs_file_t_init(&file);
	int rc = fs_open(&file, path, FS_O_READ);
	if (rc < 0) {
		LOG_WRN("loadContacts: fs_open failed (rc=%d)", rc);
		return;
	}
	uint32_t count = 0;
	for (;;) {
		ContactInfo c;
		uint8_t pub_key[32];
		uint8_t unused;
		ssize_t n = fs_read(&file, pub_key, 32);
		if (n != 32) break;
		n = fs_read(&file, (uint8_t *)&c.name, 32);
		if (n != 32) break;
		n = fs_read(&file, &c.type, 1);
		if (n != 1) break;
		n = fs_read(&file, &c.flags, 1);
		if (n != 1) break;
		n = fs_read(&file, &unused, 1);
		if (n != 1) break;
		n = fs_read(&file, (uint8_t *)&c.sync_since, 4);
		if (n != 4) break;
		n = fs_read(&file, (uint8_t *)&c.out_path_len, 1);
		if (n != 1) break;
		n = fs_read(&file, (uint8_t *)&c.last_advert_timestamp, 4);
		if (n != 4) break;
		n = fs_read(&file, c.out_path, 64);
		if (n != 64) break;
		n = fs_read(&file, (uint8_t *)&c.lastmod, 4);
		if (n != 4) break;
		n = fs_read(&file, (uint8_t *)&c.gps_lat, 4);
		if (n != 4) break;
		n = fs_read(&file, (uint8_t *)&c.gps_lon, 4);
		if (n != 4) break;
		c.id = mesh::Identity(pub_key);
		c.shared_secret_valid = false;
		if (!host->onContactLoaded(c)) break;
		count++;
	}
	fs_close(&file);
	LOG_INF("loadContacts: loaded %u contacts from %s", count, path);
}

void ZephyrDataStore::saveContacts(DataStoreHost *host)
{
	const char *path = contactsFile();
	LOG_INF("saveContacts: path=%s (_has_ext_fs=%d)", path, _has_ext_fs ? 1 : 0);

	/* Crash-safe write: write to .tmp, sync, then atomic rename.
	 * If power is lost before rename completes, the original file
	 * is still intact.  Stale .tmp files are cleaned at boot. */
	char tmp_path[48];
	snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);

	struct fs_file_t file;
	fs_file_t_init(&file);
	if (exists(tmp_path)) {
		fs_unlink(tmp_path);
	}
	int rc = fs_open(&file, tmp_path, FS_O_CREATE | FS_O_WRITE);
	if (rc < 0) {
		LOG_ERR("saveContacts: fs_open(%s) failed: %d", tmp_path, rc);
		return;
	}
	uint32_t idx = 0;
	ContactInfo c;
	uint8_t unused = 0;
	while (host->getContactForSave(idx, c)) {
		if (fs_write(&file, c.id.pub_key, 32) != 32) break;
		if (fs_write(&file, (uint8_t *)&c.name, 32) != 32) break;
		if (fs_write(&file, &c.type, 1) != 1) break;
		if (fs_write(&file, &c.flags, 1) != 1) break;
		if (fs_write(&file, &unused, 1) != 1) break;
		if (fs_write(&file, (uint8_t *)&c.sync_since, 4) != 4) break;
		if (fs_write(&file, (uint8_t *)&c.out_path_len, 1) != 1) break;
		if (fs_write(&file, (uint8_t *)&c.last_advert_timestamp, 4) != 4) break;
		if (fs_write(&file, c.out_path, 64) != 64) break;
		if (fs_write(&file, (uint8_t *)&c.lastmod, 4) != 4) break;
		if (fs_write(&file, (uint8_t *)&c.gps_lat, 4) != 4) break;
		if (fs_write(&file, (uint8_t *)&c.gps_lon, 4) != 4) break;
		idx++;
	}
	int sync_rc = fs_sync(&file);
	fs_close(&file);

	if (sync_rc < 0) {
		LOG_ERR("saveContacts: sync failed (%d), keeping old file", sync_rc);
		fs_unlink(tmp_path);
		return;
	}

	/* Atomic replace — LittleFS 2 rename() replaces the destination
	 * in a single CRC-protected metadata commit.  No unlink needed.
	 * Power loss at any point: either old file or new file, never empty. */
	rc = fs_rename(tmp_path, path);
	if (rc < 0) {
		LOG_ERR("saveContacts: rename %s → %s failed: %d", tmp_path, path, rc);
	} else {
		LOG_INF("saveContacts: saved %u contacts to %s", idx, path);
	}
}

void ZephyrDataStore::loadChannels(DataStoreHost *host)
{
	const char *path = channelsFile();
	struct fs_file_t file;
	fs_file_t_init(&file);
	if (fs_open(&file, path, FS_O_READ) < 0) {
		return;
	}
	uint8_t channel_idx = 0;
	for (;;) {
		ChannelDetails ch;
		uint8_t unused[4];
		ssize_t n = fs_read(&file, unused, 4);
		if (n != 4) break;
		n = fs_read(&file, (uint8_t *)ch.name, 32);
		if (n != 32) break;
		n = fs_read(&file, (uint8_t *)ch.channel.secret, 32);
		if (n != 32) break;
		if (host->onChannelLoaded(channel_idx, ch)) {
			channel_idx++;
		} else {
			break;
		}
	}
	fs_close(&file);
}

void ZephyrDataStore::saveChannels(DataStoreHost *host)
{
	const char *path = channelsFile();

	/* Crash-safe write: write to .tmp, sync, then atomic rename. */
	char tmp_path[48];
	snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);

	struct fs_file_t file;
	fs_file_t_init(&file);
	if (exists(tmp_path)) {
		fs_unlink(tmp_path);
	}
	if (fs_open(&file, tmp_path, FS_O_CREATE | FS_O_WRITE) < 0) {
		LOG_ERR("saveChannels: fs_open(%s) failed", tmp_path);
		return;
	}
	uint8_t channel_idx = 0;
	ChannelDetails ch;
	uint8_t unused[4] = {0};
	while (host->getChannelForSave(channel_idx, ch)) {
		if (fs_write(&file, unused, 4) != 4) break;
		if (fs_write(&file, (uint8_t *)ch.name, 32) != 32) break;
		if (fs_write(&file, (uint8_t *)ch.channel.secret, 32) != 32) break;
		channel_idx++;
	}
	int sync_rc = fs_sync(&file);
	fs_close(&file);

	if (sync_rc < 0) {
		LOG_ERR("saveChannels: sync failed (%d), keeping old file", sync_rc);
		fs_unlink(tmp_path);
		return;
	}

	/* Atomic replace — LittleFS 2 rename() replaces the destination
	 * in a single CRC-protected metadata commit.  No unlink needed. */
	int rc = fs_rename(tmp_path, path);
	if (rc < 0) {
		LOG_ERR("saveChannels: rename %s → %s failed: %d", tmp_path, path, rc);
	} else {
		LOG_DBG("saveChannels: saved %u channels to %s", channel_idx, path);
	}
}

uint8_t ZephyrDataStore::getBlobByKey(const uint8_t key[], int key_len, uint8_t dest_buf[])
{
	(void)key_len;
	const char *path = advBlobsFile();
	struct fs_file_t file;
	fs_file_t_init(&file);
	if (fs_open(&file, path, FS_O_READ) < 0) {
		return 0;
	}
	BlobRec tmp;
	uint8_t len = 0;
	while (fs_read(&file, (uint8_t *)&tmp, sizeof(tmp)) == (ssize_t)sizeof(tmp)) {
		if (memcmp(key, tmp.key, 7) == 0) {
			len = tmp.len;
			memcpy(dest_buf, tmp.data, len);
			break;
		}
	}
	fs_close(&file);
	return len;
}

bool ZephyrDataStore::putBlobByKey(const uint8_t key[], int key_len, const uint8_t src_buf[], uint8_t len)
{
	(void)key_len;
	if (len < PUB_KEY_SIZE + 4 + SIGNATURE_SIZE || len > MAX_ADVERT_PKT_LEN) {
		return false;
	}
	checkAdvBlobFile();
	const char *path = advBlobsFile();
	struct fs_file_t file;
	fs_file_t_init(&file);
	if (fs_open(&file, path, FS_O_RDWR) < 0) {
		return false;
	}
	uint32_t pos = 0, found_pos = 0;
	uint32_t min_timestamp = 0xFFFFFFFF;
	BlobRec tmp;
	while (fs_read(&file, (uint8_t *)&tmp, sizeof(tmp)) == (ssize_t)sizeof(tmp)) {
		if (memcmp(key, tmp.key, 7) == 0) {
			found_pos = pos;
			break;
		}
		if (tmp.timestamp < min_timestamp) {
			min_timestamp = tmp.timestamp;
			found_pos = pos;
		}
		pos += sizeof(tmp);
	}
	memcpy(tmp.key, key, 7);
	memcpy(tmp.data, src_buf, len);
	tmp.len = len;
	tmp.timestamp = _clock->getCurrentTime();
	fs_seek(&file, found_pos, FS_SEEK_SET);
	fs_write(&file, (uint8_t *)&tmp, sizeof(tmp));
	fs_close(&file);
	return true;
}

bool ZephyrDataStore::deleteBlobByKey(const uint8_t key[], int key_len)
{
	(void)key_len;
	const char *path = advBlobsFile();
	struct fs_file_t file;
	fs_file_t_init(&file);
	if (fs_open(&file, path, FS_O_RDWR) < 0) {
		return true;  // no blob file = nothing to delete
	}
	BlobRec tmp;
	uint32_t pos = 0;
	while (fs_read(&file, (uint8_t *)&tmp, sizeof(tmp)) == (ssize_t)sizeof(tmp)) {
		if (memcmp(key, tmp.key, 7) == 0) {
			/* Zero out the record to mark as free */
			memset(&tmp, 0, sizeof(tmp));
			fs_seek(&file, pos, FS_SEEK_SET);
			fs_write(&file, (uint8_t *)&tmp, sizeof(tmp));
			break;
		}
		pos += sizeof(tmp);
	}
	fs_close(&file);
	return true;
}

uint32_t ZephyrDataStore::getStorageUsedKb() const
{
	struct fs_statvfs sbuf;
	uint32_t used = 0;

	/* Internal flash */
	if (fs_statvfs(mountPoint(), &sbuf) == 0) {
		uint32_t total = sbuf.f_blocks * sbuf.f_frsize;
		uint32_t free = sbuf.f_bfree * sbuf.f_frsize;
		used += (total - free) / 1024;
	}

	/* External flash if available */
	if (_has_ext_fs && fs_statvfs(extMountPoint(), &sbuf) == 0) {
		uint32_t total = sbuf.f_blocks * sbuf.f_frsize;
		uint32_t free = sbuf.f_bfree * sbuf.f_frsize;
		used += (total - free) / 1024;
	}

	return used;
}

uint32_t ZephyrDataStore::getStorageTotalKb() const
{
	struct fs_statvfs sbuf;
	uint32_t total = 0;

	/* Internal flash */
	if (fs_statvfs(mountPoint(), &sbuf) == 0) {
		total += (sbuf.f_blocks * sbuf.f_frsize) / 1024;
	}

	/* External flash if available */
	if (_has_ext_fs && fs_statvfs(extMountPoint(), &sbuf) == 0) {
		total += (sbuf.f_blocks * sbuf.f_frsize) / 1024;
	}

	return total;
}

uint32_t ZephyrDataStore::getExternalStorageKb() const
{
	if (!_has_ext_fs) {
		return 0;
	}
	struct fs_statvfs sbuf;
	if (fs_statvfs(extMountPoint(), &sbuf) < 0) {
		return 0;
	}
	return (sbuf.f_blocks * sbuf.f_frsize) / 1024;
}

void ZephyrDataStore::factoryReset()
{
	LOG_INF("=== FACTORY RESET STARTING ===");

	/* Format both internal and external (QSPI) filesystems */
	if (formatFileSystem()) {
		LOG_INF("=== FACTORY RESET COMPLETE - REBOOT REQUIRED ===");
	} else {
		LOG_ERR("=== FACTORY RESET FAILED ===");
	}
}

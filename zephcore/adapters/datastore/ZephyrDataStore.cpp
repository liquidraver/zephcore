/*
 * SPDX-License-Identifier: Apache-2.0
 * Zephyr DataStore - LittleFS-backed persistence with optional QSPI flash
 *
 * Universal mount strategy:
 *   nRF52:  Manual mount /efs (ExtraFS, 100KB) + /ifs (InternalFS, 28KB)
 *           with block_size=128 and custom erase for Arduino MeshCore compat.
 *   Others: DTS-automounted /lfs (single partition).
 *   QSPI:   /ext overrides contacts mount when available.
 *
 * Data format (all platforms):
 *   contacts3   — 152-byte records (Arduino-compatible)
 *   new_prefs   — Arduino-compatible byte layout
 *   channels2   — 68-byte records (identical to Arduino)
 *   _main.id    — identity blob
 *   adv_blobs   — fixed record array
 */

#include "ZephyrDataStore.h"
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/device.h>
#include <string.h>

/* Custom 128B-block erase — defined in lfs_128b_erase.c */
extern "C" int lfs_128b_erase(const struct lfs_config *c, lfs_block_t block);

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_datastore, CONFIG_ZEPHCORE_DATASTORE_LOG_LEVEL);

#define MAX_ADVERT_PKT_LEN (2 + 32 + PUB_KEY_SIZE + 4 + SIGNATURE_SIZE + MAX_ADVERT_DATA_SIZE)

struct BlobRec {
	uint32_t timestamp;
	uint8_t key[7];
	uint8_t len;
	uint8_t data[MAX_ADVERT_PKT_LEN];
};

/* ── Mount state ────────────────────────────────────────────────────── */

/* Resolved at mount time */
const char *ZephyrDataStore::_contacts_mnt = nullptr;
const char *ZephyrDataStore::_prefs_mnt = nullptr;

static bool efs_mounted;   /* /efs (nRF52 ExtraFS) or false */
static bool ifs_mounted;   /* /ifs (nRF52 InternalFS) or false */
static bool lfs_mounted;   /* /lfs (DTS automount) or false */
static bool ext_lfs_mounted; /* /ext (QSPI) */

/* Check if a filesystem is mounted using fs_statvfs */
static bool is_mounted(const char *mount_point)
{
	struct fs_statvfs stat;
	return fs_statvfs(mount_point, &stat) == 0;
}

/* ── nRF52 dual-mount: ExtraFS + InternalFS (block_size=128) ───────── */

#if FIXED_PARTITION_EXISTS(extrafs_partition) && FIXED_PARTITION_EXISTS(internalfs_partition)
#define HAS_NRF52_DUAL_MOUNT 1

/* Arduino-compatible LFS: block_size=128 */
FS_LITTLEFS_DECLARE_CUSTOM_CONFIG(extrafs_data,
	4,    /* alignment */
	16,   /* read_size */
	16,   /* prog_size */
	128,  /* cache_size = block_size */
	128   /* lookahead_size */
);

FS_LITTLEFS_DECLARE_CUSTOM_CONFIG(internalfs_data,
	4,    /* alignment */
	16,   /* read_size */
	16,   /* prog_size */
	128,  /* cache_size = block_size */
	128   /* lookahead_size */
);

static struct fs_mount_t extrafs_mnt = {
	.type = FS_LITTLEFS,
	.mnt_point = "/efs",
	.fs_data = &extrafs_data,
	.storage_dev = (void *)FIXED_PARTITION_ID(extrafs_partition),
	.flags = 0,
};

static struct fs_mount_t internalfs_mnt = {
	.type = FS_LITTLEFS,
	.mnt_point = "/ifs",
	.fs_data = &internalfs_data,
	.storage_dev = (void *)FIXED_PARTITION_ID(internalfs_partition),
	.flags = 0,
};

static bool try_mount_nrf52_dual()
{
	int rc;

	/* Set custom block_size and erase callback before mount.
	 * The Zephyr LFS patch (0007) preserves pre-set erase callbacks. */
	extrafs_data.cfg.block_size = 128;
	extrafs_data.cfg.erase = lfs_128b_erase;

	internalfs_data.cfg.block_size = 128;
	internalfs_data.cfg.erase = lfs_128b_erase;

	/* Mount ExtraFS — contacts, channels, blobs */
	rc = fs_mount(&extrafs_mnt);
	if (rc < 0) {
		LOG_ERR("ExtraFS mount failed: %d — formatting", rc);
		const struct flash_area *fap;
		if (flash_area_open(FIXED_PARTITION_ID(extrafs_partition), &fap) == 0) {
			flash_area_flatten(fap, 0, fap->fa_size);
			flash_area_close(fap);
		}
		rc = fs_mount(&extrafs_mnt);
		if (rc < 0) {
			LOG_ERR("ExtraFS mount failed after format: %d", rc);
			return false;
		}
	}
	efs_mounted = true;
	LOG_INF("ExtraFS at /efs (100KB, 128B blocks)");

	/* Mount InternalFS — prefs, identity, BLE settings */
	rc = fs_mount(&internalfs_mnt);
	if (rc < 0) {
		LOG_ERR("InternalFS mount failed: %d — formatting", rc);
		const struct flash_area *fap;
		if (flash_area_open(FIXED_PARTITION_ID(internalfs_partition), &fap) == 0) {
			flash_area_flatten(fap, 0, fap->fa_size);
			flash_area_close(fap);
		}
		rc = fs_mount(&internalfs_mnt);
		if (rc < 0) {
			LOG_ERR("InternalFS mount failed after format: %d", rc);
			return false;
		}
	}
	ifs_mounted = true;
	LOG_INF("InternalFS at /ifs (28KB, 128B blocks)");

	return true;
}

#else
#define HAS_NRF52_DUAL_MOUNT 0
static bool try_mount_nrf52_dual() { return false; }
#endif /* extrafs_partition && internalfs_partition */

/* ── Universal mount ───────────────────────────────────────────────── */

bool ZephyrDataStore::mount()
{
	if (_contacts_mnt != nullptr) {
		return true;  /* already mounted */
	}

	/* 1. Try nRF52 dual-mount (manual, block_size=128) */
	if (try_mount_nrf52_dual()) {
		_contacts_mnt = "/efs";
		_prefs_mnt = "/ifs";
		LOG_INF("nRF52 dual-mount: contacts=/efs, prefs=/ifs");
	}
	/* 2. Fallback: DTS automount at /lfs (ESP32, MG24, nRF54L) */
	else if (is_mounted("/lfs")) {
		_contacts_mnt = "/lfs";
		_prefs_mnt = "/lfs";
		lfs_mounted = true;
		LOG_INF("Single-mount: contacts=/lfs, prefs=/lfs (DTS automount)");
	}
	else {
		LOG_ERR("No filesystem mounted!");
		return false;
	}

	/* 3. QSPI /ext override (optional, any platform) */
	if (is_mounted(EXT_MNT_POINT)) {
		ext_lfs_mounted = true;
		_contacts_mnt = EXT_MNT_POINT;
		LOG_INF("QSPI mounted at /ext — contacts redirected to QSPI");
	}

	return true;
}

void ZephyrDataStore::unmount()
{
#if HAS_NRF52_DUAL_MOUNT
	if (efs_mounted) {
		fs_unmount(&extrafs_mnt);
		efs_mounted = false;
	}
	if (ifs_mounted) {
		fs_unmount(&internalfs_mnt);
		ifs_mounted = false;
	}
#endif
	lfs_mounted = false;
	ext_lfs_mounted = false;
	_contacts_mnt = nullptr;
	_prefs_mnt = nullptr;
}

/* ── Path helpers ──────────────────────────────────────────────────── */

static char path_buf[3][48];  /* reusable path buffers */

const char *ZephyrDataStore::contactsFile() const
{
	snprintf(path_buf[0], sizeof(path_buf[0]), "%s/contacts3", _contacts_mnt);
	return path_buf[0];
}

const char *ZephyrDataStore::channelsFile() const
{
	snprintf(path_buf[1], sizeof(path_buf[1]), "%s/channels2", _contacts_mnt);
	return path_buf[1];
}

const char *ZephyrDataStore::advBlobsFile() const
{
	snprintf(path_buf[2], sizeof(path_buf[2]), "%s/adv_blobs", _contacts_mnt);
	return path_buf[2];
}

const char *ZephyrDataStore::prefsFile()
{
	static char buf[48];
	snprintf(buf, sizeof(buf), "%s/new_prefs", _prefs_mnt);
	return buf;
}

const char *ZephyrDataStore::identityFile()
{
	static char buf[48];
	snprintf(buf, sizeof(buf), "%s/_main.id", _prefs_mnt);
	return buf;
}

/* ── Init ──────────────────────────────────────────────────────────── */

ZephyrDataStore::ZephyrDataStore(mesh::RTCClock &clock)
	: _clock(&clock), _has_ext_fs(false)
{
}

void ZephyrDataStore::begin()
{
	_has_ext_fs = ext_lfs_mounted;
	LOG_INF("_has_ext_fs=%d, contacts_mnt=%s, prefs_mnt=%s",
		_has_ext_fs ? 1 : 0, _contacts_mnt, _prefs_mnt);
	LOG_INF("contacts=%s, prefs=%s", contactsFile(), prefsFile());

	if (_has_ext_fs) {
		migrateToExternalFS();
	}

	checkAdvBlobFile();
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
	if (fs_open(&file, path, FS_O_READ) < 0) {
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
	struct fs_file_t file;
	fs_file_t_init(&file);
	if (exists(path)) {
		fs_unlink(path);
	}
	if (fs_open(&file, path, FS_O_CREATE | FS_O_WRITE) < 0) {
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

/* ── QSPI migration ───────────────────────────────────────────────── */

void ZephyrDataStore::migrateToExternalFS()
{
	/* Build internal paths (contacts are on the non-QSPI internal mount).
	 * On nRF52: /efs/contacts3, on others: /lfs/contacts3 */
	const char *int_mnt = efs_mounted ? "/efs" : "/lfs";
	char int_contacts[48], ext_contacts[48];
	char int_channels[48], ext_channels[48];
	char int_blobs[48], ext_blobs[48];

	snprintf(int_contacts, sizeof(int_contacts), "%s/contacts3", int_mnt);
	snprintf(ext_contacts, sizeof(ext_contacts), "%s/contacts3", EXT_MNT_POINT);
	snprintf(int_channels, sizeof(int_channels), "%s/channels2", int_mnt);
	snprintf(ext_channels, sizeof(ext_channels), "%s/channels2", EXT_MNT_POINT);
	snprintf(int_blobs, sizeof(int_blobs), "%s/adv_blobs", int_mnt);
	snprintf(ext_blobs, sizeof(ext_blobs), "%s/adv_blobs", EXT_MNT_POINT);

	/* Migrate contacts */
	if (!exists(ext_contacts) && exists(int_contacts)) {
		LOG_INF("Migrating contacts to QSPI");
		if (copyFile(int_contacts, ext_contacts)) {
			removeFile(int_contacts);
		}
	}

	/* Migrate channels */
	if (!exists(ext_channels) && exists(int_channels)) {
		LOG_INF("Migrating channels to QSPI");
		if (copyFile(int_channels, ext_channels)) {
			removeFile(int_channels);
		}
	}

	/* Migrate adv_blobs (extend to 100 records) */
	if (!exists(ext_blobs) && exists(int_blobs)) {
		LOG_INF("Migrating adv_blobs to QSPI (20 -> 100 slots)");
		if (copyFile(int_blobs, ext_blobs)) {
			removeFile(int_blobs);
			struct fs_file_t file;
			fs_file_t_init(&file);
			if (fs_open(&file, ext_blobs, FS_O_RDWR) == 0) {
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
	if (exists(ext_contacts) && exists(int_contacts)) {
		removeFile(int_contacts);
	}
	if (exists(ext_channels) && exists(int_channels)) {
		removeFile(int_channels);
	}
	if (exists(ext_blobs) && exists(int_blobs)) {
		removeFile(int_blobs);
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
	int recs = maxBlobRecs();
	for (int i = 0; i < recs; i++) {
		fs_write(&file, &zeroes, sizeof(zeroes));
	}
	fs_close(&file);
}

/* ── Format / Factory Reset ────────────────────────────────────────── */

bool ZephyrDataStore::formatFileSystem()
{
	LOG_INF("formatFileSystem: starting...");
	unmount();

	const struct flash_area *fap;
	int rc;

#if FIXED_PARTITION_EXISTS(extrafs_partition)
	/* nRF52: format ExtraFS */
	rc = flash_area_open(FIXED_PARTITION_ID(extrafs_partition), &fap);
	if (rc == 0) {
		LOG_INF("Formatting ExtraFS (%u bytes)", (unsigned)fap->fa_size);
		flash_area_flatten(fap, 0, fap->fa_size);
		flash_area_close(fap);
	}
#endif

#if FIXED_PARTITION_EXISTS(internalfs_partition)
	/* nRF52: format InternalFS */
	rc = flash_area_open(FIXED_PARTITION_ID(internalfs_partition), &fap);
	if (rc == 0) {
		LOG_INF("Formatting InternalFS (%u bytes)", (unsigned)fap->fa_size);
		flash_area_flatten(fap, 0, fap->fa_size);
		flash_area_close(fap);
	}
#endif

#if FIXED_PARTITION_EXISTS(lfs_partition)
	/* Non-nRF52: format single LFS partition */
	rc = flash_area_open(FIXED_PARTITION_ID(lfs_partition), &fap);
	if (rc == 0) {
		LOG_INF("Formatting LFS partition (%u bytes)", (unsigned)fap->fa_size);
		flash_area_flatten(fap, 0, fap->fa_size);
		flash_area_close(fap);
	}
#endif

#if FIXED_PARTITION_EXISTS(storage_partition)
	/* Non-nRF52: format NVS storage (BLE bonds) */
	rc = flash_area_open(FIXED_PARTITION_ID(storage_partition), &fap);
	if (rc == 0) {
		LOG_INF("Formatting NVS storage (%u bytes)", (unsigned)fap->fa_size);
		flash_area_flatten(fap, 0, fap->fa_size);
		flash_area_close(fap);
	}
#endif

#if FIXED_PARTITION_EXISTS(qspi_storage_partition)
	/* QSPI if present (any platform) */
	rc = flash_area_open(FIXED_PARTITION_ID(qspi_storage_partition), &fap);
	if (rc == 0) {
		LOG_INF("Formatting QSPI (%u bytes, may take a while)", (unsigned)fap->fa_size);
		flash_area_flatten(fap, 0, fap->fa_size);
		flash_area_close(fap);
	}
#endif

	bool mounted = mount();
	LOG_INF("formatFileSystem: mount() returned %d", mounted ? 1 : 0);
	return mounted;
}

void ZephyrDataStore::factoryReset()
{
	LOG_INF("=== FACTORY RESET STARTING ===");
	if (formatFileSystem()) {
		LOG_INF("=== FACTORY RESET COMPLETE - REBOOT REQUIRED ===");
	} else {
		LOG_ERR("=== FACTORY RESET FAILED ===");
	}
}

/* ── Identity ──────────────────────────────────────────────────────── */

bool ZephyrDataStore::loadMainIdentity(mesh::LocalIdentity &identity)
{
	uint8_t buf[PRV_KEY_SIZE + PUB_KEY_SIZE + 32];
	size_t len = 0;
	if (!openRead(identityFile(), buf, sizeof(buf), len) || len < PRV_KEY_SIZE + PUB_KEY_SIZE) {
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
	return openWrite(identityFile(), buf, n);
}

/* ── Preferences (Arduino-compatible layout) ───────────────────────── */

void ZephyrDataStore::loadPrefs(NodePrefs &prefs)
{
	const char *path = prefsFile();
	bool prefs_exists = exists(path);
	LOG_INF("loadPrefs: exists(%s)=%d", path, prefs_exists ? 1 : 0);
	if (!prefs_exists) {
		LOG_WRN("loadPrefs: no prefs file found");
		return;
	}

	uint8_t buf[256];
	size_t len = 0;
	if (!openRead(path, buf, sizeof(buf), len)) {
		LOG_WRN("loadPrefs: read failed");
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
	off += 36;  /* 32 name + 4 pad */
	memcpy(&prefs.node_lat, &buf[off], sizeof(double));
	off += 8;
	memcpy(&prefs.node_lon, &buf[off], sizeof(double));
	off += 8;
	memcpy(&prefs.freq, &buf[off], sizeof(float));
	off += 4;
	prefs.sf = buf[off++];
	prefs.cr = buf[off++];
	/* Offset 62: client_repeat (Arduino-compatible placement) */
	prefs.client_repeat = buf[off++];
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
	/* Offset 78: path_hash_mode (Arduino treats as pad — harmless) */
	prefs.path_hash_mode = buf[off++];
	off += 1;  /* pad */
	memcpy(&prefs.ble_pin, &buf[off], sizeof(uint32_t));
	off += 4;
	prefs.buzzer_quiet = buf[off++];
	prefs.gps_enabled = buf[off++];
	memcpy(&prefs.gps_interval, &buf[off], sizeof(uint32_t));
	off += 4;
	prefs.autoadd_config = buf[off++];

	/* Offset 91: rx_boost (ZephCore extension — Arduino stops at 91 bytes) */
	if (off < len) {
		prefs.rx_boost = buf[off++];
	} else {
		prefs.rx_boost = 1;  /* Default to boosted for better sensitivity */
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
	/* Offset 62: client_repeat (Arduino-compatible placement) */
	buf[off++] = prefs.client_repeat;
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
	/* Offset 78: path_hash_mode (Arduino treats as pad — harmless) */
	buf[off++] = prefs.path_hash_mode;
	buf[off++] = 0;  /* pad */
	memcpy(&buf[off], &prefs.ble_pin, sizeof(uint32_t));
	off += 4;
	buf[off++] = prefs.buzzer_quiet;
	buf[off++] = prefs.gps_enabled;
	memcpy(&buf[off], &prefs.gps_interval, sizeof(uint32_t));
	off += 4;
	buf[off++] = prefs.autoadd_config;
	/* Offset 91: rx_boost (ZephCore extension — Arduino ignores) */
	buf[off++] = prefs.rx_boost;
	/* Total: 92 bytes (Arduino reads 91, ZephCore reads 92) */

	bool ok = openWrite(prefsFile(), buf, off);
	LOG_INF("savePrefs: wrote %s, ok=%d (%d bytes), name='%.16s'",
		prefsFile(), ok ? 1 : 0, (int)off, prefs.node_name);
}

/* ── Contacts: contacts3 (152B records, Arduino-compatible) ────────── */

static constexpr size_t CONTACT_DATA_SZ = 152;  /* 32+32+1+1+1+4+1+4+64+4+4+4 */

/* Pack a ContactInfo into the 152-byte wire format (Arduino contacts3) */
static void contact_to_record(const ContactInfo &c, uint8_t rec[CONTACT_DATA_SZ])
{
	uint8_t *p = rec;
	uint8_t unused = 0;
	memcpy(p, c.id.pub_key, 32);  p += 32;
	memcpy(p, c.name, 32);        p += 32;
	*p++ = c.type;
	*p++ = c.flags;
	*p++ = unused;
	memcpy(p, &c.sync_since, 4);             p += 4;
	*p++ = c.out_path_len;
	memcpy(p, &c.last_advert_timestamp, 4);  p += 4;
	memcpy(p, c.out_path, 64);               p += 64;
	memcpy(p, &c.lastmod, 4);                p += 4;
	memcpy(p, &c.gps_lat, 4);                p += 4;
	memcpy(p, &c.gps_lon, 4);                p += 4;
}

/* Unpack 152-byte wire format into a ContactInfo */
static void record_to_contact(const uint8_t rec[CONTACT_DATA_SZ], ContactInfo &c)
{
	const uint8_t *p = rec;
	uint8_t pub_key[32];
	uint8_t unused;
	memcpy(pub_key, p, 32);    p += 32;
	memcpy(c.name, p, 32);    p += 32;
	c.type = *p++;
	c.flags = *p++;
	unused = *p++;  (void)unused;
	memcpy(&c.sync_since, p, 4);             p += 4;
	c.out_path_len = *p++;
	memcpy(&c.last_advert_timestamp, p, 4);  p += 4;
	memcpy(c.out_path, p, 64);               p += 64;
	memcpy(&c.lastmod, p, 4);                p += 4;
	memcpy(&c.gps_lat, p, 4);                p += 4;
	memcpy(&c.gps_lon, p, 4);                p += 4;
	c.id = mesh::Identity(pub_key);
	c.shared_secret_valid = false;
}

void ZephyrDataStore::loadContacts(DataStoreHost *host)
{
	const char *path = contactsFile();
	LOG_INF("loadContacts: path=%s", path);

	struct fs_file_t file;
	fs_file_t_init(&file);
	int rc = fs_open(&file, path, FS_O_READ);
	if (rc < 0) {
		LOG_WRN("loadContacts: no contacts file found");
		return;
	}

	uint32_t count = 0;
	uint8_t rec[CONTACT_DATA_SZ];

	for (;;) {
		ssize_t n = fs_read(&file, rec, CONTACT_DATA_SZ);
		if (n <= 0) break;
		if (n != (ssize_t)CONTACT_DATA_SZ) {
			LOG_WRN("loadContacts: truncated record at #%u (%d bytes)",
				count, (int)n);
			break;
		}

		ContactInfo c;
		record_to_contact(rec, c);
		if (!host->onContactLoaded(c)) break;
		count++;
	}

	fs_close(&file);
	LOG_INF("loadContacts: loaded %u contacts from %s", count, path);
}

void ZephyrDataStore::saveContacts(DataStoreHost *host)
{
	const char *path = contactsFile();
	LOG_INF("saveContacts: path=%s", path);

	if (exists(path)) {
		fs_unlink(path);
	}

	struct fs_file_t file;
	fs_file_t_init(&file);
	int rc = fs_open(&file, path, FS_O_CREATE | FS_O_WRITE);
	if (rc < 0) {
		LOG_ERR("saveContacts: fs_open(%s) failed: %d", path, rc);
		return;
	}

	uint8_t rec[CONTACT_DATA_SZ];
	uint32_t idx = 0;
	ContactInfo c;

	while (host->getContactForSave(idx, c)) {
		contact_to_record(c, rec);

		if (fs_write(&file, rec, CONTACT_DATA_SZ) != (ssize_t)CONTACT_DATA_SZ) {
			LOG_ERR("saveContacts: write failed at record %u", idx);
			break;
		}
		idx++;
	}

	fs_sync(&file);
	fs_close(&file);
	LOG_INF("saveContacts: saved %u contacts to %s (%u bytes)",
		idx, path, idx * CONTACT_DATA_SZ);
}

/* ── Channels ──────────────────────────────────────────────────────── */

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
	LOG_INF("saveChannels: path=%s", path);

	if (exists(path)) {
		fs_unlink(path);
	}

	struct fs_file_t file;
	fs_file_t_init(&file);
	if (fs_open(&file, path, FS_O_CREATE | FS_O_WRITE) < 0) {
		LOG_ERR("saveChannels: fs_open(%s) failed", path);
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
	fs_sync(&file);
	fs_close(&file);
	LOG_INF("saveChannels: saved %u channels to %s", channel_idx, path);
}

/* ── Blobs ─────────────────────────────────────────────────────────── */

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
		return true;  /* no blob file = nothing to delete */
	}
	BlobRec tmp;
	uint32_t pos = 0;
	while (fs_read(&file, (uint8_t *)&tmp, sizeof(tmp)) == (ssize_t)sizeof(tmp)) {
		if (memcmp(key, tmp.key, 7) == 0) {
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

/* ── Storage stats ─────────────────────────────────────────────────── */

uint32_t ZephyrDataStore::getStorageUsedKb() const
{
	struct fs_statvfs sbuf;
	uint32_t used = 0;

	/* Primary contacts mount (/efs, /lfs, or /ext) */
	if (_contacts_mnt && fs_statvfs(_contacts_mnt, &sbuf) == 0) {
		uint32_t total = sbuf.f_blocks * sbuf.f_frsize;
		uint32_t free = sbuf.f_bfree * sbuf.f_frsize;
		used += (total - free) / 1024;
	}

	/* Prefs mount (if different from contacts) */
	if (_prefs_mnt && _prefs_mnt != _contacts_mnt &&
	    fs_statvfs(_prefs_mnt, &sbuf) == 0) {
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

	if (_contacts_mnt && fs_statvfs(_contacts_mnt, &sbuf) == 0) {
		total += (sbuf.f_blocks * sbuf.f_frsize) / 1024;
	}
	if (_prefs_mnt && _prefs_mnt != _contacts_mnt &&
	    fs_statvfs(_prefs_mnt, &sbuf) == 0) {
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
	if (fs_statvfs(EXT_MNT_POINT, &sbuf) < 0) {
		return 0;
	}
	return (sbuf.f_blocks * sbuf.f_frsize) / 1024;
}

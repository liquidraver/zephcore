/*
 * SPDX-License-Identifier: Apache-2.0
 * Zephyr DataStore - LittleFS-backed persistence with optional QSPI flash
 *
 * Universal across all platforms (nRF52, ESP32, MG24, nRF54L).
 * On nRF52, uses Arduino MeshCore-compatible dual-mount layout:
 *   /efs (ExtraFS @ 0xD4000, 100KB, block_size=128) — contacts3, channels2, blobs
 *   /ifs (InternalFS @ 0xED000, 28KB, block_size=128) — new_prefs, _main.id
 * On other platforms, uses DTS-automounted /lfs for everything.
 * QSPI /ext overrides contacts path when available (any platform).
 */

#pragma once

#include <mesh/Identity.h>
#include <mesh/RTC.h>
#include <NodePrefs.h>
#include <ContactInfo.h>
#include <ChannelDetails.h>

class DataStoreHost {
public:
	virtual bool onContactLoaded(const ContactInfo &contact) = 0;
	virtual bool getContactForSave(uint32_t idx, ContactInfo &contact) = 0;
	virtual bool onChannelLoaded(uint8_t channel_idx, const ChannelDetails &ch) = 0;
	virtual bool getChannelForSave(uint8_t channel_idx, ChannelDetails &ch) = 0;
};

class ZephyrDataStore {
public:
	explicit ZephyrDataStore(mesh::RTCClock &clock);
	void begin();
	bool formatFileSystem();
	bool loadMainIdentity(mesh::LocalIdentity &identity);
	bool saveMainIdentity(const mesh::LocalIdentity &identity);
	void loadPrefs(NodePrefs &prefs);
	void savePrefs(const NodePrefs &prefs);
	void loadContacts(DataStoreHost *host);
	void saveContacts(DataStoreHost *host);
	void loadChannels(DataStoreHost *host);
	void saveChannels(DataStoreHost *host);
	uint8_t getBlobByKey(const uint8_t key[], int key_len, uint8_t dest_buf[]);
	bool putBlobByKey(const uint8_t key[], int key_len, const uint8_t src_buf[], uint8_t len);
	bool deleteBlobByKey(const uint8_t key[], int key_len);
	uint32_t getStorageUsedKb() const;
	uint32_t getStorageTotalKb() const;

	/* Factory reset - delete all stored data */
	void factoryReset();

	/* Check if external QSPI flash is available */
	bool hasExternalStorage() const { return _has_ext_fs; }
	uint32_t getExternalStorageKb() const;

	static bool mount();
	static void unmount();

private:
	/* Mount points resolved at mount time:
	 * nRF52:  _contacts_mnt="/efs", _prefs_mnt="/ifs"
	 * Others: _contacts_mnt="/lfs", _prefs_mnt="/lfs"
	 * QSPI:   _contacts_mnt="/ext" (override) */
	static const char *_contacts_mnt;
	static const char *_prefs_mnt;

	/* External QSPI flash (optional, any platform) */
	static constexpr const char *EXT_MNT_POINT = "/ext";

	mesh::RTCClock *_clock;
	bool _has_ext_fs;

	/* Build full paths from resolved mount points */
	const char *contactsFile() const;
	const char *channelsFile() const;
	const char *advBlobsFile() const;
	static const char *prefsFile();
	static const char *identityFile();
	int maxBlobRecs() const { return _has_ext_fs ? 100 : 20; }

	void checkAdvBlobFile();
	void migrateToExternalFS();
	bool openRead(const char *path, uint8_t *buf, size_t buf_sz, size_t &out_len);
	bool openWrite(const char *path, const uint8_t *buf, size_t len);
	bool exists(const char *path);
	bool removeFile(const char *path);
	bool copyFile(const char *src, const char *dst);
};

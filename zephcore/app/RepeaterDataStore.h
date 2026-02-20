/*
 * SPDX-License-Identifier: Apache-2.0
 * RepeaterDataStore - Filesystem storage for repeater
 *
 * Uses /lfs/repeater/ prefix to keep data separate from companion.
 * This allows flashing back and forth between roles without corruption.
 */

#pragma once

#include <cstdint>
#include <stddef.h>
#include <mesh/Identity.h>
#include <helpers/NodePrefs.h>
#include <helpers/ClientACL.h>
#include <helpers/RegionMap.h>

class RepeaterDataStore {
public:
    RepeaterDataStore();

    /* Initialize filesystem and repeater directory */
    bool begin();

    /* Identity management */
    bool loadIdentity(mesh::LocalIdentity& id);
    bool saveIdentity(const mesh::LocalIdentity& id);

    /* Prefs management */
    bool loadPrefs(NodePrefs& prefs);
    bool savePrefs(const NodePrefs& prefs);

    /* ACL management - paths passed to ClientACL */
    const char* getAclPath() const { return "/lfs/repeater/acl"; }

    /* Region management - paths passed to RegionMap */
    const char* getRegionsPath() const { return "/lfs/repeater/regions2"; }

    /* Factory reset - erase all repeater data */
    bool formatFileSystem();

    /* Get base path for repeater storage */
    static const char* getBasePath() { return "/lfs/repeater"; }

private:
    bool _initialized;
};

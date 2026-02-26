# SPDX-License-Identifier: Apache-2.0
# ZephCore LR1110 Firmware Updater — updates LR1110 radio firmware to 0x0401.
#
# Board-specific: only builds for boards with an LR1110 radio (e.g., t1000_e).
#
# Build:  west build -b t1000_e/nrf52840 zephcore/tools/lr1110_updater --pristine
# Flash:  drag-drop build/zephyr/zephyr.uf2 onto UF2 drive
#         (or: nrfjprog --program build/zephyr/zephyr.hex --sectorerase -r)
#
# After the updater finishes, flash ZephCore main firmware.

cmake_minimum_required(VERSION 3.20.0)

# Reuse the main project's custom board definitions
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# Extract base board name (strip qualifier like /nrf52840)
string(REPLACE "/" ";" BOARD_PARTS ${BOARD})
list(GET BOARD_PARTS 0 BOARD_BASE)

# Find board overlay — prefer updater-specific overlay, fall back to main board
file(GLOB_RECURSE UPD_OVERLAY_CANDIDATES
     "${CMAKE_CURRENT_SOURCE_DIR}/boards/*/${BOARD_BASE}/board.overlay"
     "${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD_BASE}/board.overlay")
if(UPD_OVERLAY_CANDIDATES)
    list(GET UPD_OVERLAY_CANDIDATES 0 UPDATER_BOARD_OVERLAY)
else()
    # Fall back to main board overlay
    file(GLOB_RECURSE BOARD_OVERLAY_CANDIDATES
         "${CMAKE_CURRENT_SOURCE_DIR}/../../boards/*/${BOARD_BASE}/board.overlay")
    if(BOARD_OVERLAY_CANDIDATES)
        list(GET BOARD_OVERLAY_CANDIDATES 0 UPDATER_BOARD_OVERLAY)
    endif()
endif()
if(UPDATER_BOARD_OVERLAY)
    if(EXTRA_DTC_OVERLAY_FILE)
        set(EXTRA_DTC_OVERLAY_FILE "${EXTRA_DTC_OVERLAY_FILE};${UPDATER_BOARD_OVERLAY}" CACHE STRING "" FORCE)
    else()
        set(EXTRA_DTC_OVERLAY_FILE "${UPDATER_BOARD_OVERLAY}" CACHE STRING "" FORCE)
    endif()
endif()

# Find updater-specific board.conf
file(GLOB_RECURSE UPD_BOARD_CONF_CANDIDATES
     "${CMAKE_CURRENT_SOURCE_DIR}/boards/*/${BOARD_BASE}/board.conf"
     "${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD_BASE}/board.conf")
if(UPD_BOARD_CONF_CANDIDATES)
    list(GET UPD_BOARD_CONF_CANDIDATES 0 UPD_BOARD_CONF)
    if(EXTRA_CONF_FILE)
        set(EXTRA_CONF_FILE "${EXTRA_CONF_FILE};${UPD_BOARD_CONF}" CACHE STRING "" FORCE)
    else()
        set(EXTRA_CONF_FILE "${UPD_BOARD_CONF}" CACHE STRING "" FORCE)
    endif()
endif()

message(STATUS "LR1110 updater board overlay: ${UPDATER_BOARD_OVERLAY}")
message(STATUS "LR1110 updater board conf: ${UPD_BOARD_CONF}")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zephcore_lr1110_updater)

target_sources(app PRIVATE
    src/main.c
    src/lr11xx_hal_updater.c
    src/lr1110_bootloader.c
)

# Include paths for Semtech driver headers and firmware image
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../../adapters/radio/lr11xx
)

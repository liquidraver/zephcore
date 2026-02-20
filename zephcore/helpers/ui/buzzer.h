/*
 * ZephCore - PWM Buzzer with RTTTL Melody Playback
 * Copyright (c) 2025 ZephCore
 * SPDX-License-Identifier: Apache-2.0
 *
 * Non-blocking RTTTL melody playback using Zephyr PWM API.
 * Notes are scheduled via k_work_delayable - no polling needed.
 *
 * Usage:
 *   buzzer_init();                          // Auto-detect from DT aliases
 *   buzzer_play(MELODY_STARTUP);            // Start melody (non-blocking)
 *   buzzer_stop();                          // Stop immediately
 *   buzzer_set_quiet(true);                 // Mute
 *   buzzer_is_playing();                    // Check if melody active
 */

#ifndef ZEPHCORE_BUZZER_H
#define ZEPHCORE_BUZZER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== Predefined RTTTL Melodies ========== */
/* Format: "Name:d=duration,o=octave,b=bpm:notes..." */

#define MELODY_STARTUP     "Startup:d=4,o=5,b=160:16c6,16e6,8g6"
#define MELODY_SHUTDOWN    "Shutdown:d=4,o=5,b=100:8g5,16e5,16c5"
#define MELODY_MSG_CONTACT "MsgRcv3:d=4,o=6,b=200:32e,32g,32b,16c7"
#define MELODY_MSG_CHANNEL "kerplop:d=16,o=6,b=120:32g#,32c#"
#define MELODY_ACK         "ack:d=32,o=8,b=120:c"

/* ========== Public API ========== */

/**
 * Initialize buzzer from devicetree.
 * Looks for 'buzzer' alias in DT → pwm-leds node.
 * Also checks for optional 'buzzer-enable' alias for power gating.
 *
 * @return 0 on success, negative errno on failure, -ENODEV if no buzzer in DT
 */
int buzzer_init(void);

/**
 * Play an RTTTL melody string. Non-blocking - returns immediately.
 * If a melody is already playing, it is stopped first.
 * If buzzer is in quiet mode, this is a no-op.
 *
 * @param rtttl  RTTTL format melody string (must remain valid until done)
 */
void buzzer_play(const char *rtttl);

/**
 * Stop any playing melody immediately and silence the buzzer.
 */
void buzzer_stop(void);

/**
 * Set quiet mode. When quiet, buzzer_play() is a no-op.
 * Also controls the optional enable pin (power gate).
 *
 * @param quiet  true to mute, false to enable
 */
void buzzer_set_quiet(bool quiet);

/**
 * @return true if buzzer is in quiet mode
 */
bool buzzer_is_quiet(void);

/**
 * Set quiet mode without stopping a currently playing melody.
 * Unlike buzzer_set_quiet(), this allows the in-progress melody
 * to finish playing. Future buzzer_play() calls will be suppressed.
 * Useful for "mute" feedback where you want the confirmation sound
 * to play out before silence takes effect.
 *
 * @param quiet  true to mute (after current melody), false to enable
 */
void buzzer_set_quiet_deferred(bool quiet);

/**
 * @return true if a melody is currently playing
 */
bool buzzer_is_playing(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHCORE_BUZZER_H */

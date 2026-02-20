/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore TxtDataHelpers - text message type constants and string helpers
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define TXT_TYPE_PLAIN          0    // a plain text message
#define TXT_TYPE_CLI_DATA       1    // a CLI command
#define TXT_TYPE_SIGNED_PLAIN   2    // plain text, signed by sender

class StrHelper {
public:
	static void strncpy(char *dest, const char *src, size_t buf_sz) {
		if (buf_sz == 0) return;
		size_t i = 0;
		while (i < buf_sz - 1 && src[i] != '\0') {
			dest[i] = src[i];
			i++;
		}
		dest[i] = '\0';
	}

	static void strzcpy(char *dest, const char *src, size_t buf_sz) {
		if (buf_sz == 0) return;
		size_t i = 0;
		while (i < buf_sz - 1 && src[i] != '\0') {
			dest[i] = src[i];
			i++;
		}
		while (i < buf_sz) {
			dest[i++] = '\0';
		}
	}

	static bool isBlank(const char *str) {
		if (str == nullptr) return true;
		while (*str) {
			if (*str != ' ' && *str != '\t' && *str != '\n' && *str != '\r') return false;
			str++;
		}
		return true;
	}
};

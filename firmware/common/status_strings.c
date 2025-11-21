/*
 * status_strings.c - Human-readable status code strings
 *
 * Provides unified error messages for CLI and HTTP API
 * ROM-based lookup table, zero RAM cost
 */

#include "status.h"
#include <stddef.h>

/* Lookup table stored in ROM */
static const char * const status_strings[] = {
	[STATUS_OK]                    = "OK",
	[STATUS_ERROR]                 = "Error",
	[STATUS_ERROR_TIMEOUT]         = "Timeout",
	[STATUS_ERROR_BUSY]            = "Busy",
	[STATUS_ERROR_INVALID_PARAM]   = "Invalid parameter",
	[STATUS_ERROR_INVALID_CONFIG]  = "Configuration out of range",
	[STATUS_ERROR_NOT_INITIALIZED] = "Not initialized",
	[STATUS_ERROR_HARDWARE_FAULT]  = "Hardware fault",
	[STATUS_ERROR_OUT_OF_MEMORY]   = "Out of memory",
	[STATUS_ERROR_BUFFER_FULL]     = "Buffer full",
	[STATUS_ERROR_BUFFER_EMPTY]    = "Buffer empty",
	[STATUS_ERROR_NOT_SUPPORTED]   = "Not supported",
};

#define STATUS_STRINGS_COUNT \
	(sizeof(status_strings) / sizeof(status_strings[0]))

const char *
status_to_string(status_t status)
{
	if ((unsigned int)status >= STATUS_STRINGS_COUNT)
		return "Unknown error";
	
	if (status_strings[status] == NULL)
		return "Unknown error";
	
	return status_strings[status];
}

/*
 * cli_internal.h - Internal CLI definitions for command registry
 */

#ifndef CLI_INTERNAL_H
#define CLI_INTERNAL_H

#include "cli.h"

/*
 * Command handler function type
 */
typedef int (*cli_command_handler_t)(struct cli_context *ctx, int argc, char *argv[]);

/*
 * CLI command structure
 */
struct cli_command {
	const char *name;
	cli_command_handler_t handler;
	const char *help;
};

/*
 * Command registry - defined in cli.c
 */
extern const struct cli_command cli_commands[];
extern const size_t cli_commands_count;

/*
 * Find command by name using binary search
 */
const struct cli_command *
cli_find_command(const char *name);

#endif /* CLI_INTERNAL_H */
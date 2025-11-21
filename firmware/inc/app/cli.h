/*
 * cli.h - Simple command-line interface
 *
 * Minimal CLI for interactive testing and diagnostics
 *
 * Compliant with:
 * - MISRA-C:2012
 * - OpenBSD style guide (no typedefs for structs)
 * - DO-178C Level A requirements
 */

#ifndef CLI_H
#define CLI_H

#include <stdint.h>
#include "status.h"
#include "uart.h"
#include "fdcan.h"
#include "can_simple.h"
#include "valve_haptic.h"

/*
 * CLI configuration
 */
#define CLI_MAX_LINE_LENGTH  80U
#define CLI_MAX_ARGS         8U

/*
 * CLI context structure
 */
struct cli_context {
	struct uart_handle *uart;
	struct fdcan_handle *can;
	struct can_simple_handle *odrive;
	struct valve_context *valve_ctx;
	uint8_t node_id;  /* Current CAN node ID */
	char line_buffer[CLI_MAX_LINE_LENGTH];
	uint8_t line_pos;
};

/*
 * cli_init - Initialize CLI context
 *
 * @ctx: Pointer to CLI context
 * @uart: Pointer to UART handle
 * @can: Pointer to FDCAN handle
 *
 * Returns: STATUS_OK on success
 */
status_t cli_init(struct cli_context *ctx, struct uart_handle *uart,
                  struct fdcan_handle *can);

/*
 * cli_run - Run CLI main loop (never returns)
 *
 * @ctx: Pointer to CLI context
 */
void cli_run(struct cli_context *ctx);

#endif /* CLI_H */

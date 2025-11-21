/*
 * cli_valve_commands.c - Valve-related CLI commands
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "cli_internal.h"
#include "valve_haptic.h"
#include "valve_presets.h"
#include "status.h"
#include "valve_config.h"

/*
 * cli_cmd_valve_start - Start valve control
 */
static int
cli_cmd_valve_start(struct cli_context *ctx, int argc, char *argv[])
{
	status_t status = valve_haptic_start(ctx->valve_ctx);
	if (status == STATUS_OK) {
		uart_write_string(ctx->uart, "\r\nValve started\r\n", 100);
	} else {
		uart_write_string(ctx->uart, "\r\nFailed to start valve\r\n", 100);
		uart_printf(ctx->uart, "Error code: %d\r\n", status);
		
		/* Provide specific error messages */
		if (status == 2) {
			uart_write_string(ctx->uart, "TIMEOUT: Failed to read encoder\r\n", 100);
		} else if (status == 3) {
			uart_write_string(ctx->uart, "ERROR: Valve not in IDLE state\r\n", 100);
		} else if (status == 7) {
			uart_write_string(ctx->uart, "ERROR: Failed to set controller mode (torque control)\r\n", 100);
		} else if (status == 9) {
			uart_write_string(ctx->uart, "ERROR: Odrive reports axis error\r\n", 100);
			uart_write_string(ctx->uart, "Run 'odrive_status' to see error code\r\n", 100);
		} else if (status == 10) {
			uart_write_string(ctx->uart, "ERROR: Odrive not in closed loop state\r\n", 100);
		} else if (status == 11) {
			uart_write_string(ctx->uart, "ERROR: Failed to enable closed loop control\r\n", 100);
		}
	}
	return 0;
}

/*
 * cli_cmd_valve_stop - Stop valve control
 */
static int
cli_cmd_valve_stop(struct cli_context *ctx, int argc, char *argv[])
{
	valve_haptic_stop(ctx->valve_ctx);
	uart_write_string(ctx->uart, "\r\nValve stopped\r\n", 100);
	return 0;
}

/*
 * cli_cmd_valve_damping - Set viscous damping
 */
static int
cli_cmd_valve_damping(struct cli_context *ctx, int argc, char *argv[])
{
	if (argc < 2) {
		uart_write_string(ctx->uart, "\r\nUsage: valve_damping <val>\r\n", 100);
		return 0;
	}
	
	float val = strtof(argv[1], NULL);
	struct valve_config temp_cfg = ctx->valve_ctx->config;
	temp_cfg.hil_b_viscous_nm_s_per_rad = val;
	
	status_t status = valve_haptic_stage_config(ctx->valve_ctx, &temp_cfg, CFG_FIELD_VISCOUS);
	if (status != STATUS_OK) {
		uart_write_string(ctx->uart, "\r\nFailed to stage viscous damping\r\n", 100);
		uart_printf(ctx->uart, "Error: %s\r\n", status_to_string(status));
	} else {
		uart_write_string(ctx->uart, "\r\nViscous damping set\r\n", 100);
	}
	return 0;
}

/*
 * cli_cmd_valve_status - Show valve status
 */
static int
cli_cmd_valve_status(struct cli_context *ctx, int argc, char *argv[])
{
	struct valve_state *state = valve_haptic_get_state(ctx->valve_ctx);
	const char *state_str;
	
	switch (state->status) {
	case VALVE_STATE_IDLE:
		state_str = "IDLE";
		break;
	case VALVE_STATE_INITIALIZING:
		state_str = "INITIALIZING";
		break;
	case VALVE_STATE_RUNNING:
		state_str = "RUNNING";
		break;
	case VALVE_STATE_ERROR:
		state_str = "ERROR";
		break;
	default:
		state_str = "UNKNOWN";
		break;
	}
	
	uart_printf(ctx->uart, "\r\nValve Status:\r\n");
	uart_printf(ctx->uart, "State:              %s\r\n", state_str);
	uart_printf(ctx->uart, "Position:           %.3f deg\r\n", state->position_deg);
	uart_printf(ctx->uart, "Velocity:           %.3f rad/s\r\n", state->omega_rad_s);
	uart_printf(ctx->uart, "Torque:             %.3f N·m\r\n", state->torque_nm);
	uart_printf(ctx->uart, "Torque limit:       %.3f N·m\r\n", ctx->valve_ctx->config.hil_tau_max_limit_nm);
	uart_printf(ctx->uart, "Viscous damping:    %.3f N·m·s/rad\r\n", ctx->valve_ctx->config.hil_b_viscous_nm_s_per_rad);
	uart_printf(ctx->uart, "Coulomb friction:   %.3f N·m\r\n", ctx->valve_ctx->config.hil_tau_c_coulomb_nm);
	uart_printf(ctx->uart, "Wall stiffness:     %.3f N·m/turn\r\n", ctx->valve_ctx->config.hil_k_w_wall_stiffness_nm_per_turn);
	uart_printf(ctx->uart, "Wall damping:       %.3f N·m·s/turn\r\n", ctx->valve_ctx->config.hil_c_w_wall_damping_nm_s_per_turn);
	uart_printf(ctx->uart, "Scale:              %.3f deg/turn\r\n", ctx->valve_ctx->config.degrees_per_turn);
	uart_printf(ctx->uart, "Epsilon:            %.6f\r\n", ctx->valve_ctx->config.hil_eps_smoothing);
	uart_printf(ctx->uart, "Energy tank:        %.6f J\r\n", state->passivity_energy_j);
	uart_printf(ctx->uart, "Loop time:          %lu us\r\n", (unsigned long)state->diag.last_loop_time_us);
	
	return 0;
}

/*
 * Valve command table
 */
const struct cli_command cli_valve_commands[] = {
	{"valve_damping", cli_cmd_valve_damping, "Set viscous damping (N·m·s/rad)"},
	{"valve_start", cli_cmd_valve_start, "Start valve control"},
	{"valve_status", cli_cmd_valve_status, "Show valve status"},
	{"valve_stop", cli_cmd_valve_stop, "Stop valve control"},
};

const size_t cli_valve_commands_count = sizeof(cli_valve_commands) / sizeof(cli_valve_commands[0]);
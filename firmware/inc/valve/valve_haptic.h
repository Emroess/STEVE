/*
 * valve_haptic.h
 *
 * Haptic valve simulation using Odrive S1 as force-feedback actuator.
 * Control loop runs at 1000 Hz with float-based physics calculations.
 *
 * AUTONOMOUS OPERATION:
 * The valve control loop operates entirely independently from the main
 * application once started. It executes deterministically in the TIM6 ISR
 * at exactly 1kHz (1ms period). No periodic service calls are required.
 *
 * Usage:
 *   1. Initialize: valve_haptic_init()
 *   2. Configure: valve_haptic_load_preset(ctx, VALVE_PRESET_MEDIUM, 90.0f)
 *   3. Start: valve_haptic_start()  -> Begins autonomous 1kHz operation
 *   4. Stop: valve_haptic_stop()    -> Halts autonomous operation
 *
 * The control loop processes encoder feedback, calculates physics-based
 * torque commands, and sends them to the ODrive completely independently.
 * The main application can query status but does not need to service the loop.
 */

#ifndef VALVE_HAPTIC_H
#define VALVE_HAPTIC_H

#include <stdint.h>
#include "status.h"
#include "valve_filters.h"
#include "config/valve.h"

/* Valve preset presets - user-friendly resistance levels */
typedef enum {
    VALVE_PRESET_LIGHT = 0,       /* Butterfly/faucet - minimal resistance */
    VALVE_PRESET_MEDIUM = 1,      /* Ball valve - moderate resistance */
    VALVE_PRESET_HEAVY = 2,       /* Gate valve - substantial resistance */
    VALVE_PRESET_INDUSTRIAL = 3   /* Globe/gas main - heavy industrial feel */
} valve_preset_t;

/* Forward declarations */
struct can_simple_handle;

/* Valve simulation states */
typedef enum {
    VALVE_STATE_IDLE = 0,
    VALVE_STATE_INITIALIZING,
    VALVE_STATE_RUNNING,
    VALVE_STATE_ERROR
} valve_state_t;

/* Error types */
typedef enum {
    VALVE_ERROR_NONE = 0,
    VALVE_ERROR_CAN,           /* CAN communication failure */
    VALVE_ERROR_ODRIVE,        /* Odrive reported error */
    VALVE_ERROR_LIMIT          /* Torque or position limit exceeded */
} valve_error_t;

/* Valve configuration (loaded from preset) */
struct valve_config {
    /* Position limits */
    float closed_position_deg; /* Fully closed (typically 0.0) */
    float open_position_deg;   /* Fully open (e.g., 90.0) */
    float degrees_per_turn;    /* Mechanical degrees represented by one encoder turn */

    /* Safety */
    float torque_limit_nm;     /* Symmetric torque limit (±Nm) */

    /* HIL physics parameters (viscous + Coulomb model) */
    float hil_b_viscous_nm_s_per_rad;      /* Viscous damping coefficient b [N·m·s/rad] */
    float hil_tau_c_coulomb_nm;            /* Coulomb friction torque τ_c [N·m] */
    float hil_k_w_wall_stiffness_nm_per_turn; /* Wall stiffness k_w [N·m/turn] */
    float hil_c_w_wall_damping_nm_s_per_turn; /* Wall damping c_w [N·m·s/turn] */
    float hil_eps_smoothing;                /* Smoothing parameter ε for sign function */
    float hil_tau_max_limit_nm;             /* Maximum torque limit τ_max [N·m] */
};

/* Safety event counters for diagnostics */
struct valve_safety_diagnostics {
    uint32_t encoder_stale_events;      /* Count of stale encoder detections */
    uint32_t encoder_timeout_events;    /* Count of encoder timeout events */
    uint32_t axis_error_events;         /* Count of ODrive axis errors */
    uint32_t motor_error_events;        /* Count of motor errors */
    uint32_t encoder_error_events;      /* Count of encoder errors */
    uint32_t torque_discontinuity_events; /* Count of torque discontinuities */
    uint32_t emergency_stops;           /* Count of emergency stops triggered */
    float max_torque_derivative;        /* Maximum torque derivative observed (Nm/ms) */
    uint32_t last_error_code;           /* Last ODrive error code */
    uint32_t last_error_timestamp_ms;   /* Timestamp of last error (ms) */
    /* Temperature monitoring (read-only; limits enforced by ODrive S1) */
    float peak_fet_temperature_c;       /* Peak FET temperature (°C) */
    float peak_motor_temperature_c;     /* Peak motor temperature (°C) */
};

/* Simple diagnostics structure */
struct valve_diagnostics_simple {
    uint32_t loop_count;       /* Total iterations */
    uint32_t telemetry_age_ms; /* Age of encoder telemetry */
    uint32_t heartbeat_age_ms; /* Age of heartbeat telemetry */
    status_t last_can_status;  /* Last CAN operation status */
    uint8_t can_retry_count;   /* CAN retries in current iteration */
    struct valve_safety_diagnostics safety; /* Safety event tracking */
    /* Phase 2: Loop timing diagnostics */
    uint32_t loop_time_min_us;  /* Minimum loop execution time (µs) */
    uint32_t loop_time_max_us;  /* Maximum loop execution time (µs) */
    uint32_t loop_time_sum_us;  /* Sum for average calculation (µs) */
    uint32_t timing_sample_count; /* Number of timing samples */
    /* Streaming timebase (monotonic, microseconds, accumulated from DWT) */
    uint64_t t_us_accum;         /* Monotonic timestamp (µs) for stream alignment */
    uint32_t last_loop_time_us;  /* Instantaneous loop time (µs) for this iteration */
    uint32_t sample_seq;         /* Monotonic sample sequence counter */
    /* Encoder data age statistics */
    uint32_t encoder_age_min_us;  /* Minimum encoder data age (µs) */
    uint32_t encoder_age_max_us;  /* Maximum encoder data age (µs) */
    uint32_t encoder_age_sum_us;  /* Sum for median/average calculation (µs) */
    uint32_t encoder_age_count;   /* Number of encoder age samples */
};

/* Simplified unified valve state */
struct valve_state {
    float position_deg;        /* Current position (degrees) */
    float omega_rad_s;         /* Current angular velocity (rad/s) */
    float alpha_rad_s2;        /* Current angular acceleration (rad/s²) for feedforward */
    float prev_omega_rad_s;    /* Previous angular velocity for acceleration calculation */
    float command_position_deg;/* Legacy command field; mirrors measured position */
    float raw_position_turns;  /* Latest encoder report (turns) */
    float degrees_per_turn;    /* Runtime conversion scale */
    float torque_nm;           /* Current applied torque (Nm) */
    float previous_torque_nm;  /* Previous torque for rate limiting (Nm) */
    float filtered_torque_nm;  /* Low-pass filtered torque output (Nm) */
    float passivity_energy_j;  /* Energy tank for passivity guard (J) */
    uint8_t status;            /* State/status flags */
    uint8_t in_emergency_stop; /* Emergency stop flag (0=normal, 1=stopped) */
    uint8_t quiet_active;      /* 1 when quiet-at-rest gate is suppressing torque */
    struct valve_diagnostics_simple diag; /* Basic diagnostics */
    struct can_simple_handle *odrive;     /* Odrive handle for CAN communication */
    float encoder_zero_turns;  /* Encoder zero reference */
};

/* Valve context (combines simplified state and config) */
struct valve_context {
    struct valve_state state;
    struct valve_config config;
    struct valve_config staged_config;
    uint32_t staged_field_mask;
    volatile uint8_t staged_pending;
    /* Autonomous operation: control loop executes directly in TIM6 ISR */
};

/* Public API */
status_t valve_haptic_init(struct valve_context *ctx, struct can_simple_handle *odrive);
status_t valve_haptic_load_preset(struct valve_context *ctx, valve_preset_t preset, float travel_degrees);
status_t valve_haptic_start(struct valve_context *ctx);  /* Start autonomous 1kHz control loop */
void valve_haptic_stop(struct valve_context *ctx);       /* Stop autonomous control loop */
void valve_haptic_process(struct valve_context *ctx);    /* Internal: called by TIM6 ISR */
void valve_haptic_timer_isr(void);                       /* Internal: TIM6 ISR callback */
void TIM6_DAC_IRQHandler(void);                          /* TIM6 interrupt handler */
struct valve_state *valve_haptic_get_state(struct valve_context *ctx);
struct valve_config *valve_haptic_get_config(struct valve_context *ctx);
struct valve_context *valve_haptic_get_context(void);
status_t valve_haptic_stage_config(struct valve_context *ctx, const struct valve_config *cfg, uint32_t field_mask);
float valve_haptic_calc_settling_time_ms(void);          /* Calculate settling time from perfmon data */
status_t valve_haptic_get_loop_timing(struct valve_context *ctx, uint32_t *min_us, uint32_t *avg_us, uint32_t *max_us); /* Get timing statistics */

#endif /* VALVE_HAPTIC_H */

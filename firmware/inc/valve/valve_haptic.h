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
    uint32_t overtemp_events;           /* Count of overtemperature events */
    uint32_t torque_discontinuity_events; /* Count of torque discontinuities */
    uint32_t emergency_stops;           /* Count of emergency stops triggered */
    float max_torque_derivative;        /* Maximum torque derivative observed (Nm/ms) */
    uint32_t last_error_code;           /* Last ODrive error code */
    uint32_t last_error_timestamp_ms;   /* Timestamp of last error (ms) */
    /* Phase 2: Performance and monitoring */
    uint32_t saturation_events;         /* Count of torque saturation events */
    uint32_t collision_events;          /* Count of collision detections */
    uint32_t thermal_throttle_events;   /* Count of thermal throttling events */
    float peak_iq_measured;             /* Peak |Iq_measured| observed (A) */
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
    float startup_ramp_progress; /* Startup ramp progress [0.0, 1.0] */
    uint32_t startup_start_ms;  /* Timestamp when startup began (ms) */
    uint8_t status;            /* State/status flags */
    uint8_t in_emergency_stop; /* Emergency stop flag (0=normal, 1=stopped) */
    uint8_t thermal_throttle_active; /* Thermal throttling flag */
    uint8_t quiet_active;      /* 1 when quiet-at-rest gate is suppressing torque */
    float torque_limit_scale;  /* Dynamic torque limit scaling [0.0, 1.0] */
    /* Phase 2: Saturation and collision detection state */
    float last_iq_setpoint;    /* Previous Iq setpoint for saturation detection */
    uint32_t saturation_start_ms; /* Timestamp when saturation first detected */
    struct valve_diagnostics_simple diag; /* Basic diagnostics */
    struct can_simple_handle *odrive;     /* Odrive handle for CAN communication */
    float encoder_zero_turns;  /* Encoder zero reference */
};

/* Control loop configuration */
#define VALVE_CONTROL_LOOP_HZ          1000U
#define VALVE_CONTROL_LOOP_PERIOD_S    (1.0f / (float)VALVE_CONTROL_LOOP_HZ)
#define VALVE_CONTROL_LOOP_PERIOD_MS   (1000U / VALVE_CONTROL_LOOP_HZ)
#define VALVE_CONTROL_LOOP_PERIOD_US   (1000000U / VALVE_CONTROL_LOOP_HZ)
#define VALVE_LOOP_DT_S                VALVE_CONTROL_LOOP_PERIOD_S  /* Fixed timing assumption */

/* TIM6 timer configuration */
#define TIM6_PRESCALER                 199U    /* 200MHz -> 1MHz */
#define TIM6_BASE_FREQUENCY_HZ         1000000U /* 1 MHz after prescaling */

/* Safety limits */
#define VALVE_MAX_POSITION_DEG         3600.0f  /* 10 turns maximum position */

/* Mathematical constants */
#define VALVE_TWO_PI                   6.28318530718f  /* 2 * PI */
#define VALVE_TWO_PI_APPROX            6.28f           /* Approximate 2 * PI for calculations */
#define VALVE_DEFAULT_DEGREES_PER_TURN 360.0f          /* Default: 1 encoder turn = 360 valve degrees */

/* Hardware constants */
#define VALVE_CPU_CLOCK_MHZ            400U            /* CPU clock in MHz */

/* Control constants */
#define VALVE_STOP_BOUNCE_THRESHOLD_DPS 10.0f          /* Velocity threshold for bounce detection */
#define VALVE_STARTUP_RATE_LIMIT_NM_PER_S 20.0f        /* Reduced rate limit during startup */
#define VALVE_STARTUP_TORQUE_LIMIT_NM    2.0f          /* Initial torque limit during startup ramp (Nm) */
#define VALVE_STARTUP_HOLD_TIME_MS       200U          /* Initial hold period for encoder stability (ms) */
#define VALVE_STARTUP_RAMP_TIME_MS       500U          /* Torque ramp duration after hold (ms) */
#define VALVE_TORQUE_LIMIT_MARGIN        1.1f          /* Safety margin for torque limit checks */
#define VALVE_VELOCITY_DEADZONE_DPS      0.1f          /* Velocity below this is considered zero */

/* Safety thresholds for Phase 1 implementation */
#define VALVE_ENCODER_STALE_THRESHOLD_MS  10U          /* Critical: encoder data >10ms old triggers E-stop */
#define VALVE_ENCODER_WARNING_THRESHOLD_MS 5U          /* Warning: encoder data >5ms old */
#define VALVE_TORQUE_DERIVATIVE_LIMIT_NM_PER_MS 0.05f  /* Max allowed torque derivative (~50 Nm/s) */
#define VALVE_TORQUE_DISCONTINUITY_THRESHOLD 0.1f      /* Discontinuity detection threshold (Nm) */
#define VALVE_TORQUE_FILTER_CUTOFF_HZ        400.0f     /* Torque smoothing low-pass cutoff */
#define VALVE_PASSIVITY_ENERGY_CAP_J         2.0f      /* Max stored passive energy (|J|) */

/* Phase 2: Torque observer and thermal protection thresholds */
#define VALVE_IQ_SATURATION_THRESHOLD_A    2.0f         /* |Iq_measured - Iq_setpoint| > 2A indicates saturation */
#define VALVE_IQ_SATURATION_DURATION_MS    50U          /* Saturation must persist for 50ms */
#define VALVE_IQ_SETPOINT_MIN_FOR_SAT_A    5.0f         /* Only check saturation if |Iq_setpoint| > 5A */
#define VALVE_IQ_COLLISION_THRESHOLD_A     8.0f         /* |Iq_measured| > 8A indicates collision */
#define VALVE_IQ_COLLISION_SETPOINT_MAX_A  3.0f         /* Collision if |Iq_setpoint| < 3A */
#define VALVE_TEMP_FET_WARNING_C           72.0f        /* FET temperature warning threshold */
#define VALVE_TEMP_MOTOR_WARNING_C         62.0f        /* Motor temperature warning threshold */
#define VALVE_TEMP_FET_CRITICAL_C          85.0f        /* FET temperature critical threshold */
#define VALVE_TEMP_MOTOR_CRITICAL_C        75.0f        /* Motor temperature critical threshold */
#define VALVE_THERMAL_THROTTLE_SCALE       0.75f        /* Torque scale during thermal throttling */

/* Phase 2: Feedforward compensation */
#define VALVE_INERTIA_KGM2                 0.00010f     /* Reduced system inertia for stability (kg·m²) */
#define VALVE_DEG_TO_RAD                   0.0174533f   /* Conversion: degrees to radians */
#define VALVE_RAD_TO_DEG                   (1.0f / VALVE_DEG_TO_RAD)

/* Diagnostic constants */
#define VALVE_SETTLING_TIME_FACTOR       5.0f           /* Factor for settling time calculation */
#define VALVE_MIN_SAMPLES_FOR_STABILITY  100U           /* Minimum samples for stability analysis */
#define VALVE_MIN_VEL_PEAK_FOR_STABILITY 5.0f           /* Minimum velocity peak for stability analysis */

/* Safety limits */
#define VALVE_MAX_TORQUE_LIMIT_NM      30.0f           /* Maximum allowed torque limit */
#define VALVE_MAX_STOP_CUSHION_DEG     45.0f           /* Maximum stop cushion angle */
#define VALVE_ODRIVE_VEL_LIMIT_TURNS_PER_S 8.0f        /* ODrive velocity limit (turns/s) */
#define VALVE_ODRIVE_CURRENT_LIMIT_A    10.0f          /* ODrive current limit (amps) */

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

/*
 * perfmon.h - Lightweight on-device performance monitor
 */

#ifndef PERFMON_H
#define PERFMON_H

#include <stdint.h>

#define PM_CAP 1024U

#if (PM_CAP & (PM_CAP - 1U)) != 0
#error "PM_CAP must be a power of two for ring buffer masking"
#endif

#define PERF_AGE_INVALID 0xFFFFU

#define PERF_GUARD_TORQUE_CLAMP  0x01U
#define PERF_GUARD_RATE_LIMIT    0x02U
#define PERF_GUARD_STABILITY     0x04U
#define PERF_GUARD_OVERRUN       0x08U

struct perf_sample {
	float pos_deg;
	float vel_rad_s;
	float tau_nm;
	float passivity_energy_j;
	uint16_t telemetry_age_ms;
	uint16_t heartbeat_age_ms;
	uint16_t total_us;
	uint16_t loop_period_us;
	uint16_t physics_us;
	uint16_t can_tx_us;
	uint8_t guard_flags;
	uint8_t reserved;
	uint32_t telemetry_seq;
	uint32_t tx_dropped_frames;
};

struct perfmon_timing_stats {
	uint16_t total_min_us;
	uint16_t total_max_us;
	float total_mean_us;
	uint16_t physics_min_us;
	uint16_t physics_max_us;
	float physics_mean_us;
	uint16_t can_tx_min_us;
	uint16_t can_tx_max_us;
	float can_tx_mean_us;
	uint16_t jitter_min_us;
	uint16_t jitter_max_us;
	float jitter_mean_us;
};

struct perfmon_transport_stats {
	uint16_t telemetry_age_min_ms;
	uint16_t telemetry_age_max_ms;
	float telemetry_age_mean_ms;
	uint16_t heartbeat_age_min_ms;
	uint16_t heartbeat_age_max_ms;
	float heartbeat_age_mean_ms;
	uint32_t telemetry_gap_events;
	uint32_t telemetry_gap_frames;
	uint32_t tx_dropped_frames;
};

struct perfmon_motion_stats {
	float pos_rms;
	float vel_rms;
	float tau_rms;
	float pos_peak_to_peak;
	float vel_peak_to_peak;
	float zero_cross_rate_hz;
};

struct perfmon_guard_stats {
	uint32_t torque_clamps;
	uint32_t rate_limits;
	uint32_t stability_events;
};

struct perfmon_snapshot {
	uint16_t sample_count;
	struct perfmon_timing_stats timing;
	struct perfmon_transport_stats transport;
	struct perfmon_motion_stats motion;
	struct perfmon_guard_stats guard;
};

void perfmon_init(void);
void perfmon_set_budget_us(uint16_t budget_us);
void perfmon_push(float pos_deg, float vel_rad_s, float tau_nm, float passivity_energy_j,
    uint16_t telemetry_age_ms, uint16_t total_us, uint16_t loop_period_us,
    uint16_t physics_us, uint16_t can_tx_us,
    uint16_t heartbeat_age_ms, uint32_t telemetry_seq,
    uint32_t tx_dropped_frames, uint8_t guard_flags);
void perfmon_stats(float *pos_rms, float *vel_rms, float *tau_rms);
void perfmon_snapshot(struct perfmon_snapshot *snapshot);
void perfmon_guard_event(uint8_t guard_flags);
uint16_t perfmon_get_fill_count(void);
uint32_t perfmon_get_write_index(void);
const struct perf_sample *perfmon_get_sample(uint32_t index);

#endif /* PERFMON_H */

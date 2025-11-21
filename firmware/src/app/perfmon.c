/*
 * perfmon.c - Lightweight on-device performance monitor
 */

#include "perfmon.h"
#include "arm_math.h"
#include <stddef.h>
#include <string.h>

#define PERF_ZERO_EPS        0.05f
#define PERF_USEC_PER_SEC    1000000.0f

static struct perf_sample perf_buf[PM_CAP];
static uint32_t perf_wr;
static uint16_t perf_fill;
static uint16_t perf_budget_us;

static struct perfmon_guard_stats guard_extra;

static void perfmon_accumulate_motion(const struct perf_sample *s,
    double *pos_sq, double *vel_sq, double *tau_sq,
    float *pos_min, float *pos_max, float *vel_min, float *vel_max,
    uint8_t *have_motion);

static void perfmon_accumulate_transport(const struct perf_sample *s,
    uint16_t *tele_min, uint16_t *tele_max, uint64_t *tele_sum,
    uint32_t *tele_samples, uint16_t *hb_min, uint16_t *hb_max,
    uint64_t *hb_sum, uint32_t *hb_samples,
    uint32_t *gap_events, uint32_t *gap_frames,
    uint32_t *tx_drops, uint8_t *have_tx,
    uint32_t *prev_seq, uint8_t *seq_valid, uint32_t *prev_tx);

static void perfmon_accumulate_timing(const struct perf_sample *s,
    uint16_t *tot_min, uint16_t *tot_max, double *tot_sum,
    uint16_t *phys_min, uint16_t *phys_max, double *phys_sum,
    uint16_t *can_min, uint16_t *can_max, double *can_sum,
    uint16_t *jitter_min, uint16_t *jitter_max, double *jitter_sum,
    uint32_t *jitter_samples);

static void perfmon_accumulate_guards(const struct perf_sample *s,
    uint32_t *torque, uint32_t *rate, uint32_t *stability);

void
perfmon_init(void)
{
	uint32_t i;

	perf_wr = 0U;
	perf_fill = 0U;
	perf_budget_us = 0U;
	guard_extra.torque_clamps = 0U;
	guard_extra.rate_limits = 0U;
	guard_extra.stability_events = 0U;

	for (i = 0U; i < PM_CAP; i++) {
		perf_buf[i].pos_deg = 0.0f;
		perf_buf[i].vel_rad_s = 0.0f;
		perf_buf[i].tau_nm = 0.0f;
		perf_buf[i].passivity_energy_j = 0.0f;
		perf_buf[i].telemetry_age_ms = PERF_AGE_INVALID;
		perf_buf[i].heartbeat_age_ms = PERF_AGE_INVALID;
		perf_buf[i].total_us = 0U;
		perf_buf[i].loop_period_us = 0U;
		perf_buf[i].physics_us = 0U;
		perf_buf[i].can_tx_us = 0U;
		perf_buf[i].guard_flags = 0U;
		perf_buf[i].reserved = 0U;
		perf_buf[i].telemetry_seq = 0U;
		perf_buf[i].tx_dropped_frames = 0U;
	}
}

void
perfmon_set_budget_us(uint16_t budget_us)
{
	perf_budget_us = budget_us;
}

void
perfmon_guard_event(uint8_t guard_flags)
{
	if ((guard_flags & PERF_GUARD_TORQUE_CLAMP) != 0U) {
		guard_extra.torque_clamps++;
	}
	if ((guard_flags & PERF_GUARD_RATE_LIMIT) != 0U) {
		guard_extra.rate_limits++;
	}
	if ((guard_flags & PERF_GUARD_STABILITY) != 0U) {
		guard_extra.stability_events++;
	}
}

void
perfmon_push(float pos_deg, float vel_rad_s, float tau_nm, float passivity_energy_j,
    uint16_t telemetry_age_ms, uint16_t total_us, uint16_t loop_period_us,
    uint16_t physics_us, uint16_t can_tx_us,
    uint16_t heartbeat_age_ms, uint32_t telemetry_seq,
    uint32_t tx_dropped_frames, uint8_t guard_flags)
{
	uint32_t idx;
	struct perf_sample *sample;

	idx = perf_wr & (PM_CAP - 1U);
	perf_wr++;
	if (perf_fill < PM_CAP) {
		perf_fill++;
	}

	sample = &perf_buf[idx];
	sample->pos_deg = pos_deg;
	sample->vel_rad_s = vel_rad_s;
	sample->tau_nm = tau_nm;
	sample->passivity_energy_j = passivity_energy_j;
	sample->telemetry_age_ms = telemetry_age_ms;
	sample->heartbeat_age_ms = heartbeat_age_ms;
	sample->total_us = total_us;
	sample->loop_period_us = loop_period_us;
	sample->physics_us = physics_us;
	sample->can_tx_us = can_tx_us;
	sample->telemetry_seq = telemetry_seq;
	sample->tx_dropped_frames = tx_dropped_frames;
	sample->guard_flags = guard_flags;
}

void
perfmon_stats(float *pos_rms, float *vel_rms, float *tau_rms)
{
	struct perfmon_snapshot snap;

	if (pos_rms == NULL || vel_rms == NULL || tau_rms == NULL) {
		return;
	}

	perfmon_snapshot(&snap);
	*pos_rms = snap.motion.pos_rms;
	*vel_rms = snap.motion.vel_rms;
	*tau_rms = snap.motion.tau_rms;
}

void
perfmon_snapshot(struct perfmon_snapshot *snapshot)
{
	uint16_t count;
	uint32_t i;
	uint32_t idx;
	uint32_t start;
	float pos_min;
	float pos_max;
	float vel_min;
	float vel_max;
	double pos_sq;
	double vel_sq;
	double tau_sq;
	uint8_t have_motion;
	uint16_t tele_min;
	uint16_t tele_max;
	uint64_t tele_sum;
	uint32_t tele_samples;
	uint16_t hb_min;
	uint16_t hb_max;
	uint64_t hb_sum;
	uint32_t hb_samples;
	uint32_t gap_events;
	uint32_t gap_frames;
	uint32_t tx_drops;
	uint32_t prev_seq;
	uint32_t prev_tx;
	uint8_t seq_valid;
	uint8_t have_tx;
	uint16_t tot_min;
	uint16_t tot_max;
	double tot_sum;
	uint16_t phys_min;
	uint16_t phys_max;
	double phys_sum;
	uint16_t can_min;
	uint16_t can_max;
	double can_sum;
	uint16_t jitter_min;
	uint16_t jitter_max;
	double jitter_sum;
	uint32_t jitter_samples;
	uint32_t guard_torque;
	uint32_t guard_rate;
	uint32_t guard_stability;
	float zero_cross_count;
	float zero_cross_rate;
	double window_us;
	int prev_sign;

	if (snapshot == NULL) {
		return;
	}

	memset(snapshot, 0, sizeof(*snapshot));

	count = perf_fill;
	snapshot->sample_count = count;
	if (count == 0U) {
		snapshot->transport.telemetry_age_min_ms = PERF_AGE_INVALID;
		snapshot->transport.telemetry_age_max_ms = 0U;
		snapshot->transport.heartbeat_age_min_ms = PERF_AGE_INVALID;
		snapshot->transport.heartbeat_age_max_ms = 0U;
		return;
	}

	start = (perf_wr - (uint32_t)count);
	pos_sq = 0.0;
	vel_sq = 0.0;
	tau_sq = 0.0;
	have_motion = 0U;
	pos_min = 0.0f;
	pos_max = 0.0f;
	vel_min = 0.0f;
	vel_max = 0.0f;

	tele_min = PERF_AGE_INVALID;
	tele_max = 0U;
	tele_sum = 0U;
	tele_samples = 0U;
	hb_min = PERF_AGE_INVALID;
	hb_max = 0U;
	hb_sum = 0U;
	hb_samples = 0U;
	gap_events = 0U;
	gap_frames = 0U;
	tx_drops = 0U;
	prev_seq = 0U;
	prev_tx = 0U;
	seq_valid = 0U;
	have_tx = 0U;

	tot_min = UINT16_MAX;
	tot_max = 0U;
	tot_sum = 0.0;
	phys_min = UINT16_MAX;
	phys_max = 0U;
	phys_sum = 0.0;
	can_min = UINT16_MAX;
	can_max = 0U;
	can_sum = 0.0;
	jitter_min = UINT16_MAX;
	jitter_max = 0U;
	jitter_sum = 0.0;
	jitter_samples = 0U;

	guard_torque = 0U;
	guard_rate = 0U;
	guard_stability = 0U;
	zero_cross_count = 0.0f;
	window_us = 0.0;
	prev_sign = 0;

	for (i = 0U; i < count; i++) {
		idx = (start + i) & (PM_CAP - 1U);
		perfmon_accumulate_motion(&perf_buf[idx], &pos_sq, &vel_sq, &tau_sq,
		    &pos_min, &pos_max, &vel_min, &vel_max, &have_motion);
		perfmon_accumulate_transport(&perf_buf[idx], &tele_min, &tele_max,
		    &tele_sum, &tele_samples, &hb_min, &hb_max, &hb_sum,
		    &hb_samples, &gap_events, &gap_frames, &tx_drops, &have_tx,
		    &prev_seq, &seq_valid, &prev_tx);
		perfmon_accumulate_timing(&perf_buf[idx], &tot_min, &tot_max, &tot_sum,
		    &phys_min, &phys_max, &phys_sum, &can_min, &can_max, &can_sum,
		    &jitter_min, &jitter_max, &jitter_sum, &jitter_samples);
		perfmon_accumulate_guards(&perf_buf[idx], &guard_torque, &guard_rate,
		    &guard_stability);
		{
			uint16_t sample_period = perf_buf[idx].loop_period_us;

			if (sample_period == 0U && perf_budget_us != 0U) {
				sample_period = perf_budget_us;
			}
			window_us += (double)sample_period;
		}

		if (have_motion != 0U) {
			float vel = perf_buf[idx].vel_rad_s;
			int sign;

			if (vel > PERF_ZERO_EPS) {
				sign = 1;
			} else if (vel < -PERF_ZERO_EPS) {
				sign = -1;
			} else {
				sign = 0;
			}

			if (sign != 0 && prev_sign != 0 && sign != prev_sign) {
				zero_cross_count += 1.0f;
			}
			if (sign != 0) {
				prev_sign = sign;
			}
		}
	}

	if (have_motion != 0U && count > 0U) {
		float pos_rms_val, vel_rms_val, tau_rms_val;
		arm_status pos_status, vel_status, tau_status;

		pos_status = arm_sqrt_f32((float)(pos_sq / (double)count), &pos_rms_val);
		vel_status = arm_sqrt_f32((float)(vel_sq / (double)count), &vel_rms_val);
		tau_status = arm_sqrt_f32((float)(tau_sq / (double)count), &tau_rms_val);

		if (pos_status == ARM_MATH_SUCCESS) {
			snapshot->motion.pos_rms = pos_rms_val;
		}
		if (vel_status == ARM_MATH_SUCCESS) {
			snapshot->motion.vel_rms = vel_rms_val;
		}
		if (tau_status == ARM_MATH_SUCCESS) {
			snapshot->motion.tau_rms = tau_rms_val;
		}

		snapshot->motion.pos_peak_to_peak = pos_max - pos_min;
		snapshot->motion.vel_peak_to_peak = vel_max - vel_min;
	}

	if (jitter_samples == 0U) {
		jitter_min = 0U;
		jitter_max = 0U;
	}

	snapshot->timing.total_min_us = (tot_min == UINT16_MAX) ? 0U : tot_min;
	snapshot->timing.total_max_us = tot_max;
	snapshot->timing.total_mean_us = (count > 0U) ?
	    (float)(tot_sum / (double)count) : 0.0f;
	snapshot->timing.physics_min_us = (phys_min == UINT16_MAX) ? 0U : phys_min;
	snapshot->timing.physics_max_us = phys_max;
	snapshot->timing.physics_mean_us = (count > 0U) ?
	    (float)(phys_sum / (double)count) : 0.0f;
	snapshot->timing.can_tx_min_us = (can_min == UINT16_MAX) ? 0U : can_min;
	snapshot->timing.can_tx_max_us = can_max;
	snapshot->timing.can_tx_mean_us = (count > 0U) ?
	    (float)(can_sum / (double)count) : 0.0f;
	snapshot->timing.jitter_min_us = jitter_min;
	snapshot->timing.jitter_max_us = jitter_max;
	snapshot->timing.jitter_mean_us = (jitter_samples > 0U) ?
	    (float)(jitter_sum / (double)jitter_samples) : 0.0f;

	if (tele_samples == 0U) {
		tele_min = PERF_AGE_INVALID;
		tele_max = 0U;
	}
	if (hb_samples == 0U) {
		hb_min = PERF_AGE_INVALID;
		hb_max = 0U;
	}

	snapshot->transport.telemetry_age_min_ms = tele_min;
	snapshot->transport.telemetry_age_max_ms = tele_max;
	snapshot->transport.telemetry_age_mean_ms = (tele_samples > 0U) ?
	    (float)((double)tele_sum / (double)tele_samples) : 0.0f;
	snapshot->transport.heartbeat_age_min_ms = hb_min;
	snapshot->transport.heartbeat_age_max_ms = hb_max;
	snapshot->transport.heartbeat_age_mean_ms = (hb_samples > 0U) ?
	    (float)((double)hb_sum / (double)hb_samples) : 0.0f;
	snapshot->transport.telemetry_gap_events = gap_events;
	snapshot->transport.telemetry_gap_frames = gap_frames;
	snapshot->transport.tx_dropped_frames = tx_drops;

	snapshot->guard.torque_clamps = guard_torque + guard_extra.torque_clamps;
	snapshot->guard.rate_limits = guard_rate + guard_extra.rate_limits;
	snapshot->guard.stability_events = guard_stability +
	    guard_extra.stability_events;

	if (window_us <= 0.0) {
		zero_cross_rate = 0.0f;
	} else {
		zero_cross_rate = zero_cross_count *
		    (float)(PERF_USEC_PER_SEC / window_us);
	}
	snapshot->motion.zero_cross_rate_hz = zero_cross_rate;
}

static void
perfmon_accumulate_motion(const struct perf_sample *s,
    double *pos_sq, double *vel_sq, double *tau_sq,
    float *pos_min, float *pos_max, float *vel_min, float *vel_max,
    uint8_t *have_motion)
{
	if (*have_motion == 0U) {
		*pos_min = s->pos_deg;
		*pos_max = s->pos_deg;
		*vel_min = s->vel_rad_s;
		*vel_max = s->vel_rad_s;
		*have_motion = 1U;
	}

	*pos_sq += (double)s->pos_deg * (double)s->pos_deg;
	*vel_sq += (double)s->vel_rad_s * (double)s->vel_rad_s;
	*tau_sq += (double)s->tau_nm * (double)s->tau_nm;

	if (s->pos_deg < *pos_min) {
		*pos_min = s->pos_deg;
	} else if (s->pos_deg > *pos_max) {
		*pos_max = s->pos_deg;
	}

	if (s->vel_rad_s < *vel_min) {
		*vel_min = s->vel_rad_s;
	} else if (s->vel_rad_s > *vel_max) {
		*vel_max = s->vel_rad_s;
	}
}

static void
perfmon_accumulate_transport(const struct perf_sample *s,
    uint16_t *tele_min, uint16_t *tele_max, uint64_t *tele_sum,
    uint32_t *tele_samples, uint16_t *hb_min, uint16_t *hb_max,
    uint64_t *hb_sum, uint32_t *hb_samples,
    uint32_t *gap_events, uint32_t *gap_frames,
    uint32_t *tx_drops, uint8_t *have_tx,
    uint32_t *prev_seq, uint8_t *seq_valid, uint32_t *prev_tx)
{
	if (s->telemetry_age_ms != PERF_AGE_INVALID) {
		if (*tele_samples == 0U || s->telemetry_age_ms < *tele_min) {
			*tele_min = s->telemetry_age_ms;
		}
		if (*tele_samples == 0U || s->telemetry_age_ms > *tele_max) {
			*tele_max = s->telemetry_age_ms;
		}
		*tele_sum += s->telemetry_age_ms;
		(*tele_samples)++;
	}

	if (s->heartbeat_age_ms != PERF_AGE_INVALID) {
		if (*hb_samples == 0U || s->heartbeat_age_ms < *hb_min) {
			*hb_min = s->heartbeat_age_ms;
		}
		if (*hb_samples == 0U || s->heartbeat_age_ms > *hb_max) {
			*hb_max = s->heartbeat_age_ms;
		}
		*hb_sum += s->heartbeat_age_ms;
		(*hb_samples)++;
	}

	if (*seq_valid != 0U && s->telemetry_seq > *prev_seq + 1U) {
		uint32_t delta;

		delta = s->telemetry_seq - *prev_seq - 1U;
		*gap_frames += delta;
		(*gap_events)++;
	}
	if (s->telemetry_seq != 0U) {
		*prev_seq = s->telemetry_seq;
		*seq_valid = 1U;
	}

	if (*have_tx != 0U) {
		if (s->tx_dropped_frames >= *prev_tx) {
			*tx_drops += (s->tx_dropped_frames - *prev_tx);
		} else {
			*tx_drops += s->tx_dropped_frames;
		}
	} else {
		*have_tx = 1U;
	}
	*prev_tx = s->tx_dropped_frames;
}

static void
perfmon_accumulate_timing(const struct perf_sample *s,
    uint16_t *tot_min, uint16_t *tot_max, double *tot_sum,
    uint16_t *phys_min, uint16_t *phys_max, double *phys_sum,
    uint16_t *can_min, uint16_t *can_max, double *can_sum,
    uint16_t *jitter_min, uint16_t *jitter_max, double *jitter_sum,
    uint32_t *jitter_samples)
{
	uint16_t jitter;
	uint16_t total;
	uint16_t physics;
	uint16_t can_tx;
	uint16_t period;

	total = s->total_us;
	physics = s->physics_us;
	can_tx = s->can_tx_us;
	period = s->loop_period_us;

	if (total < *tot_min) {
		*tot_min = total;
	}
	if (total > *tot_max) {
		*tot_max = total;
	}
	*tot_sum += (double)total;

	if (physics < *phys_min) {
		*phys_min = physics;
	}
	if (physics > *phys_max) {
		*phys_max = physics;
	}
	*phys_sum += (double)physics;

	if (can_tx < *can_min) {
		*can_min = can_tx;
	}
	if (can_tx > *can_max) {
		*can_max = can_tx;
	}
	*can_sum += (double)can_tx;

	if (period == 0U && perf_budget_us != 0U) {
		period = perf_budget_us;
	}
	if (perf_budget_us != 0U && period != 0U) {
		if (period >= perf_budget_us) {
			jitter = (uint16_t)(period - perf_budget_us);
		} else {
			jitter = (uint16_t)(perf_budget_us - period);
		}
		if (jitter < *jitter_min) {
			*jitter_min = jitter;
		}
		if (jitter > *jitter_max) {
			*jitter_max = jitter;
		}
		*jitter_sum += (double)jitter;
		(*jitter_samples)++;
	}
}

static void
perfmon_accumulate_guards(const struct perf_sample *s,
    uint32_t *torque, uint32_t *rate, uint32_t *stability)
{
	if ((s->guard_flags & PERF_GUARD_TORQUE_CLAMP) != 0U) {
		(*torque)++;
	}
	if ((s->guard_flags & PERF_GUARD_RATE_LIMIT) != 0U) {
		(*rate)++;
	}
	if ((s->guard_flags & PERF_GUARD_STABILITY) != 0U) {
		(*stability)++;
	}
}

uint16_t
perfmon_get_fill_count(void)
{
	return perf_fill;
}

uint32_t
perfmon_get_write_index(void)
{
	return perf_wr;
}

const struct perf_sample *
perfmon_get_sample(uint32_t index)
{
	if (index >= PM_CAP) {
		return NULL;
	}
	return &perf_buf[index];
}

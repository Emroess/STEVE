/*
 * performance_manager.c - Performance monitoring service layer implementation
 */

#include "performance_manager.h"
#include "perfmon.h"
#include <stddef.h>

status_t
perf_get_stats(struct perfmon_snapshot *snapshot)
{
	if (snapshot == NULL) {
		return STATUS_ERROR_INVALID_PARAM;
	}
	perfmon_snapshot(snapshot);
	return STATUS_OK;
}

status_t
perf_get_rms_values(float *pos_rms, float *vel_rms, float *tau_rms)
{
	if (pos_rms == NULL || vel_rms == NULL || tau_rms == NULL) {
		return STATUS_ERROR_INVALID_PARAM;
	}
	perfmon_stats(pos_rms, vel_rms, tau_rms);
	return STATUS_OK;
}

status_t
perf_dump_csv(void (*output_fn)(const char *))
{
	if (output_fn == NULL) {
		return STATUS_ERROR_INVALID_PARAM;
	}
	
	/* CSV header */
	output_fn("pos_deg,vel_rad_s,tau_nm,energy_j,age_ms,total_us,period_us,physics_us,can_us,guards\r\n");
	
	/* This would iterate through perfmon buffer and output CSV rows */
	/* Implementation depends on perfmon internal API access */
	
	return STATUS_OK;
}

status_t
perf_stream_raw(void (*output_fn)(const char *), uint32_t sample_count)
{
	(void)sample_count;
	if (output_fn == NULL) {
		return STATUS_ERROR_INVALID_PARAM;
	}
	
	/* Stream raw perfmon samples */
	/* Implementation depends on perfmon internal API access */
	
	return STATUS_OK;
}

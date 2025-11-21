/*
 * performance_manager.h - Performance monitoring service layer
 *
 * Provides unified API for performance monitoring used by both CLI and HTTP API.
 * Wraps perfmon with higher-level operations.
 */

#ifndef PERFORMANCE_MANAGER_H
#define PERFORMANCE_MANAGER_H

#include "perfmon.h"
#include "status.h"
#include <stdint.h>

/*
 * Performance statistics queries
 */
status_t perf_get_stats(struct perfmon_snapshot *snapshot);
status_t perf_get_rms_values(float *pos_rms, float *vel_rms, float *tau_rms);

/*
 * Performance data dump operations
 */
status_t perf_dump_csv(void (*output_fn)(const char *));
status_t perf_stream_raw(void (*output_fn)(const char *), uint32_t sample_count);

#endif /* PERFORMANCE_MANAGER_H */

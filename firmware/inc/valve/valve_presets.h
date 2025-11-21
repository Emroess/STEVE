/*
 * valve_presets.h
 *
 * Simplified valve preset presets for haptic simulation.
 * Provides user-friendly preset levels that map to physics parameters.
 */

#ifndef VALVE_PRESETS_H
#define VALVE_PRESETS_H

#include "valve_haptic.h"
#include "valve_nvm.h"

#define VALVE_PRESET_COUNT 4

/* Global preset parameters */
extern struct preset_params preset_params[VALVE_PRESET_COUNT];

/* Initialize presets from NVM */
status_t valve_presets_init(void);

/* Save presets to NVM */
status_t valve_presets_save(const struct preset_params new_presets[VALVE_PRESET_COUNT]);

/* Get a single preset */
status_t valve_preset_get(int index, struct preset_params *out);

/* Set a single preset and save to NVM */
status_t valve_preset_set(int index, const struct preset_params *in);

/* Generate valve configuration from preset preset and travel range
 * 
 * @param preset: Resistance level (LIGHT, MEDIUM, HEAVY, INDUSTRIAL)
 * @param travel_degrees: Total rotation range (e.g., 90, 180, 360)
 * @param cfg: Output configuration structure to populate
 * @return: STATUS_OK on success, error code on invalid parameters
 */
status_t valve_preset_from_preset(valve_preset_t preset, float travel_degrees, struct valve_config *cfg);

/* Validate preset configuration */
status_t valve_preset_validate(const struct valve_config *cfg);

/* Get default travel range for a preset */
float valve_preset_get_default_travel(valve_preset_t preset);

/*
 * Save current valve configuration as a preset
 *
 * Returns: STATUS_OK on success, error code on failure
 * ctx: Valve context
 * preset_idx: Preset index (0-3)
 */
status_t valve_preset_save_current(struct valve_context *ctx,
    uint8_t preset_idx);

#endif /* VALVE_PRESETS_H */

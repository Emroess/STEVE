/*
 * valve_config.h - Valve configuration field masks
 *
 * Field masks for selective configuration updates via valve_haptic_stage_config()
 * Used by both CLI and HTTP API for atomic parameter updates
 */

#ifndef VALVE_CONFIG_H
#define VALVE_CONFIG_H

#include <stdint.h>

/* Configuration field bitmasks for selective updates */
#define CFG_FIELD_VISCOUS        (1U << 0)
#define CFG_FIELD_COULOMB        (1U << 1)
#define CFG_FIELD_WALL_STIFFNESS (1U << 2)
#define CFG_FIELD_WALL_DAMPING   (1U << 3)
#define CFG_FIELD_SMOOTHING      (1U << 4)
#define CFG_FIELD_TORQUE_LIMIT   (1U << 5)
#define CFG_FIELD_OPEN_POS       (1U << 6)
#define CFG_FIELD_CLOSED_POS     (1U << 7)
#define CFG_FIELD_SCALE          (1U << 8)

#endif /* VALVE_CONFIG_H */

/*
 * valve_physics.h
 *
 * HIL Physics Model: Viscous + Coulomb friction with soft end-stops
 * Exact implementation of hil_valve_model.py physics
 */

#ifndef VALVE_PHYSICS_H
#define VALVE_PHYSICS_H

#include "valve_haptic.h"
#include <stdbool.h>

/* Calculate total torque using HIL physics model (viscous + Coulomb) */
float valve_physics_calculate_torque_hil(
    const struct valve_config *cfg,
    float position_deg,
    float omega_rad_s,
    bool quiet_active
);

/* Clamp torque to limits */
float valve_physics_clamp_torque(
    float torque_nm,
    float limit_nm
);

#endif /* VALVE_PHYSICS_H */

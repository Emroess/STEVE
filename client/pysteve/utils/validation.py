"""
Validation utilities for STEVE data and parameters.
"""

from typing import Optional, Union
import numpy as np

from pysteve.core.config import ValveConfig
from pysteve.core.exceptions import SteveValidationError


def validate_config(config: ValveConfig) -> None:
    """
    Validate ValveConfig parameters.
    
    Args:
        config: Configuration to validate
    
    Raises:
        SteveValidationError: If validation fails
    """
    # Viscous damping
    if not (0.01 <= config.viscous <= 0.5):
        raise SteveValidationError(
            f"viscous must be in [0.01, 0.5], got {config.viscous}"
        )

    # Coulomb friction
    if not (0.005 <= config.coulomb <= 0.05):
        raise SteveValidationError(
            f"coulomb must be in [0.005, 0.05], got {config.coulomb}"
        )

    # Wall stiffness
    if not (0.5 <= config.wall_stiffness <= 5.0):
        raise SteveValidationError(
            f"wall_stiffness must be in [0.5, 5.0], got {config.wall_stiffness}"
        )

    # Wall damping
    if not (0.05 <= config.wall_damping <= 0.5):
        raise SteveValidationError(
            f"wall_damping must be in [0.05, 0.5], got {config.wall_damping}"
        )

    # Smoothing epsilon
    if not (0.0001 <= config.smoothing <= 0.01):
        raise SteveValidationError(
            f"smoothing must be in [0.0001, 0.01], got {config.smoothing}"
        )

    # Torque limit
    if not (0.1 <= config.torque_limit <= 2.0):
        raise SteveValidationError(
            f"torque_limit must be in [0.1, 2.0], got {config.torque_limit}"
        )

    # Travel
    if not (1 <= config.travel <= 360):
        raise SteveValidationError(
            f"travel must be in [1, 360], got {config.travel}"
        )

    # Position limits
    if config.open_position <= config.closed_position:
        raise SteveValidationError(
            f"open_position ({config.open_position}) must be > closed_position ({config.closed_position})"
        )


def validate_position(
    position: float, min_pos: float = 0, max_pos: float = 90
) -> None:
    """
    Validate position value.
    
    Args:
        position: Position in degrees
        min_pos: Minimum allowed position
        max_pos: Maximum allowed position
    
    Raises:
        SteveValidationError: If validation fails
    """
    if not (min_pos <= position <= max_pos):
        raise SteveValidationError(
            f"position must be in [{min_pos}, {max_pos}], got {position}"
        )


def validate_torque(torque: float, limit: float = 0.5) -> None:
    """
    Validate torque value.
    
    Args:
        torque: Torque in N·m
        limit: Maximum torque magnitude
    
    Raises:
        SteveValidationError: If validation fails
    """
    if abs(torque) > limit:
        raise SteveValidationError(
            f"torque magnitude must be <= {limit}, got {abs(torque)}"
        )


def validate_velocity(velocity: float, max_vel: float = 20.0) -> None:
    """
    Validate velocity value.
    
    Args:
        velocity: Angular velocity in rad/s
        max_vel: Maximum velocity magnitude
    
    Raises:
        SteveValidationError: If validation fails
    """
    if abs(velocity) > max_vel:
        raise SteveValidationError(
            f"velocity magnitude must be <= {max_vel}, got {abs(velocity)}"
        )


def validate_preset_slot(slot: Union[int, str]) -> int:
    """
    Validate preset slot.
    
    Args:
        slot: Preset slot (0-3 or name)
    
    Returns:
        Validated slot number
    
    Raises:
        SteveValidationError: If validation fails
    """
    # Convert name to slot
    if isinstance(slot, str):
        slot_map = {"tight": 0, "medium": 1, "smooth": 2, "loose": 3}
        if slot in slot_map:
            slot = slot_map[slot]
        else:
            raise SteveValidationError(f"Unknown preset name: {slot}")

    # Validate slot number
    if not isinstance(slot, int) or not (0 <= slot <= 3):
        raise SteveValidationError(f"Preset slot must be in [0, 3], got {slot}")

    return slot


def validate_sample(sample: dict) -> bool:
    """
    Validate data sample structure and values.
    
    Args:
        sample: Data sample dict
    
    Returns:
        True if valid, False otherwise
    """
    required_fields = ["position_deg", "omega_rad_s", "torque_nm", "data_valid"]

    # Check required fields
    for field in required_fields:
        if field not in sample:
            return False

    # Check data_valid flag
    if not sample["data_valid"]:
        return False

    # Check numeric fields are valid
    try:
        pos = float(sample["position_deg"])
        vel = float(sample["omega_rad_s"])
        torque = float(sample["torque_nm"])

        # Check for NaN or Inf
        if not all(np.isfinite([pos, vel, torque])):
            return False

    except (ValueError, TypeError):
        return False

    return True


def deg_to_rad(degrees: float) -> float:
    """
    Convert degrees to radians.
    
    Args:
        degrees: Angle in degrees
    
    Returns:
        Angle in radians
    """
    return np.deg2rad(degrees)


def rad_to_deg(radians: float) -> float:
    """
    Convert radians to degrees.
    
    Args:
        radians: Angle in radians
    
    Returns:
        Angle in degrees
    """
    return np.rad2deg(radians)


def normalize_angle(angle: float, min_angle: float = 0, max_angle: float = 360) -> float:
    """
    Normalize angle to [min_angle, max_angle) range.
    
    Args:
        angle: Angle in degrees
        min_angle: Minimum angle
        max_angle: Maximum angle
    
    Returns:
        Normalized angle
    """
    range_size = max_angle - min_angle
    normalized = ((angle - min_angle) % range_size) + min_angle
    return normalized


def clamp(value: float, min_val: float, max_val: float) -> float:
    """
    Clamp value to [min_val, max_val].
    
    Args:
        value: Value to clamp
        min_val: Minimum value
        max_val: Maximum value
    
    Returns:
        Clamped value
    """
    return max(min_val, min(max_val, value))


def interpolate_linear(
    x: float, x0: float, x1: float, y0: float, y1: float
) -> float:
    """
    Linear interpolation between two points.
    
    Args:
        x: Input value
        x0, x1: Input range
        y0, y1: Output range
    
    Returns:
        Interpolated value
    """
    if x1 == x0:
        return y0

    t = (x - x0) / (x1 - x0)
    return y0 + t * (y1 - y0)


def moving_average(data: list, window_size: int) -> list:
    """
    Compute moving average of data.
    
    Args:
        data: Input data
        window_size: Window size for averaging
    
    Returns:
        Smoothed data
    """
    if window_size < 1:
        return data

    result = []
    for i in range(len(data)):
        start = max(0, i - window_size + 1)
        end = i + 1
        window = data[start:end]
        result.append(sum(window) / len(window))

    return result


def detect_outliers(data: list, threshold: float = 3.0) -> list:
    """
    Detect outliers using z-score method.
    
    Args:
        data: Input data
        threshold: Z-score threshold for outliers
    
    Returns:
        List of outlier indices
    """
    data_array = np.array(data)
    mean = np.mean(data_array)
    std = np.std(data_array)

    if std == 0:
        return []

    z_scores = np.abs((data_array - mean) / std)
    outlier_indices = np.where(z_scores > threshold)[0].tolist()

    return outlier_indices


def compute_rms(data: list) -> float:
    """
    Compute root mean square (RMS) of data.
    
    Args:
        data: Input data
    
    Returns:
        RMS value
    """
    data_array = np.array(data)
    return float(np.sqrt(np.mean(data_array**2)))


def compute_derivative(
    values: list, timestamps: list, method: str = "central"
) -> list:
    """
    Compute numerical derivative.
    
    Args:
        values: Data values
        timestamps: Corresponding timestamps
        method: "forward", "backward", or "central"
    
    Returns:
        Derivative values
    """
    values_array = np.array(values)
    timestamps_array = np.array(timestamps)

    if method == "forward":
        deriv = np.gradient(values_array, timestamps_array, edge_order=1)
    elif method == "backward":
        deriv = np.gradient(values_array, timestamps_array, edge_order=1)
    elif method == "central":
        deriv = np.gradient(values_array, timestamps_array, edge_order=2)
    else:
        raise ValueError(f"Unknown derivative method: {method}")

    return deriv.tolist()

"""
Preset reward functions for STEVE valve RL environments.
"""

from typing import Dict, Any, Union
import numpy as np


def smooth_operation_reward(
    obs: Union[np.ndarray, Dict],
    action: np.ndarray,
    info: Dict[str, Any],
) -> float:
    """
    Reward smooth valve operation by penalizing jerk and acceleration.
    
    Encourages smooth, controlled movements with minimal vibration.
    
    Args:
        obs: Current observation (normalized)
        action: Applied action
        info: Step info with position, velocity, torque
    
    Returns:
        Reward value (higher is better)
    """
    # Extract velocity and torque
    if isinstance(obs, dict):
        vel_norm = obs["velocity"][0]
        torque_norm = obs["torque"][0]
    else:
        vel_norm = obs[1]
        torque_norm = obs[2]

    # Penalize high velocities (encourage slow, smooth motion)
    velocity_penalty = abs(vel_norm) * 0.5

    # Penalize high torques (encourage gentle operation)
    torque_penalty = abs(torque_norm) * 0.3

    # Penalize large action changes (encourage smooth control)
    action_magnitude_penalty = abs(action[0]) * 0.2

    # Base reward for staying active
    base_reward = 0.1

    reward = base_reward - velocity_penalty - torque_penalty - action_magnitude_penalty
    return float(reward)


def energy_efficiency_reward(
    obs: Union[np.ndarray, Dict],
    action: np.ndarray,
    info: Dict[str, Any],
) -> float:
    """
    Reward energy-efficient valve operation.
    
    Minimizes power consumption and wasted energy.
    
    Args:
        obs: Current observation (normalized)
        action: Applied action
        info: Step info with position, velocity, torque
    
    Returns:
        Reward value (higher is better)
    """
    # Extract values
    if isinstance(obs, dict):
        vel_norm = obs["velocity"][0]
        torque_norm = obs["torque"][0]
    else:
        vel_norm = obs[1]
        torque_norm = obs[2]

    # Get actual velocity and torque from info
    velocity = info.get("velocity", 0)
    torque = info.get("torque", 0)

    # Power = torque * angular_velocity
    power = abs(torque * velocity)

    # Penalize power consumption
    power_penalty = power * 0.5

    # Penalize unnecessary torque
    torque_penalty = abs(torque_norm) * 0.3

    # Base reward
    base_reward = 0.1

    reward = base_reward - power_penalty - torque_penalty
    return float(reward)


def trajectory_following_reward(
    obs: Union[np.ndarray, Dict],
    action: np.ndarray,
    info: Dict[str, Any],
    target_position: float,
) -> float:
    """
    Reward following a target trajectory or reaching target position.
    
    Penalizes position error from target.
    
    Args:
        obs: Current observation (normalized)
        action: Applied action
        info: Step info with position, velocity, torque
        target_position: Target position in degrees
    
    Returns:
        Reward value (higher is better)
    """
    # Get current position
    current_position = info.get("position", 0)

    # Compute position error
    position_error = abs(current_position - target_position)

    # Normalize error to [0, 1] assuming max error of 90 degrees
    normalized_error = min(position_error / 90.0, 1.0)

    # Penalize error
    error_penalty = normalized_error * 2.0

    # Extract velocity
    if isinstance(obs, dict):
        vel_norm = obs["velocity"][0]
    else:
        vel_norm = obs[1]

    # Penalize high velocity when near target (encourage settling)
    if position_error < 5.0:  # Within 5 degrees
        velocity_penalty = abs(vel_norm) * 0.5
    else:
        velocity_penalty = 0.0

    # Bonus for reaching target
    if position_error < 2.0:  # Within 2 degrees
        target_bonus = 1.0
    else:
        target_bonus = 0.0

    reward = target_bonus - error_penalty - velocity_penalty
    return float(reward)


def custom_reward_wrapper(
    reward_fn: callable,
) -> callable:
    """
    Wrap a custom reward function to match the standard interface.
    
    Args:
        reward_fn: Custom reward function with signature:
            (obs, action, info) -> float
    
    Returns:
        Wrapped reward function
    
    Example:
        >>> def my_reward(obs, action, info):
        ...     position = info["position"]
        ...     return -abs(position - 45.0)  # Reward staying at 45 deg
        >>> 
        >>> wrapped = custom_reward_wrapper(my_reward)
        >>> env = SteveValveEnv(
        ...     steve_ip="192.168.1.100",
        ...     reward_function=wrapped
        ... )
    """
    def wrapper(obs, action, info):
        return float(reward_fn(obs, action, info))

    return wrapper


def sparse_reward(
    obs: Union[np.ndarray, Dict],
    action: np.ndarray,
    info: Dict[str, Any],
    success_threshold: float = 2.0,
    target_position: float = 45.0,
) -> float:
    """
    Sparse reward: +1 when within threshold, 0 otherwise.
    
    Useful for goal-reaching tasks with HER (Hindsight Experience Replay).
    
    Args:
        obs: Current observation (normalized)
        action: Applied action
        info: Step info with position, velocity, torque
        success_threshold: Position error threshold for success (degrees)
        target_position: Target position in degrees
    
    Returns:
        Reward value (1.0 for success, 0.0 otherwise)
    """
    current_position = info.get("position", 0)
    position_error = abs(current_position - target_position)

    if position_error <= success_threshold:
        return 1.0
    else:
        return 0.0


def shaped_distance_reward(
    obs: Union[np.ndarray, Dict],
    action: np.ndarray,
    info: Dict[str, Any],
    target_position: float = 45.0,
    distance_scale: float = 0.01,
) -> float:
    """
    Reward shaped by negative distance to target.
    
    Provides dense feedback proportional to distance from goal.
    
    Args:
        obs: Current observation (normalized)
        action: Applied action
        info: Step info with position, velocity, torque
        target_position: Target position in degrees
        distance_scale: Scale factor for distance penalty
    
    Returns:
        Reward value (higher is better)
    """
    current_position = info.get("position", 0)
    distance = abs(current_position - target_position)

    # Negative distance scaled
    reward = -distance * distance_scale

    return float(reward)


def multi_objective_reward(
    obs: Union[np.ndarray, Dict],
    action: np.ndarray,
    info: Dict[str, Any],
    target_position: float = 45.0,
    weights: Dict[str, float] = None,
) -> float:
    """
    Multi-objective reward combining multiple criteria.
    
    Balances trajectory following, smoothness, and energy efficiency.
    
    Args:
        obs: Current observation (normalized)
        action: Applied action
        info: Step info with position, velocity, torque
        target_position: Target position in degrees
        weights: Weights for each objective {
            "position": 1.0,
            "smoothness": 0.5,
            "energy": 0.3
        }
    
    Returns:
        Weighted sum of objectives
    """
    if weights is None:
        weights = {"position": 1.0, "smoothness": 0.5, "energy": 0.3}

    # Position tracking component
    position_reward = trajectory_following_reward(obs, action, info, target_position)

    # Smoothness component
    smoothness_reward = smooth_operation_reward(obs, action, info)

    # Energy efficiency component
    energy_reward = energy_efficiency_reward(obs, action, info)

    # Weighted sum
    total_reward = (
        weights.get("position", 1.0) * position_reward
        + weights.get("smoothness", 0.5) * smoothness_reward
        + weights.get("energy", 0.3) * energy_reward
    )

    return float(total_reward)

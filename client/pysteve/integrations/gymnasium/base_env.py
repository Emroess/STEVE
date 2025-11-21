"""
Gymnasium environment for STEVE valve control with RL.
"""

import time
from typing import Optional, Dict, Any, Tuple, Callable, Union, Literal
import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:
    raise ImportError(
        "Gymnasium required. Install with: pip install pysteve[gymnasium]"
    )

from pysteve.core.client import SteveClient
from pysteve.core.streaming import SteveStreamer
from pysteve.core.exceptions import SteveError


class SteveValveEnv(gym.Env):
    """
    Gymnasium environment for STEVE valve control and reinforcement learning.
    
    Provides configurable observation/action spaces, reward functions, and
    termination conditions for training RL agents on valve manipulation tasks.
    
    Args:
        steve_ip: STEVE device IP address
        max_steps: Maximum steps per episode (default: 1000)
        terminate_on_torque_limit: End episode if torque limit hit (default: False)
        terminate_on_position_limit: End episode if position limit hit (default: True)
        terminate_on_energy: Energy threshold for termination (default: None)
        reward_function: Reward function name or callable (default: "smooth_operation")
        observation_type: 'vector' or 'dict' (default: 'vector')
        action_type: 'torque' or 'config' (default: 'torque')
        target_position: Target position for tracking tasks (default: None)
        stream_rate_hz: Data streaming rate (default: 50)
        api_key: STEVE API key (default: "steve-valve-2025")
    
    Observation Space (vector mode):
        Box(3): [position_normalized, velocity_normalized, torque_normalized]
    
    Action Space (torque mode):
        Box(1): [target_torque] in range [-1, 1] (scaled to torque_limit)
    
    Example:
        >>> env = SteveValveEnv(
        ...     steve_ip="192.168.1.100",
        ...     max_steps=1000,
        ...     reward_function="trajectory_following",
        ...     target_position=45.0
        ... )
        >>> 
        >>> obs, info = env.reset()
        >>> for _ in range(1000):
        ...     action = env.action_space.sample()
        ...     obs, reward, terminated, truncated, info = env.step(action)
        ...     if terminated or truncated:
        ...         break
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        steve_ip: str,
        max_steps: int = 1000,
        terminate_on_torque_limit: bool = False,
        terminate_on_position_limit: bool = True,
        terminate_on_energy: Optional[float] = None,
        reward_function: Union[str, Callable] = "smooth_operation",
        observation_type: Literal["vector", "dict"] = "vector",
        action_type: Literal["torque", "config"] = "torque",
        target_position: Optional[float] = None,
        stream_rate_hz: int = 50,
        api_key: str = "steve-valve-2025",
    ):
        super().__init__()

        self.steve_ip = steve_ip
        self.max_steps = max_steps
        self.terminate_on_torque_limit = terminate_on_torque_limit
        self.terminate_on_position_limit = terminate_on_position_limit
        self.terminate_on_energy = terminate_on_energy
        self.reward_function_name = reward_function if isinstance(reward_function, str) else "custom"
        self.observation_type = observation_type
        self.action_type = action_type
        self.target_position = target_position
        self.stream_rate_hz = stream_rate_hz
        self.api_key = api_key

        # Client and streaming
        self.client: Optional[SteveClient] = None
        self.streamer: Optional[SteveStreamer] = None

        # Episode state
        self._current_step = 0
        self._episode_reward = 0.0
        self._last_state: Optional[Dict[str, Any]] = None
        self._last_action: Optional[np.ndarray] = None

        # Configuration
        self._position_range = (0.0, 90.0)  # Will be updated from config
        self._velocity_range = (-10.0, 10.0)  # rad/s
        self._torque_range = (-0.5, 0.5)  # Nm

        # Define observation space
        if observation_type == "vector":
            # [position_norm, velocity_norm, torque_norm]
            self.observation_space = spaces.Box(
                low=-1.0, high=1.0, shape=(3,), dtype=np.float32
            )
        else:
            # Dict space with separate fields
            self.observation_space = spaces.Dict(
                {
                    "position": spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32),
                    "velocity": spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32),
                    "torque": spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32),
                }
            )

        # Define action space
        if action_type == "torque":
            # Direct torque control
            self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)
        else:
            # Configuration parameter updates (viscous, coulomb, wall_stiffness)
            self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)

        # Reward function
        if callable(reward_function):
            self._reward_fn = reward_function
        else:
            self._reward_fn = self._get_reward_function(reward_function)

    def _get_reward_function(self, name: str) -> Callable:
        """Get reward function by name."""
        from pysteve.integrations.gymnasium.reward_functions import (
            smooth_operation_reward,
            energy_efficiency_reward,
            trajectory_following_reward,
        )

        if name == "smooth_operation":
            return smooth_operation_reward
        elif name == "energy_efficiency":
            return energy_efficiency_reward
        elif name == "trajectory_following":
            return lambda obs, action, info: trajectory_following_reward(
                obs, action, info, self.target_position or 45.0
            )
        else:
            raise ValueError(f"Unknown reward function: {name}")

    def reset(
        self, seed: Optional[int] = None, options: Optional[Dict] = None
    ) -> Tuple[Union[np.ndarray, Dict], Dict]:
        """
        Reset environment to initial state.
        
        Args:
            seed: Random seed
            options: Reset options (preset, initial_position, etc.)
        
        Returns:
            Tuple of (observation, info)
        """
        super().reset(seed=seed)

        # Connect if not connected
        if self.client is None:
            self._connect()

        # Reset episode state
        self._current_step = 0
        self._episode_reward = 0.0
        self._last_action = None

        # Load preset if specified
        preset = options.get("preset", "smooth") if options else "smooth"
        self.client.load_preset(preset)
        time.sleep(0.2)

        # Enable motor and start valve
        try:
            self.client.enable_motor()
            time.sleep(0.5)
            self.client.start_valve()
            time.sleep(0.3)
        except Exception as e:
            raise SteveError(f"Failed to start valve: {e}")

        # Update config ranges
        config = self.client.get_config()
        self._position_range = (config.closed_position, config.open_position)
        self._torque_range = (-config.torque_limit, config.torque_limit)

        # Wait for first data sample
        max_wait = 2.0
        start_time = time.time()
        while time.time() - start_time < max_wait:
            state = self.streamer.get_latest_sample()
            if state and state.get("data_valid"):
                self._last_state = state
                break
            time.sleep(0.05)

        if self._last_state is None:
            raise SteveError("No valid data received during reset")

        # Build observation
        obs = self._build_observation(self._last_state)

        # Build info
        info = {
            "episode": 0,
            "step": 0,
            "preset": preset,
        }

        return obs, info

    def step(
        self, action: np.ndarray
    ) -> Tuple[Union[np.ndarray, Dict], float, bool, bool, Dict]:
        """
        Execute one step in the environment.
        
        Args:
            action: Action from policy
        
        Returns:
            Tuple of (observation, reward, terminated, truncated, info)
        """
        self._current_step += 1
        self._last_action = action

        # Apply action
        if self.action_type == "torque":
            # Scale action to torque range
            torque = float(action[0]) * self._torque_range[1]
            try:
                self.client.set_torque(torque)
            except Exception:
                pass  # Ignore errors
        else:
            # Scale actions to config parameter ranges
            viscous = 0.01 + (action[0] + 1) * 0.245  # 0.01-0.5
            coulomb = 0.005 + (action[1] + 1) * 0.0225  # 0.005-0.05
            wall_stiff = 0.5 + (action[2] + 1) * 2.25  # 0.5-5.0
            try:
                self.client.update_config(
                    viscous=float(viscous),
                    coulomb=float(coulomb),
                    wall_stiffness=float(wall_stiff),
                )
            except Exception:
                pass

        # Wait for next sample (at stream rate)
        time.sleep(1.0 / self.stream_rate_hz)

        # Get current state
        state = self.streamer.get_latest_sample()
        if state is None or not state.get("data_valid"):
            # Use last valid state
            state = self._last_state

        self._last_state = state

        # Build observation
        obs = self._build_observation(state)

        # Compute reward
        info = {
            "step": self._current_step,
            "position": state.get("position_deg", 0),
            "velocity": state.get("omega_rad_s", 0),
            "torque": state.get("torque_nm", 0),
        }
        reward = self._reward_fn(obs, action, info)
        self._episode_reward += reward

        # Check termination conditions
        terminated, termination_reason = self._check_termination(state)
        truncated = self._current_step >= self.max_steps

        if terminated or truncated:
            info["termination_reason"] = termination_reason if terminated else "max_steps"
            info["episode_reward"] = self._episode_reward
            info["episode_length"] = self._current_step

        return obs, reward, terminated, truncated, info

    def _connect(self) -> None:
        """Connect to STEVE device and start streaming."""
        self.client = SteveClient(
            self.steve_ip, api_key=self.api_key, auto_reconnect=True
        )
        self.client.connect()

        stream_interval_ms = int(1000 / self.stream_rate_hz)
        self.streamer = SteveStreamer(self.client, threadsafe=True)
        self.streamer.start_stream(interval_ms=stream_interval_ms)

        # Wait for streaming to start
        time.sleep(0.5)

    def _build_observation(self, state: Dict[str, Any]) -> Union[np.ndarray, Dict]:
        """Build observation from state."""
        # Normalize values to [-1, 1]
        pos = state.get("position_deg", 0)
        vel = state.get("omega_rad_s", 0)
        torque = state.get("torque_nm", 0)

        pos_norm = self._normalize(pos, self._position_range)
        vel_norm = self._normalize(vel, self._velocity_range)
        torque_norm = self._normalize(torque, self._torque_range)

        if self.observation_type == "vector":
            return np.array([pos_norm, vel_norm, torque_norm], dtype=np.float32)
        else:
            return {
                "position": np.array([pos_norm], dtype=np.float32),
                "velocity": np.array([vel_norm], dtype=np.float32),
                "torque": np.array([torque_norm], dtype=np.float32),
            }

    def _normalize(self, value: float, range_tuple: Tuple[float, float]) -> float:
        """Normalize value to [-1, 1] based on range."""
        min_val, max_val = range_tuple
        if max_val == min_val:
            return 0.0
        normalized = 2.0 * (value - min_val) / (max_val - min_val) - 1.0
        return np.clip(normalized, -1.0, 1.0)

    def _check_termination(self, state: Dict[str, Any]) -> Tuple[bool, Optional[str]]:
        """Check if episode should terminate."""
        # Check position limits
        if self.terminate_on_position_limit:
            pos = state.get("position_deg", 0)
            if pos <= self._position_range[0] or pos >= self._position_range[1]:
                return True, "position_limit"

        # Check torque limits
        if self.terminate_on_torque_limit:
            torque = abs(state.get("torque_nm", 0))
            if torque >= abs(self._torque_range[1]) * 0.95:
                return True, "torque_limit"

        # Check energy threshold
        if self.terminate_on_energy is not None:
            energy = state.get("passivity_mj", 0) / 1000.0  # Convert to J
            if energy >= self.terminate_on_energy:
                return True, "energy_threshold"

        return False, None

    def close(self) -> None:
        """Cleanup resources."""
        if self.client:
            try:
                self.client.stop_valve()
                self.client.disable_motor()
            except Exception:
                pass

        if self.streamer:
            try:
                self.streamer.stop_stream()
            except Exception:
                pass

        if self.client:
            try:
                self.client.disconnect()
            except Exception:
                pass

    def __del__(self) -> None:
        """Cleanup on deletion."""
        self.close()

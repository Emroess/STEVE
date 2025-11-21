"""
Gymnasium wrappers for STEVE valve environments.
"""

from typing import Optional, Tuple, Union, Dict
import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
    from gymnasium.core import ObservationWrapper, RewardWrapper, ActionWrapper
except ImportError:
    raise ImportError(
        "Gymnasium required. Install with: pip install pysteve[gymnasium]"
    )


class NormalizeObservationWrapper(ObservationWrapper):
    """
    Normalize observations to zero mean and unit variance.
    
    Tracks running statistics and applies normalization.
    
    Args:
        env: Environment to wrap
        epsilon: Small value to avoid division by zero
        clip: Clip normalized values to [-clip, clip]
    
    Example:
        >>> env = SteveValveEnv(steve_ip="192.168.1.100")
        >>> env = NormalizeObservationWrapper(env)
    """

    def __init__(
        self,
        env: gym.Env,
        epsilon: float = 1e-8,
        clip: Optional[float] = 10.0,
    ):
        super().__init__(env)
        self.epsilon = epsilon
        self.clip = clip

        # Running statistics
        self.obs_mean = np.zeros(env.observation_space.shape)
        self.obs_var = np.ones(env.observation_space.shape)
        self.count = 0

    def observation(self, obs: np.ndarray) -> np.ndarray:
        """Normalize observation."""
        # Update running statistics
        self.count += 1
        delta = obs - self.obs_mean
        self.obs_mean += delta / self.count
        delta2 = obs - self.obs_mean
        self.obs_var += (delta * delta2 - self.obs_var) / self.count

        # Normalize
        obs_std = np.sqrt(self.obs_var + self.epsilon)
        normalized = (obs - self.obs_mean) / obs_std

        # Clip if specified
        if self.clip is not None:
            normalized = np.clip(normalized, -self.clip, self.clip)

        return normalized


class FilterObservationWrapper(ObservationWrapper):
    """
    Apply low-pass filtering to observations to reduce noise.
    
    Uses exponential moving average (EMA) filter.
    
    Args:
        env: Environment to wrap
        alpha: Filter coefficient (0=no filtering, 1=no smoothing)
    
    Example:
        >>> env = SteveValveEnv(steve_ip="192.168.1.100")
        >>> env = FilterObservationWrapper(env, alpha=0.3)
    """

    def __init__(self, env: gym.Env, alpha: float = 0.3):
        super().__init__(env)
        self.alpha = alpha
        self.filtered_obs: Optional[np.ndarray] = None

    def observation(self, obs: np.ndarray) -> np.ndarray:
        """Apply EMA filter."""
        if self.filtered_obs is None:
            self.filtered_obs = obs
        else:
            self.filtered_obs = self.alpha * obs + (1 - self.alpha) * self.filtered_obs

        return self.filtered_obs

    def reset(self, **kwargs):
        """Reset filter state."""
        self.filtered_obs = None
        return super().reset(**kwargs)


class FrameStackWrapper(gym.Wrapper):
    """
    Stack multiple consecutive frames as observation.
    
    Useful for recurrent patterns and velocity estimation.
    
    Args:
        env: Environment to wrap
        num_stack: Number of frames to stack
    
    Example:
        >>> env = SteveValveEnv(steve_ip="192.168.1.100")
        >>> env = FrameStackWrapper(env, num_stack=4)
    """

    def __init__(self, env: gym.Env, num_stack: int = 4):
        super().__init__(env)
        self.num_stack = num_stack

        # Create frame buffer
        obs_shape = env.observation_space.shape
        self.frames = np.zeros((num_stack,) + obs_shape, dtype=np.float32)

        # Update observation space
        stacked_shape = (num_stack * obs_shape[0],)
        self.observation_space = spaces.Box(
            low=-1.0, high=1.0, shape=stacked_shape, dtype=np.float32
        )

    def reset(self, **kwargs):
        """Reset with stacked frames."""
        obs, info = self.env.reset(**kwargs)
        self.frames = np.zeros_like(self.frames)
        self.frames[-1] = obs
        return self._get_obs(), info

    def step(self, action):
        """Step and update frame stack."""
        obs, reward, terminated, truncated, info = self.env.step(action)

        # Shift frames and add new observation
        self.frames[:-1] = self.frames[1:]
        self.frames[-1] = obs

        return self._get_obs(), reward, terminated, truncated, info

    def _get_obs(self) -> np.ndarray:
        """Get stacked observation."""
        return self.frames.flatten()


class TimeLimitWrapper(gym.Wrapper):
    """
    Add time limit to episodes.
    
    Truncates episodes after max_steps.
    
    Args:
        env: Environment to wrap
        max_steps: Maximum steps per episode
    
    Example:
        >>> env = SteveValveEnv(steve_ip="192.168.1.100")
        >>> env = TimeLimitWrapper(env, max_steps=500)
    """

    def __init__(self, env: gym.Env, max_steps: int = 1000):
        super().__init__(env)
        self.max_steps = max_steps
        self._elapsed_steps = 0

    def reset(self, **kwargs):
        """Reset step counter."""
        self._elapsed_steps = 0
        return self.env.reset(**kwargs)

    def step(self, action):
        """Step and check time limit."""
        obs, reward, terminated, truncated, info = self.env.step(action)
        self._elapsed_steps += 1

        if self._elapsed_steps >= self.max_steps:
            truncated = True
            info["TimeLimit.truncated"] = True

        return obs, reward, terminated, truncated, info


class RewardScalingWrapper(RewardWrapper):
    """
    Scale rewards by constant factor.
    
    Args:
        env: Environment to wrap
        scale: Scale factor for rewards
    
    Example:
        >>> env = SteveValveEnv(steve_ip="192.168.1.100")
        >>> env = RewardScalingWrapper(env, scale=10.0)
    """

    def __init__(self, env: gym.Env, scale: float = 1.0):
        super().__init__(env)
        self.scale = scale

    def reward(self, reward: float) -> float:
        """Scale reward."""
        return reward * self.scale


class RewardClippingWrapper(RewardWrapper):
    """
    Clip rewards to specified range.
    
    Args:
        env: Environment to wrap
        min_reward: Minimum reward value
        max_reward: Maximum reward value
    
    Example:
        >>> env = SteveValveEnv(steve_ip="192.168.1.100")
        >>> env = RewardClippingWrapper(env, min_reward=-1.0, max_reward=1.0)
    """

    def __init__(
        self,
        env: gym.Env,
        min_reward: float = -10.0,
        max_reward: float = 10.0,
    ):
        super().__init__(env)
        self.min_reward = min_reward
        self.max_reward = max_reward

    def reward(self, reward: float) -> float:
        """Clip reward."""
        return np.clip(reward, self.min_reward, self.max_reward)


class ActionRepeatWrapper(gym.Wrapper):
    """
    Repeat each action for multiple steps.
    
    Useful for temporal abstraction and faster training.
    
    Args:
        env: Environment to wrap
        repeat: Number of times to repeat each action
    
    Example:
        >>> env = SteveValveEnv(steve_ip="192.168.1.100")
        >>> env = ActionRepeatWrapper(env, repeat=4)
    """

    def __init__(self, env: gym.Env, repeat: int = 4):
        super().__init__(env)
        self.repeat = repeat

    def step(self, action):
        """Repeat action multiple times."""
        total_reward = 0.0
        for _ in range(self.repeat):
            obs, reward, terminated, truncated, info = self.env.step(action)
            total_reward += reward
            if terminated or truncated:
                break

        return obs, total_reward, terminated, truncated, info


class DictToBoxWrapper(ObservationWrapper):
    """
    Convert Dict observation space to Box.
    
    Flattens dictionary observations into vector.
    
    Args:
        env: Environment with Dict observation space
    
    Example:
        >>> env = SteveValveEnv(
        ...     steve_ip="192.168.1.100",
        ...     observation_type="dict"
        ... )
        >>> env = DictToBoxWrapper(env)
    """

    def __init__(self, env: gym.Env):
        super().__init__(env)

        if not isinstance(env.observation_space, spaces.Dict):
            raise ValueError("Environment must have Dict observation space")

        # Compute flattened size
        total_size = 0
        for space in env.observation_space.spaces.values():
            if isinstance(space, spaces.Box):
                total_size += np.prod(space.shape)
            else:
                raise ValueError("Only Box spaces supported in Dict")

        # Create new Box space
        self.observation_space = spaces.Box(
            low=-1.0, high=1.0, shape=(total_size,), dtype=np.float32
        )

    def observation(self, obs: Dict) -> np.ndarray:
        """Flatten dict observation."""
        parts = []
        for key in sorted(obs.keys()):
            parts.append(obs[key].flatten())
        return np.concatenate(parts)


class ActionSmoothingWrapper(ActionWrapper):
    """
    Smooth actions using exponential moving average.
    
    Reduces sudden action changes for smoother control.
    
    Args:
        env: Environment to wrap
        alpha: Smoothing factor (0=no smoothing, 1=no filtering)
    
    Example:
        >>> env = SteveValveEnv(steve_ip="192.168.1.100")
        >>> env = ActionSmoothingWrapper(env, alpha=0.5)
    """

    def __init__(self, env: gym.Env, alpha: float = 0.5):
        super().__init__(env)
        self.alpha = alpha
        self.last_action: Optional[np.ndarray] = None

    def action(self, action: np.ndarray) -> np.ndarray:
        """Smooth action."""
        if self.last_action is None:
            self.last_action = action
        else:
            action = self.alpha * action + (1 - self.alpha) * self.last_action
            self.last_action = action

        return action

    def reset(self, **kwargs):
        """Reset action history."""
        self.last_action = None
        return self.env.reset(**kwargs)

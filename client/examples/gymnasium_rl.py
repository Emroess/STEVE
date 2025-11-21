"""
Example: Using STEVE with Gymnasium for reinforcement learning.

This example demonstrates:
1. Creating a Gymnasium environment with STEVE
2. Training a simple RL agent
3. Using different reward functions and wrappers
4. Evaluating trained agent performance
"""

import time
import numpy as np
from pysteve.integrations.gymnasium import (
    SteveValveEnv,
    NormalizeObservationWrapper,
    ActionSmoothingWrapper,
)

# Configuration
STEVE_IP = "192.168.1.100"
NUM_EPISODES = 10


def random_policy_example():
    """Example 1: Random policy baseline."""
    print("=" * 60)
    print("Example 1: Random Policy Baseline")
    print("=" * 60)

    # Create environment
    env = SteveValveEnv(
        steve_ip=STEVE_IP,
        max_steps=500,
        reward_function="smooth_operation",
    )

    try:
        episode_rewards = []

        for episode in range(NUM_EPISODES):
            obs, info = env.reset()
            episode_reward = 0.0

            for step in range(500):
                # Random action
                action = env.action_space.sample()

                # Step environment
                obs, reward, terminated, truncated, info = env.step(action)
                episode_reward += reward

                if terminated or truncated:
                    print(
                        f"Episode {episode + 1}: "
                        f"reward={episode_reward:.2f}, "
                        f"steps={step + 1}, "
                        f"reason={info.get('termination_reason', 'max_steps')}"
                    )
                    break

            episode_rewards.append(episode_reward)

        print(f"\nMean reward: {np.mean(episode_rewards):.2f}")
        print(f"Std reward: {np.std(episode_rewards):.2f}")

    finally:
        env.close()


def trajectory_tracking_example():
    """Example 2: Trajectory tracking task."""
    print("\n" + "=" * 60)
    print("Example 2: Trajectory Tracking")
    print("=" * 60)

    target_position = 45.0  # Target at 45 degrees

    # Create environment with trajectory following reward
    env = SteveValveEnv(
        steve_ip=STEVE_IP,
        max_steps=1000,
        reward_function="trajectory_following",
        target_position=target_position,
        terminate_on_position_limit=False,
    )

    # Add wrappers for better performance
    env = ActionSmoothingWrapper(env, alpha=0.7)

    try:
        obs, info = env.reset()
        print(f"Target position: {target_position} degrees")

        for step in range(1000):
            # Simple proportional controller
            current_position = info.get("position", 0)
            error = target_position - current_position

            # Proportional action (scaled to [-1, 1])
            action = np.array([np.clip(error / 90.0, -1.0, 1.0)])

            obs, reward, terminated, truncated, info = env.step(action)

            if step % 100 == 0:
                print(
                    f"Step {step}: "
                    f"pos={info['position']:.2f}°, "
                    f"error={error:.2f}°, "
                    f"reward={reward:.3f}"
                )

            if terminated or truncated:
                print(f"\nEpisode ended at step {step}")
                print(f"Final position: {info['position']:.2f}°")
                print(f"Final error: {abs(info['position'] - target_position):.2f}°")
                break

    finally:
        env.close()


def wrapped_environment_example():
    """Example 3: Environment with multiple wrappers."""
    print("\n" + "=" * 60)
    print("Example 3: Wrapped Environment")
    print("=" * 60)

    # Create base environment
    env = SteveValveEnv(
        steve_ip=STEVE_IP,
        max_steps=500,
        reward_function="energy_efficiency",
    )

    # Apply wrappers
    env = NormalizeObservationWrapper(env, clip=5.0)
    env = ActionSmoothingWrapper(env, alpha=0.5)

    try:
        obs, info = env.reset()
        print("Environment wrapped with:")
        print("  - NormalizeObservationWrapper (clip=5.0)")
        print("  - ActionSmoothingWrapper (alpha=0.5)")

        episode_reward = 0.0

        for step in range(500):
            # Random action
            action = env.action_space.sample()

            obs, reward, terminated, truncated, info = env.step(action)
            episode_reward += reward

            if step % 100 == 0:
                print(
                    f"Step {step}: "
                    f"obs_norm={np.linalg.norm(obs):.3f}, "
                    f"reward={reward:.3f}"
                )

            if terminated or truncated:
                print(f"\nEpisode reward: {episode_reward:.2f}")
                break

    finally:
        env.close()


def custom_reward_example():
    """Example 4: Custom reward function."""
    print("\n" + "=" * 60)
    print("Example 4: Custom Reward Function")
    print("=" * 60)

    # Define custom reward function
    def my_custom_reward(obs, action, info):
        """Reward staying near 30 degrees with low velocity."""
        target = 30.0
        position = info.get("position", 0)
        velocity = abs(info.get("velocity", 0))

        position_error = abs(position - target)
        position_reward = -position_error / 10.0

        velocity_penalty = -velocity * 0.5

        return position_reward + velocity_penalty

    # Create environment with custom reward
    env = SteveValveEnv(
        steve_ip=STEVE_IP,
        max_steps=500,
        reward_function=my_custom_reward,
    )

    try:
        obs, info = env.reset()
        print("Using custom reward: stay near 30° with low velocity")

        episode_reward = 0.0

        for step in range(500):
            # Simple controller toward 30 degrees
            current_position = info.get("position", 0)
            error = 30.0 - current_position
            action = np.array([np.clip(error / 90.0, -1.0, 1.0)])

            obs, reward, terminated, truncated, info = env.step(action)
            episode_reward += reward

            if step % 100 == 0:
                print(
                    f"Step {step}: "
                    f"pos={info['position']:.2f}°, "
                    f"vel={info['velocity']:.3f} rad/s, "
                    f"reward={reward:.3f}"
                )

            if terminated or truncated:
                print(f"\nTotal reward: {episode_reward:.2f}")
                break

    finally:
        env.close()


def main():
    """Run all examples."""
    print("\n" + "=" * 60)
    print("STEVE Gymnasium Integration Examples")
    print("=" * 60)
    print(f"\nConnecting to STEVE device at {STEVE_IP}")
    print("Make sure the device is powered on and connected to the network.\n")

    try:
        # Run examples
        random_policy_example()
        time.sleep(2)

        trajectory_tracking_example()
        time.sleep(2)

        wrapped_environment_example()
        time.sleep(2)

        custom_reward_example()

        print("\n" + "=" * 60)
        print("All examples completed successfully!")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\n\nExamples interrupted by user.")
    except Exception as e:
        print(f"\n\nError running examples: {e}")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    main()

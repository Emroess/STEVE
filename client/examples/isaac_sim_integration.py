"""
Example: Using STEVE with NVIDIA Isaac Sim.

This example demonstrates:
1. Creating USD scene with valves
2. Connecting STEVE hardware to Isaac Sim
3. Bidirectional synchronization
4. Multi-valve coordination

Note: This requires NVIDIA Isaac Sim 2023.1+ to be installed and configured.
"""

import time
import asyncio
from omni.isaac.core import World
from omni.isaac.core.utils.stage import get_current_stage

from pysteve import SteveClient
from pysteve.integrations.isaac import (
    IsaacSteveConnector,
    IsaacSceneBuilder,
    MultiValveCoordinator,
)
from pysteve.core.config import ValveConfig

# Configuration
STEVE_IP = "192.168.1.100"


def basic_isaac_integration():
    """Example 1: Basic Isaac Sim integration."""
    print("=" * 60)
    print("Example 1: Basic Isaac Sim Integration")
    print("=" * 60)

    # Get USD stage
    stage = get_current_stage()

    # Build scene with valve
    builder = IsaacSceneBuilder(stage)

    # Add lighting
    builder.add_lighting()

    # Create valve configuration
    config = ValveConfig(
        viscous=0.15, coulomb=0.02, wall_stiffness=2.0, torque_limit=0.5, travel=90
    )

    # Create valve in scene
    valve_path = builder.create_valve(
        parent_path="/World",
        valve_name="SteveValve",
        position=(0, 0, 0.5),
        config=config,
    )

    print(f"Created valve at {valve_path}")

    # Connect to hardware
    connector = IsaacSteveConnector(
        stage=stage,
        device_ip=STEVE_IP,
        prim_path=valve_path,
        sync_mode="bidirectional",
    )

    try:
        # Connect and start
        connector.connect()
        connector.start()

        print("\nStarting synchronization...")
        print("Valve will sync between Isaac Sim and hardware")

        # Run simulation loop
        for i in range(1000):
            # Update synchronization
            connector.update()

            if i % 100 == 0:
                hw_state = connector.get_hardware_state()
                if hw_state:
                    print(
                        f"Step {i:4d}: "
                        f"HW pos={hw_state.get('position_deg', 0):6.2f}°, "
                        f"torque={hw_state.get('torque_nm', 0):6.3f} N·m"
                    )

            time.sleep(0.01)

        print("\nSimulation complete!")

    finally:
        connector.disconnect()


def multi_valve_coordination():
    """Example 2: Multi-valve coordination."""
    print("\n" + "=" * 60)
    print("Example 2: Multi-Valve Coordination")
    print("=" * 60)

    # Device IPs
    devices = [
        {"id": "valve1", "ip": "192.168.1.100"},
        {"id": "valve2", "ip": "192.168.1.101"},
    ]

    # Get stage
    stage = get_current_stage()

    # Build scene
    builder = IsaacSceneBuilder(stage)
    builder.add_lighting()

    # Create valve array
    config = ValveConfig(viscous=0.1, coulomb=0.01, wall_stiffness=1.5)

    valve_paths = builder.create_valve_array(
        parent_path="/World",
        array_name="ValveArray",
        num_valves=len(devices),
        spacing=0.3,
        config=config,
    )

    # Create coordinator
    coordinator = MultiValveCoordinator(stage=stage, sync_mode="bidirectional")

    # Add valves to coordinator
    for i, device in enumerate(devices):
        coordinator.add_valve(
            valve_id=device["id"], device_ip=device["ip"], prim_path=valve_paths[i]
        )

    try:
        # Connect all
        coordinator.connect_all()
        coordinator.calibrate_all()
        coordinator.start_all_valves()
        coordinator.start_all()

        print(f"\nCoordinating {len(devices)} valves...")

        # Run synchronized operation
        for i in range(1000):
            # Update all valves
            coordinator.update_all()

            if i % 100 == 0:
                # Get synchronized observations
                observations = coordinator.get_synchronized_observations()

                print(f"\nStep {i}:")
                for valve_id, obs in observations.items():
                    print(
                        f"  {valve_id}: "
                        f"pos={obs[0]:6.2f}°, "
                        f"vel={obs[1]:6.3f} rad/s, "
                        f"torque={obs[2]:6.3f} N·m"
                    )

            time.sleep(0.01)

        # Get statistics
        stats = coordinator.get_statistics()
        print("\nCoordinator Statistics:")
        print(f"  Uptime: {stats['uptime_s']:.1f}s")
        print(f"  Active valves: {stats['num_valves']}")

        print("\nMulti-valve coordination complete!")

    finally:
        coordinator.disconnect_all()


def usd_config_sync_example():
    """Example 3: Syncing config between USD and hardware."""
    print("\n" + "=" * 60)
    print("Example 3: USD Configuration Sync")
    print("=" * 60)

    stage = get_current_stage()

    # Build valve
    builder = IsaacSceneBuilder(stage)
    config = ValveConfig(viscous=0.2, coulomb=0.015, wall_stiffness=3.0)

    valve_path = builder.create_valve(
        parent_path="/World", valve_name="ConfigSyncValve", position=(0, 0, 0.5), config=config
    )

    # Connect
    connector = IsaacSteveConnector(stage=stage, device_ip=STEVE_IP, prim_path=valve_path)

    try:
        connector.connect()

        # Sync config TO USD (from hardware)
        print("\nSyncing config from hardware to USD...")
        hw_config = connector.client.get_config()
        connector.sync_config_to_usd(hw_config)
        print(f"  Hardware config written to {valve_path}")

        # Modify USD config
        print("\nModifying USD config...")
        modified_config = ValveConfig(viscous=0.25, coulomb=0.02, wall_stiffness=2.5)
        connector.sync_config_to_usd(modified_config)

        # Sync config FROM USD (to hardware)
        print("\nSyncing config from USD to hardware...")
        usd_config = connector.sync_config_from_usd()
        connector.client.update_config(**usd_config.to_dict())
        print("  USD config applied to hardware")

        # Verify
        new_hw_config = connector.client.get_config()
        print(f"\nVerified hardware config:")
        print(f"  Viscous: {new_hw_config.viscous:.3f}")
        print(f"  Coulomb: {new_hw_config.coulomb:.3f}")
        print(f"  Wall stiffness: {new_hw_config.wall_stiffness:.3f}")

    finally:
        connector.disconnect()


def callback_based_control():
    """Example 4: Callback-based control and monitoring."""
    print("\n" + "=" * 60)
    print("Example 4: Callback-Based Control")
    print("=" * 60)

    stage = get_current_stage()

    # Build valve
    builder = IsaacSceneBuilder(stage)
    valve_path = builder.create_valve(parent_path="/World", valve_name="CallbackValve")

    # Connect
    connector = IsaacSteveConnector(stage=stage, device_ip=STEVE_IP, prim_path=valve_path)

    # Define callbacks
    def on_hw_update(hw_state):
        """Called when hardware state updates."""
        position = hw_state.get("position_deg", 0)

        # Apply control logic
        if position > 80:
            print(f"  Warning: Position high ({position:.1f}°)")
        elif position < 10:
            print(f"  Warning: Position low ({position:.1f}°)")

    def on_sim_update(sim_state):
        """Called when simulation state updates."""
        pass

    # Register callbacks
    connector.register_hw_update_callback(on_hw_update)
    connector.register_sim_update_callback(on_sim_update)

    try:
        connector.connect()
        connector.start()

        print("\nRunning with callbacks...")
        print("Callbacks will monitor hardware state")

        for i in range(500):
            connector.update()
            time.sleep(0.02)

        print("\nCallback-based control complete!")

    finally:
        connector.disconnect()


def main():
    """Run all examples."""
    print("\n" + "=" * 60)
    print("STEVE Isaac Sim Integration Examples")
    print("=" * 60)
    print("\nNote: These examples require NVIDIA Isaac Sim to be running.")
    print("Launch Isaac Sim and run this script from the Isaac Sim Python console.\n")

    try:
        # Run examples
        basic_isaac_integration()
        time.sleep(2)

        # Uncomment for multi-valve (requires multiple devices)
        # multi_valve_coordination()
        # time.sleep(2)

        usd_config_sync_example()
        time.sleep(2)

        callback_based_control()

        print("\n" + "=" * 60)
        print("All examples completed!")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\n\nExamples interrupted by user.")
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    main()

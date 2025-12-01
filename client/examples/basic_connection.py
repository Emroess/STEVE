"""
Basic connection and control example.

Demonstrates connecting to STEVE, enabling the motor, starting the valve,
monitoring status, and adjusting parameters in real-time.
"""

import time
from pysteve import SteveClient, RealtimeTuner

# Configuration
STEVE_IP = "192.168.1.100"  # Change to your STEVE device IP


def main():
    """Main example function."""
    print("PySteve Basic Connection Example")
    print("=" * 50)

    # Connect to STEVE device
    print(f"\nConnecting to STEVE at {STEVE_IP}...")
    with SteveClient(STEVE_IP) as steve:
        print("✓ Connected successfully")

        # Check connection
        if steve.ping():
            print("✓ Device responding")
        else:
            print("✗ Device not responding")
            return

        # Enable ODrive motor
        print("\nEnabling motor...")
        steve.enable_motor()
        time.sleep(1.0)
        print("✓ Motor enabled")

        # Get initial status
        status = steve.get_status()
        print(f"\nInitial Status:")
        print(f"  Mode: {status['mode']}")
        print(f"  Position: {status['pos_deg']:.2f}°")
        print(f"  Velocity: {status['vel_rad_s']:.3f} rad/s")
        print(f"  Torque: {status['torque_nm']:.3f} Nm")

        # Get current configuration
        config = steve.get_config()
        print(f"\nCurrent Configuration:")
        print(f"  Viscous: {config.viscous:.4f} N·m·s/rad")
        print(f"  Coulomb: {config.coulomb:.4f} N·m")
        print(f"  Wall Stiffness: {config.wall_stiffness:.2f} N·m/turn")
        print(f"  Wall Damping: {config.wall_damping:.4f} N·m·s/turn")

        # Load light preset (index 0)
        print("\nLoading 'light' preset...")
        steve.load_preset(0)
        time.sleep(0.5)
        print("✓ Preset loaded")

        # Start valve
        print("\nStarting valve simulation...")
        steve.start_valve()
        print("✓ Valve running")

        # Monitor for 5 seconds
        print("\nMonitoring valve (5 seconds):")
        for i in range(10):
            status = steve.get_status()
            print(
                f"  [{i+1:2d}] Pos: {status['pos_deg']:7.2f}° | "
                f"Vel: {status['vel_rad_s']:7.3f} rad/s | "
                f"Torque: {status['torque_nm']:7.3f} Nm"
            )
            time.sleep(0.5)

        # Real-time parameter tuning
        print("\nDemonstrating real-time parameter tuning...")
        tuner = RealtimeTuner(steve)

        print("  Increasing viscous damping...")
        tuner.set_viscous(0.08)
        time.sleep(2.0)

        print("  Increasing wall stiffness...")
        tuner.set_wall_stiffness(2.0)
        time.sleep(2.0)

        print("  Resetting to light preset...")
        steve.load_preset(0)
        time.sleep(2.0)

        # Update multiple parameters at once
        print("\nUpdating multiple parameters atomically...")
        tuner.update_multiple(
            viscous=0.06, coulomb=0.012, wall_stiffness=1.5, wall_damping=0.12
        )
        time.sleep(2.0)

        status = steve.get_status()
        print(f"\nFinal Status:")
        print(f"  Position: {status['pos_deg']:.2f}°")
        print(f"  Velocity: {status['vel_rad_s']:.3f} rad/s")
        print(f"  Torque: {status['torque_nm']:.3f} Nm")

        # Stop valve
        print("\nStopping valve...")
        steve.stop_valve()
        print("✓ Valve stopped")

        # Disable motor
        print("Disabling motor...")
        steve.disable_motor()
        print("✓ Motor disabled")

    print("\n" + "=" * 50)
    print("Example completed successfully!")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n\nError: {e}")
        raise

"""
Real-time data streaming example.

Demonstrates connecting to STEVE's TCP streaming server and receiving
high-speed data with callbacks.
"""

import time
from pysteve import SteveClient, SteveStreamer

# Configuration
STEVE_IP = "192.168.1.100"  # Change to your STEVE device IP
STREAM_RATE_HZ = 50  # 50 Hz streaming (20 ms interval)
DURATION_SECONDS = 10


def main():
    """Main example function."""
    print("PySteve Streaming Example")
    print("=" * 50)

    # Connect to STEVE
    print(f"\nConnecting to STEVE at {STEVE_IP}...")
    client = SteveClient(STEVE_IP)
    client.connect()
    print("✓ Connected")

    # Enable motor and start valve
    print("\nEnabling motor and starting valve...")
    client.enable_motor()
    time.sleep(1.0)
    client.start_valve(0)  # Start with light preset
    print("✓ Valve running")

    # Create streamer
    streamer = SteveStreamer(client, threadsafe=True, buffer_size=1000)

    # Define callback for streaming data
    sample_count = [0]  # Use list for mutable counter in closure

    def on_data(sample):
        """Callback for each data sample."""
        sample_count[0] += 1
        if sample_count[0] % 50 == 0:  # Print every 50 samples (1 second at 50 Hz)
            print(
                f"  [{sample_count[0]:4d}] "
                f"Pos: {sample.get('position_deg', 0):7.2f}° | "
                f"Vel: {sample.get('omega_rad_s', 0):7.3f} rad/s | "
                f"Torque: {sample.get('torque_nm', 0):7.3f} Nm | "
                f"Seq: {sample.get('seq', 0)}"
            )

    # Start streaming
    interval_ms = int(1000 / STREAM_RATE_HZ)
    print(f"\nStarting streaming at {STREAM_RATE_HZ} Hz ({interval_ms} ms interval)...")
    streamer.start_stream(interval_ms=interval_ms, callback=on_data)
    print("✓ Streaming started")

    # Let it run
    print(f"\nStreaming for {DURATION_SECONDS} seconds...")
    time.sleep(DURATION_SECONDS)

    # Stop streaming
    print("\nStopping streaming...")
    streamer.stop_stream()
    print("✓ Streaming stopped")

    # Report statistics
    print(f"\nStatistics:")
    print(f"  Total samples received: {sample_count[0]}")
    print(f"  Dropped packets: {streamer.dropped_packets}")
    print(f"  Buffer usage: {streamer.buffer_usage * 100:.1f}%")

    # Get buffered samples
    all_samples = streamer.get_all_samples()
    print(f"  Buffered samples: {len(all_samples)}")

    if all_samples:
        # Compute statistics
        positions = [s.get("position_deg", 0) for s in all_samples]
        torques = [s.get("torque_nm", 0) for s in all_samples]

        import statistics

        print(f"\nData Summary:")
        print(f"  Position - Mean: {statistics.mean(positions):.2f}° | "
              f"Std: {statistics.stdev(positions):.2f}°")
        print(f"  Torque - Mean: {statistics.mean(torques):.3f} Nm | "
              f"Max: {max(torques):.3f} Nm")

    # Cleanup
    print("\nStopping valve and disconnecting...")
    client.stop_valve()
    client.disable_motor()
    client.disconnect()
    print("✓ Done")

    print("\n" + "=" * 50)
    print("Streaming example completed!")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n\nError: {e}")
        raise

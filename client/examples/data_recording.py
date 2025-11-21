"""
Example: Data recording and visualization with STEVE.

This example demonstrates:
1. Recording valve data during operation
2. Exporting to multiple formats
3. Creating plots and visualizations
4. Statistical analysis
"""

import time
import numpy as np

from pysteve import SteveClient, SteveStreamer
from pysteve.control import DataRecorder, RealtimeTuner
from pysteve.utils import RealtimePlotter, StaticPlotter, StreamBuffer

# Configuration
STEVE_IP = "192.168.1.100"
RECORDING_DURATION_S = 10


def simple_recording_example():
    """Example 1: Basic data recording."""
    print("=" * 60)
    print("Example 1: Basic Data Recording")
    print("=" * 60)

    # Connect
    client = SteveClient(STEVE_IP)
    client.connect()
    client.load_preset("smooth")

    streamer = SteveStreamer(client)
    recorder = DataRecorder(client, streamer)

    try:
        # Start valve
        client.enable_motor()
        time.sleep(0.5)
        client.start_valve()

        # Record data
        print(f"\nRecording data for {RECORDING_DURATION_S} seconds...")
        recorder.start_recording()

        time.sleep(RECORDING_DURATION_S)

        recorder.stop_recording()
        print(f"Recording complete! Collected {recorder.get_sample_count()} samples")

        # Export to multiple formats
        print("\nExporting data...")
        recorder.export_csv("data/recording.csv")
        recorder.export_hdf5("data/recording.h5")
        recorder.export_json("data/recording.json")

        # Get statistics
        stats = recorder.get_statistics()
        print("\nStatistics:")
        print(f"  Position: {stats['position_deg']['mean']:.2f}° ± {stats['position_deg']['std']:.2f}°")
        print(
            f"  Velocity: {stats['omega_rad_s']['mean']:.3f} ± {stats['omega_rad_s']['std']:.3f} rad/s"
        )
        print(f"  Torque: {stats['torque_nm']['mean']:.3f} ± {stats['torque_nm']['std']:.3f} N·m")

    finally:
        client.stop_valve()
        client.disconnect()


def realtime_plotting_example():
    """Example 2: Real-time plotting."""
    print("\n" + "=" * 60)
    print("Example 2: Real-Time Plotting")
    print("=" * 60)

    # Connect
    client = SteveClient(STEVE_IP)
    client.connect()
    client.load_preset("medium")

    streamer = SteveStreamer(client)

    # Create plotter
    plotter = RealtimePlotter(
        max_points=500, update_interval_ms=50, fields=["position_deg", "torque_nm"]
    )

    # Register plotter callback
    streamer.register_callback(plotter.update)

    try:
        # Start valve and streaming
        client.enable_motor()
        time.sleep(0.5)
        client.start_valve()
        streamer.start_stream(interval_ms=20)

        print("\nShowing real-time plot...")
        print("Close the plot window to continue.")

        # Show plot (blocking)
        plotter.show(block=True)

    finally:
        streamer.stop_stream()
        client.stop_valve()
        client.disconnect()


def static_plotting_example():
    """Example 3: Static plots from recorded data."""
    print("\n" + "=" * 60)
    print("Example 3: Static Plotting")
    print("=" * 60)

    # Connect and record
    client = SteveClient(STEVE_IP)
    client.connect()
    client.load_preset("tight")

    streamer = SteveStreamer(client)
    recorder = DataRecorder(client, streamer)

    try:
        client.enable_motor()
        time.sleep(0.5)
        client.start_valve()

        print(f"\nRecording data for {RECORDING_DURATION_S} seconds...")
        recorder.start_recording()
        time.sleep(RECORDING_DURATION_S)
        recorder.stop_recording()

        # Get dataframe
        df = recorder.get_dataframe()
        print(f"Recorded {len(df)} samples")

        # Create static plots
        plotter = StaticPlotter()

        print("\nGenerating plots...")
        plotter.plot_timeseries(df, "plots/timeseries.png")
        plotter.plot_phase_space(df, "plots/phase_space.png")
        plotter.plot_histogram(df, "position_deg", "plots/position_hist.png")
        plotter.plot_histogram(df, "torque_nm", "plots/torque_hist.png")

        print("Saved plots to plots/")

    finally:
        client.stop_valve()
        client.disconnect()


def parameter_sweep_recording_example():
    """Example 4: Parameter sweep with data collection."""
    print("\n" + "=" * 60)
    print("Example 4: Parameter Sweep with Recording")
    print("=" * 60)

    # Connect
    client = SteveClient(STEVE_IP)
    client.connect()

    streamer = SteveStreamer(client)
    recorder = DataRecorder(client, streamer)

    try:
        client.enable_motor()
        time.sleep(0.5)
        client.start_valve()

        # Test different viscous damping values
        viscous_values = [0.05, 0.1, 0.2, 0.3]

        for viscous in viscous_values:
            print(f"\nTesting viscous={viscous:.2f}...")

            # Update config
            client.update_config(viscous=viscous)
            time.sleep(1)  # Let system settle

            # Record data
            recorder.start_recording()
            time.sleep(5)
            recorder.stop_recording()

            # Export with descriptive name
            filename = f"data/viscous_{viscous:.2f}.csv"
            recorder.export_csv(filename)

            # Get statistics
            stats = recorder.get_statistics()
            print(
                f"  Mean velocity: {stats['omega_rad_s']['mean']:.3f} rad/s, "
                f"Std: {stats['omega_rad_s']['std']:.3f}"
            )

            # Clear for next test
            recorder.clear()

        print("\nParameter sweep complete!")

    finally:
        client.stop_valve()
        client.disconnect()


def realtime_tuning_with_feedback():
    """Example 5: Real-time tuning with visual feedback."""
    print("\n" + "=" * 60)
    print("Example 5: Real-Time Tuning with Feedback")
    print("=" * 60)

    # Connect
    client = SteveClient(STEVE_IP)
    client.connect()
    client.load_preset("medium")

    streamer = SteveStreamer(client)
    tuner = RealtimeTuner(client, streamer)

    # Create buffer for monitoring
    buffer = StreamBuffer(maxlen=500)
    streamer.register_callback(buffer.add)

    try:
        client.enable_motor()
        time.sleep(0.5)
        client.start_valve()
        streamer.start_stream(interval_ms=20)

        print("\nPerforming real-time parameter tuning...")

        # Tune viscous damping
        print("\nTuning viscous damping...")
        tuner.set_parameter("viscous", 0.05)
        time.sleep(2)

        stats = buffer.get_statistics()
        print(f"  Velocity std: {stats['omega_rad_s']['std']:.3f} rad/s")

        tuner.set_parameter("viscous", 0.2)
        time.sleep(2)

        stats = buffer.get_statistics()
        print(f"  Velocity std: {stats['omega_rad_s']['std']:.3f} rad/s")

        # Tune coulomb friction
        print("\nTuning coulomb friction...")
        tuner.set_parameter("coulomb", 0.01)
        time.sleep(2)

        stats = buffer.get_statistics()
        print(f"  Mean torque: {abs(stats['torque_nm']['mean']):.3f} N·m")

        tuner.set_parameter("coulomb", 0.03)
        time.sleep(2)

        stats = buffer.get_statistics()
        print(f"  Mean torque: {abs(stats['torque_nm']['mean']):.3f} N·m")

        print("\nTuning complete!")

    finally:
        streamer.stop_stream()
        client.stop_valve()
        client.disconnect()


def main():
    """Run all examples."""
    print("\n" + "=" * 60)
    print("STEVE Data Recording and Visualization Examples")
    print("=" * 60)

    try:
        # Run examples
        simple_recording_example()
        time.sleep(2)

        realtime_plotting_example()
        time.sleep(2)

        static_plotting_example()
        time.sleep(2)

        parameter_sweep_recording_example()
        time.sleep(2)

        realtime_tuning_with_feedback()

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

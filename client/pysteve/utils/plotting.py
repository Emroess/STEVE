"""
Real-time plotting utilities for STEVE data visualization.
"""

import time
import threading
from typing import Optional, List, Dict, Any, Tuple
from collections import deque
import numpy as np

try:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
except ImportError:
    raise ImportError("matplotlib required. Install with: pip install matplotlib")


class RealtimePlotter:
    """
    Real-time plotting for STEVE valve data.
    
    Creates live updating plots with sparklines for position, velocity, torque.
    
    Args:
        max_points: Maximum points to display (default: 500)
        update_interval_ms: Plot update interval (default: 100)
        fields: List of fields to plot (default: ["position_deg", "omega_rad_s", "torque_nm"])
    
    Example:
        >>> from pysteve import SteveClient, SteveStreamer
        >>> from pysteve.utils import RealtimePlotter
        >>> 
        >>> client = SteveClient("192.168.1.100")
        >>> client.connect()
        >>> client.start_valve()
        >>> 
        >>> streamer = SteveStreamer(client)
        >>> plotter = RealtimePlotter()
        >>> 
        >>> # Register plotter callback
        >>> streamer.register_callback(plotter.update)
        >>> 
        >>> # Start streaming and show plot
        >>> streamer.start_stream()
        >>> plotter.show()
    """

    def __init__(
        self,
        max_points: int = 500,
        update_interval_ms: int = 100,
        fields: Optional[List[str]] = None,
    ):
        self.max_points = max_points
        self.update_interval_ms = update_interval_ms
        self.fields = fields or ["position_deg", "omega_rad_s", "torque_nm"]

        # Data buffers
        self.timestamps: deque = deque(maxlen=max_points)
        self.data: Dict[str, deque] = {field: deque(maxlen=max_points) for field in self.fields}

        # Thread safety
        self._lock = threading.Lock()

        # Plot objects
        self.fig = None
        self.axes = []
        self.lines = []
        self._animation = None

        # Start time
        self._start_time = time.time()

    def update(self, sample: Dict[str, Any]) -> None:
        """
        Update plot with new sample.
        
        Args:
            sample: Data sample dict
        """
        with self._lock:
            # Get relative timestamp
            timestamp = sample.get("timestamp_ms", 0) / 1000.0

            self.timestamps.append(timestamp)

            # Add data for each field
            for field in self.fields:
                value = sample.get(field, 0)
                self.data[field].append(value)

    def _init_plot(self) -> None:
        """Initialize plot figure."""
        num_plots = len(self.fields)

        self.fig, self.axes = plt.subplots(num_plots, 1, figsize=(10, 2 * num_plots))

        if num_plots == 1:
            self.axes = [self.axes]

        # Create line for each subplot
        for ax, field in zip(self.axes, self.fields):
            (line,) = ax.plot([], [], "b-", linewidth=1.5)
            self.lines.append(line)

            # Configure subplot
            ax.set_ylabel(field)
            ax.grid(True, alpha=0.3)

        self.axes[-1].set_xlabel("Time (s)")
        self.fig.suptitle("STEVE Valve Real-Time Data")

    def _update_plot(self, frame: int) -> List:
        """Animation update function."""
        with self._lock:
            if not self.timestamps:
                return self.lines

            # Get data
            times = list(self.timestamps)

            # Update each line
            for line, field in zip(self.lines, self.fields):
                values = list(self.data[field])

                if len(times) == len(values):
                    line.set_data(times, values)

                    # Update axis limits
                    ax = line.axes
                    ax.set_xlim(min(times), max(times))

                    if values:
                        value_range = max(values) - min(values)
                        if value_range > 0:
                            margin = value_range * 0.1
                            ax.set_ylim(min(values) - margin, max(values) + margin)

        return self.lines

    def show(self, block: bool = True) -> None:
        """
        Show plot window.
        
        Args:
            block: Block execution until window closed
        """
        self._init_plot()

        # Create animation
        self._animation = FuncAnimation(
            self.fig,
            self._update_plot,
            interval=self.update_interval_ms,
            blit=False,
        )

        plt.tight_layout()
        plt.show(block=block)

    def save_plot(self, filename: str) -> None:
        """
        Save current plot to file.
        
        Args:
            filename: Output file path (PNG, PDF, SVG, etc.)
        """
        if self.fig is None:
            self._init_plot()
            self._update_plot(0)

        self.fig.savefig(filename, dpi=300, bbox_inches="tight")
        print(f"Saved plot to {filename}")

    def clear(self) -> None:
        """Clear all data buffers."""
        with self._lock:
            self.timestamps.clear()
            for field in self.fields:
                self.data[field].clear()


class StaticPlotter:
    """
    Static plotting for recorded STEVE data.
    
    Create publication-quality plots from recorded data.
    
    Args:
        figsize: Figure size (width, height)
        style: Matplotlib style name
    
    Example:
        >>> from pysteve.control import DataRecorder
        >>> from pysteve.utils import StaticPlotter
        >>> 
        >>> # Record data
        >>> recorder = DataRecorder(client, streamer)
        >>> recorder.start_recording()
        >>> # ... run experiment ...
        >>> recorder.stop_recording()
        >>> 
        >>> # Plot results
        >>> df = recorder.get_dataframe()
        >>> plotter = StaticPlotter()
        >>> plotter.plot_timeseries(df, "experiment_plot.png")
    """

    def __init__(
        self,
        figsize: Tuple[float, float] = (12, 8),
        style: str = "seaborn-v0_8-darkgrid",
    ):
        self.figsize = figsize
        try:
            plt.style.use(style)
        except:
            pass  # Style not available

    def plot_timeseries(
        self,
        data: Any,
        filename: Optional[str] = None,
        fields: Optional[List[str]] = None,
    ) -> None:
        """
        Plot time series data.
        
        Args:
            data: pandas DataFrame with time-indexed data
            filename: Output file path (optional)
            fields: List of fields to plot
        """
        if fields is None:
            fields = ["position_deg", "omega_rad_s", "torque_nm"]

        fig, axes = plt.subplots(len(fields), 1, figsize=self.figsize)

        if len(fields) == 1:
            axes = [axes]

        # Get time column
        if "timestamp_ms" in data.columns:
            time_s = data["timestamp_ms"] / 1000.0
            time_s = time_s - time_s.iloc[0]  # Start at 0
        else:
            time_s = np.arange(len(data)) / 100.0  # Assume 100 Hz

        # Plot each field
        for ax, field in zip(axes, fields):
            if field in data.columns:
                ax.plot(time_s, data[field], linewidth=1.5)
                ax.set_ylabel(field)
                ax.grid(True, alpha=0.3)

        axes[-1].set_xlabel("Time (s)")
        fig.suptitle("STEVE Valve Time Series")

        plt.tight_layout()

        if filename:
            plt.savefig(filename, dpi=300, bbox_inches="tight")
            print(f"Saved plot to {filename}")
        else:
            plt.show()

    def plot_phase_space(
        self, data: Any, filename: Optional[str] = None
    ) -> None:
        """
        Plot phase space (position vs velocity).
        
        Args:
            data: pandas DataFrame with data
            filename: Output file path (optional)
        """
        fig, ax = plt.subplots(figsize=(8, 8))

        position = data["position_deg"]
        velocity = data["omega_rad_s"]

        # Plot trajectory
        ax.plot(position, velocity, linewidth=1, alpha=0.7)

        # Mark start and end
        ax.plot(position.iloc[0], velocity.iloc[0], "go", markersize=10, label="Start")
        ax.plot(position.iloc[-1], velocity.iloc[-1], "ro", markersize=10, label="End")

        ax.set_xlabel("Position (deg)")
        ax.set_ylabel("Velocity (rad/s)")
        ax.set_title("Phase Space Trajectory")
        ax.grid(True, alpha=0.3)
        ax.legend()

        plt.tight_layout()

        if filename:
            plt.savefig(filename, dpi=300, bbox_inches="tight")
            print(f"Saved plot to {filename}")
        else:
            plt.show()

    def plot_histogram(
        self, data: Any, field: str, filename: Optional[str] = None, bins: int = 50
    ) -> None:
        """
        Plot histogram of field values.
        
        Args:
            data: pandas DataFrame with data
            field: Field name to plot
            filename: Output file path (optional)
            bins: Number of histogram bins
        """
        fig, ax = plt.subplots(figsize=(8, 6))

        values = data[field]

        ax.hist(values, bins=bins, alpha=0.7, edgecolor="black")
        ax.set_xlabel(field)
        ax.set_ylabel("Count")
        ax.set_title(f"Distribution of {field}")
        ax.grid(True, alpha=0.3, axis="y")

        # Add statistics text
        stats_text = f"Mean: {values.mean():.2f}\nStd: {values.std():.2f}\nMin: {values.min():.2f}\nMax: {values.max():.2f}"
        ax.text(
            0.98,
            0.98,
            stats_text,
            transform=ax.transAxes,
            verticalalignment="top",
            horizontalalignment="right",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
        )

        plt.tight_layout()

        if filename:
            plt.savefig(filename, dpi=300, bbox_inches="tight")
            print(f"Saved plot to {filename}")
        else:
            plt.show()

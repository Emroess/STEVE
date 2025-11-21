"""
Automated parameter sweep and testing utilities.
"""

import time
from typing import List, Dict, Any, Optional, Callable
import itertools

from pysteve.core.exceptions import SteveValidationError


class ParameterSweep:
    """
    Automated parameter sweep for batch testing and optimization.
    
    Systematically tests parameter combinations while recording valve
    behavior and performance metrics.
    
    Args:
        client: SteveClient instance
        streamer: Optional SteveStreamer for high-rate data collection
    
    Example:
        >>> from pysteve import SteveClient, SteveStreamer
        >>> from pysteve.control import ParameterSweep
        >>> 
        >>> client = SteveClient("192.168.1.100")
        >>> sweep = ParameterSweep(client)
        >>> 
        >>> results = sweep.sweep_range(
        ...     "viscous",
        ...     start=0.02,
        ...     end=0.10,
        ...     steps=5,
        ...     dwell_time=2.0
        ... )
        >>> sweep.export_results(results, "viscous_sweep.csv")
    """

    def __init__(
        self,
        client: "SteveClient",  # type: ignore
        streamer: Optional["SteveStreamer"] = None,  # type: ignore
    ):
        self.client = client
        self.streamer = streamer

    def sweep_range(
        self,
        param_name: str,
        start: float,
        end: float,
        steps: int,
        dwell_time: float = 1.0,
        progress_callback: Optional[Callable[[int, int, float], None]] = None,
    ) -> List[Dict[str, Any]]:
        """
        Sweep single parameter across linear range.
        
        Args:
            param_name: Parameter to sweep
            start: Starting value
            end: Ending value
            steps: Number of steps
            dwell_time: Time at each value [s]
            progress_callback: Optional callback(step, total, value)
        
        Returns:
            List of result dictionaries with metrics at each value
        """
        import numpy as np

        values = np.linspace(start, end, steps)
        results = []

        for i, value in enumerate(values):
            # Update parameter
            self.client.update_config(**{param_name: value})
            time.sleep(0.1)  # Allow parameter update

            # Collect data during dwell
            samples = []
            if self.streamer and self.streamer.is_streaming:
                self.streamer.clear_buffer()
                time.sleep(dwell_time)
                samples = self.streamer.get_all_samples()
            else:
                # Poll at 10 Hz
                for _ in range(int(dwell_time * 10)):
                    samples.append(self.client.get_status())
                    time.sleep(0.1)

            # Compute metrics
            metrics = self._compute_metrics(samples)
            metrics["param_name"] = param_name
            metrics["param_value"] = float(value)
            results.append(metrics)

            if progress_callback:
                progress_callback(i + 1, steps, value)

        return results

    def sweep_grid(
        self,
        param_grid: Dict[str, List[float]],
        dwell_time: float = 1.0,
        progress_callback: Optional[Callable[[int, int, Dict], None]] = None,
    ) -> List[Dict[str, Any]]:
        """
        Sweep multiple parameters in grid pattern.
        
        Args:
            param_grid: Dict mapping param names to lists of values
            dwell_time: Time at each combination [s]
            progress_callback: Optional callback(step, total, params)
        
        Returns:
            List of result dictionaries
        
        Example:
            >>> results = sweep.sweep_grid({
            ...     "viscous": [0.02, 0.05, 0.08],
            ...     "coulomb": [0.005, 0.010, 0.015]
            ... })
        """
        # Generate all combinations
        param_names = list(param_grid.keys())
        value_lists = [param_grid[name] for name in param_names]
        combinations = list(itertools.product(*value_lists))
        total = len(combinations)

        results = []

        for i, values in enumerate(combinations):
            params = dict(zip(param_names, values))

            # Update all parameters
            self.client.update_config(**params)
            time.sleep(0.1)

            # Collect data
            samples = []
            if self.streamer and self.streamer.is_streaming:
                self.streamer.clear_buffer()
                time.sleep(dwell_time)
                samples = self.streamer.get_all_samples()
            else:
                for _ in range(int(dwell_time * 10)):
                    samples.append(self.client.get_status())
                    time.sleep(0.1)

            # Compute metrics
            metrics = self._compute_metrics(samples)
            metrics.update(params)
            results.append(metrics)

            if progress_callback:
                progress_callback(i + 1, total, params)

        return results

    def _compute_metrics(self, samples: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Compute summary metrics from collected samples."""
        import numpy as np

        if not samples:
            return {}

        # Extract time series
        positions = []
        velocities = []
        torques = []

        for s in samples:
            if "pos_deg" in s:
                positions.append(s["pos_deg"])
            elif "position_deg" in s:
                positions.append(s["position_deg"])

            if "vel_rad_s" in s:
                velocities.append(s["vel_rad_s"])
            elif "omega_rad_s" in s:
                velocities.append(s["omega_rad_s"])

            if "torque_nm" in s:
                torques.append(s["torque_nm"])

        # Compute statistics
        metrics = {
            "num_samples": len(samples),
        }

        if positions:
            metrics.update(
                {
                    "pos_mean": float(np.mean(positions)),
                    "pos_std": float(np.std(positions)),
                    "pos_min": float(np.min(positions)),
                    "pos_max": float(np.max(positions)),
                }
            )

        if velocities:
            metrics.update(
                {
                    "vel_mean": float(np.mean(np.abs(velocities))),
                    "vel_std": float(np.std(velocities)),
                    "vel_max": float(np.max(np.abs(velocities))),
                }
            )

        if torques:
            metrics.update(
                {
                    "torque_mean": float(np.mean(np.abs(torques))),
                    "torque_std": float(np.std(torques)),
                    "torque_max": float(np.max(np.abs(torques))),
                    "torque_rms": float(np.sqrt(np.mean(np.square(torques)))),
                }
            )

        # Energy metrics
        if velocities and torques and len(velocities) == len(torques):
            power = np.array(velocities) * np.array(torques)
            metrics["power_mean"] = float(np.mean(np.abs(power)))
            metrics["energy_total"] = float(np.sum(np.abs(power)) / len(samples))

        return metrics

    def export_results(self, results: List[Dict[str, Any]], filepath: str) -> None:
        """
        Export sweep results to CSV file.
        
        Args:
            results: Results from sweep_range() or sweep_grid()
            filepath: Output file path
        """
        import csv

        if not results:
            return

        # Get all keys
        keys = set()
        for r in results:
            keys.update(r.keys())
        keys = sorted(keys)

        # Write CSV
        with open(filepath, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(results)

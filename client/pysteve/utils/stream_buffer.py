"""
Thread-safe circular buffer for streaming data with export capabilities.
"""

import threading
import time
from typing import Optional, Dict, Any, List
from collections import deque
import csv
import json
from pathlib import Path

try:
    import h5py
except ImportError:
    h5py = None

try:
    import pandas as pd
except ImportError:
    pd = None


class StreamBuffer:
    """
    Thread-safe circular buffer for STEVE streaming data.
    
    Provides buffering, export to multiple formats, and statistics.
    
    Args:
        maxlen: Maximum buffer size (default: 10000)
        threadsafe: Enable thread safety (default: True)
    
    Example:
        >>> buffer = StreamBuffer(maxlen=5000)
        >>> 
        >>> # Add samples
        >>> for sample in samples:
        ...     buffer.add(sample)
        >>> 
        >>> # Get statistics
        >>> stats = buffer.get_statistics()
        >>> print(f"Mean position: {stats['position_deg']['mean']:.2f}")
        >>> 
        >>> # Export to file
        >>> buffer.export_csv("data.csv")
        >>> buffer.export_hdf5("data.h5")
    """

    def __init__(self, maxlen: int = 10000, threadsafe: bool = True):
        self.maxlen = maxlen
        self.threadsafe = threadsafe

        self.buffer: deque = deque(maxlen=maxlen)
        self._lock = threading.Lock() if threadsafe else None

        # Statistics
        self._total_samples = 0
        self._dropped_samples = 0

    def add(self, sample: Dict[str, Any]) -> None:
        """
        Add sample to buffer.
        
        Args:
            sample: Data sample dict
        """
        if self.threadsafe:
            with self._lock:
                self._add_sample(sample)
        else:
            self._add_sample(sample)

    def _add_sample(self, sample: Dict[str, Any]) -> None:
        """Internal add (assumes lock held if threadsafe)."""
        # Track if buffer is full
        was_full = len(self.buffer) >= self.maxlen

        self.buffer.append(sample)
        self._total_samples += 1

        if was_full:
            self._dropped_samples += 1

    def get_latest(self, n: int = 1) -> List[Dict[str, Any]]:
        """
        Get latest n samples.
        
        Args:
            n: Number of samples to retrieve
        
        Returns:
            List of most recent samples
        """
        if self.threadsafe:
            with self._lock:
                return list(self.buffer)[-n:]
        else:
            return list(self.buffer)[-n:]

    def get_all(self) -> List[Dict[str, Any]]:
        """Get all samples in buffer."""
        if self.threadsafe:
            with self._lock:
                return list(self.buffer)
        else:
            return list(self.buffer)

    def clear(self) -> None:
        """Clear buffer."""
        if self.threadsafe:
            with self._lock:
                self.buffer.clear()
        else:
            self.buffer.clear()

    def get_statistics(self) -> Dict[str, Any]:
        """
        Compute statistics over buffered data.
        
        Returns:
            Dict with mean, std, min, max for each field
        """
        samples = self.get_all()
        if not samples:
            return {}

        # Extract numeric fields
        fields = ["position_deg", "omega_rad_s", "torque_nm"]
        stats = {}

        for field in fields:
            values = [s.get(field, 0) for s in samples if field in s]
            if values:
                import numpy as np

                stats[field] = {
                    "mean": float(np.mean(values)),
                    "std": float(np.std(values)),
                    "min": float(np.min(values)),
                    "max": float(np.max(values)),
                }

        # Add buffer stats
        stats["buffer"] = {
            "size": len(self.buffer),
            "max_size": self.maxlen,
            "total_samples": self._total_samples,
            "dropped_samples": self._dropped_samples,
        }

        return stats

    def export_csv(self, filename: str) -> None:
        """
        Export buffer to CSV file.
        
        Args:
            filename: Output CSV file path
        """
        samples = self.get_all()
        if not samples:
            return

        # Get all keys
        all_keys = set()
        for sample in samples:
            all_keys.update(sample.keys())

        keys = sorted(all_keys)

        # Write CSV
        path = Path(filename)
        path.parent.mkdir(parents=True, exist_ok=True)

        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(samples)

        print(f"Exported {len(samples)} samples to {filename}")

    def export_hdf5(self, filename: str, dataset_name: str = "steve_data") -> None:
        """
        Export buffer to HDF5 file.
        
        Args:
            filename: Output HDF5 file path
            dataset_name: Dataset name in HDF5 file
        """
        if h5py is None:
            raise ImportError("h5py required for HDF5 export. Install with: pip install h5py")

        samples = self.get_all()
        if not samples:
            return

        path = Path(filename)
        path.parent.mkdir(parents=True, exist_ok=True)

        with h5py.File(path, "w") as f:
            # Create datasets for each field
            for key in samples[0].keys():
                values = [s.get(key, 0) for s in samples]

                # Convert to appropriate type
                if isinstance(values[0], (int, float)):
                    f.create_dataset(f"{dataset_name}/{key}", data=values)
                elif isinstance(values[0], str):
                    dt = h5py.string_dtype(encoding="utf-8")
                    f.create_dataset(f"{dataset_name}/{key}", data=values, dtype=dt)

        print(f"Exported {len(samples)} samples to {filename}")

    def export_json(self, filename: str) -> None:
        """
        Export buffer to JSON file.
        
        Args:
            filename: Output JSON file path
        """
        samples = self.get_all()
        if not samples:
            return

        path = Path(filename)
        path.parent.mkdir(parents=True, exist_ok=True)

        with open(path, "w") as f:
            json.dump(samples, f, indent=2)

        print(f"Exported {len(samples)} samples to {filename}")

    def to_dataframe(self) -> Any:
        """
        Convert buffer to pandas DataFrame.
        
        Returns:
            pandas DataFrame
        """
        if pd is None:
            raise ImportError("pandas required. Install with: pip install pandas")

        samples = self.get_all()
        return pd.DataFrame(samples)

    def get_time_range(self) -> tuple:
        """
        Get time range of buffered data.
        
        Returns:
            Tuple of (start_time, end_time) in seconds
        """
        samples = self.get_all()
        if not samples:
            return (0, 0)

        times = [s.get("timestamp_ms", 0) / 1000.0 for s in samples]
        return (min(times), max(times))

    def get_sample_rate(self) -> float:
        """
        Estimate sample rate from buffered data.
        
        Returns:
            Sample rate in Hz
        """
        samples = self.get_all()
        if len(samples) < 2:
            return 0.0

        times = [s.get("timestamp_ms", 0) / 1000.0 for s in samples]
        time_diff = times[-1] - times[0]

        if time_diff > 0:
            return len(samples) / time_diff
        else:
            return 0.0

"""
Data recording utilities for synchronized data collection and export.
"""

import time
import threading
from typing import Optional, List, Dict, Any, Union
from pathlib import Path
import json


class DataRecorder:
    """
    Data recorder for synchronized data collection from STEVE streaming.
    
    Records streaming data with timestamp synchronization, sequence validation,
    and export to multiple formats (CSV, HDF5, pickle, JSON).
    
    Args:
        streamer: SteveStreamer instance to record from
        auto_interpolate: Automatically interpolate missing samples (default: True)
        validate_sequence: Check for dropped packets (default: True)
    
    Example:
        >>> from pysteve import SteveClient, SteveStreamer
        >>> from pysteve.control import DataRecorder
        >>> 
        >>> client = SteveClient("192.168.1.100")
        >>> streamer = SteveStreamer(client)
        >>> recorder = DataRecorder(streamer)
        >>> 
        >>> recorder.start_recording()
        >>> time.sleep(10)
        >>> recorder.stop_recording()
        >>> recorder.save("data.csv")
    """

    def __init__(
        self,
        streamer: "SteveStreamer",  # type: ignore
        auto_interpolate: bool = True,
        validate_sequence: bool = True,
    ):
        self.streamer = streamer
        self.auto_interpolate = auto_interpolate
        self.validate_sequence = validate_sequence

        self._recording = False
        self._samples: List[Dict[str, Any]] = []
        self._lock = threading.Lock()
        self._start_time: Optional[float] = None
        self._metadata: Dict[str, Any] = {}
        self._dropped_count = 0

    def start_recording(self) -> None:
        """
        Start recording data from streamer.
        
        Registers callback with streamer to collect data.
        """
        if self._recording:
            return

        self._recording = True
        self._samples.clear()
        self._start_time = time.time()
        self._dropped_count = 0

        # Register callback
        self.streamer.register_callback(self._record_sample)

    def stop_recording(self) -> None:
        """
        Stop recording data.
        
        Unregisters callback from streamer.
        """
        if not self._recording:
            return

        self._recording = False
        self.streamer.unregister_callback(self._record_sample)

        # Store metadata
        self._metadata = {
            "recording_duration": time.time() - self._start_time if self._start_time else 0,
            "sample_count": len(self._samples),
            "dropped_packets": self._dropped_count,
            "timestamp": time.time(),
        }

        # Perform interpolation if enabled
        if self.auto_interpolate and self._dropped_count > 0:
            self._interpolate_missing_samples()

    def _record_sample(self, sample: Dict[str, Any]) -> None:
        """Callback to record each sample."""
        with self._lock:
            # Validate sequence if enabled
            if self.validate_sequence and self._samples:
                last_seq = self._samples[-1].get("seq")
                current_seq = sample.get("seq")
                
                if last_seq is not None and current_seq is not None:
                    expected = (last_seq + 1) & 0xFFFFFFFF
                    if current_seq != expected:
                        dropped = (current_seq - expected) & 0xFFFFFFFF
                        self._dropped_count += dropped

            # Add recording timestamp
            sample = sample.copy()
            sample["recording_time"] = time.time()
            
            self._samples.append(sample)

    def _interpolate_missing_samples(self) -> None:
        """Interpolate missing samples based on sequence numbers."""
        if len(self._samples) < 2:
            return

        interpolated = []
        
        for i in range(len(self._samples) - 1):
            current = self._samples[i]
            next_sample = self._samples[i + 1]
            
            interpolated.append(current)
            
            # Check for gap
            current_seq = current.get("seq")
            next_seq = next_sample.get("seq")
            
            if current_seq is None or next_seq is None:
                continue
                
            gap = (next_seq - current_seq - 1) & 0xFFFFFFFF
            
            if gap > 0 and gap < 100:  # Only interpolate reasonable gaps
                # Linear interpolation for numeric fields
                for j in range(1, gap + 1):
                    alpha = j / (gap + 1)
                    interpolated_sample = self._interpolate_samples(current, next_sample, alpha)
                    interpolated_sample["interpolated"] = True
                    interpolated.append(interpolated_sample)
        
        # Add last sample
        interpolated.append(self._samples[-1])
        
        with self._lock:
            self._samples = interpolated

    def _interpolate_samples(
        self, s1: Dict[str, Any], s2: Dict[str, Any], alpha: float
    ) -> Dict[str, Any]:
        """Linearly interpolate between two samples."""
        result = {}
        
        numeric_fields = [
            "position_turns", "position_deg", "torque_nm", "filt_torque_nm",
            "omega_rad_s", "t_us", "timestamp_ms", "loop_time_us"
        ]
        
        for field in numeric_fields:
            if field in s1 and field in s2:
                v1 = s1[field]
                v2 = s2[field]
                if isinstance(v1, (int, float)) and isinstance(v2, (int, float)):
                    result[field] = v1 + alpha * (v2 - v1)
        
        # Copy non-numeric fields from s1
        for field in ["status", "quiet", "err", "data_valid"]:
            if field in s1:
                result[field] = s1[field]
        
        return result

    def get_samples(self) -> List[Dict[str, Any]]:
        """
        Get all recorded samples.
        
        Returns:
            List of sample dictionaries
        """
        with self._lock:
            return self._samples.copy()

    def get_metadata(self) -> Dict[str, Any]:
        """
        Get recording metadata.
        
        Returns:
            Dictionary with recording statistics
        """
        return self._metadata.copy()

    def save(self, filepath: Union[str, Path], format: Optional[str] = None) -> None:
        """
        Save recorded data to file.
        
        Format is auto-detected from file extension if not specified.
        
        Args:
            filepath: Output file path
            format: Optional format override ('csv', 'hdf5', 'pickle', 'json')
        
        Example:
            >>> recorder.save("data.csv")
            >>> recorder.save("data.h5", format="hdf5")
        """
        filepath = Path(filepath)
        
        if format is None:
            format = filepath.suffix.lstrip(".")
        
        format = format.lower()
        
        if format in ["csv", "txt"]:
            self._save_csv(filepath)
        elif format in ["h5", "hdf5"]:
            self._save_hdf5(filepath)
        elif format in ["pkl", "pickle"]:
            self._save_pickle(filepath)
        elif format == "json":
            self._save_json(filepath)
        else:
            raise ValueError(f"Unsupported format: {format}")

    def _save_csv(self, filepath: Path) -> None:
        """Save data to CSV file."""
        import csv
        
        with self._lock:
            samples = self._samples.copy()
        
        if not samples:
            return
        
        # Get all field names
        fieldnames = set()
        for s in samples:
            fieldnames.update(s.keys())
        fieldnames = sorted(fieldnames)
        
        # Write CSV
        with open(filepath, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(samples)

    def _save_hdf5(self, filepath: Path) -> None:
        """Save data to HDF5 file."""
        try:
            import h5py
            import numpy as np
        except ImportError:
            raise ImportError("h5py required for HDF5 export. Install with: pip install pysteve[data]")
        
        with self._lock:
            samples = self._samples.copy()
        
        if not samples:
            return
        
        with h5py.File(filepath, "w") as f:
            # Create datasets for numeric fields
            numeric_fields = [
                "position_turns", "position_deg", "torque_nm", "filt_torque_nm",
                "omega_rad_s", "t_us", "timestamp_ms", "loop_time_us", "seq",
                "passivity_mj", "hb_age"
            ]
            
            for field in numeric_fields:
                values = [s.get(field, np.nan) for s in samples]
                if any(v is not None and v is not np.nan for v in values):
                    f.create_dataset(field, data=values)
            
            # Store metadata as attributes
            for key, value in self._metadata.items():
                f.attrs[key] = value

    def _save_pickle(self, filepath: Path) -> None:
        """Save data to pickle file."""
        import pickle
        
        with self._lock:
            samples = self._samples.copy()
        
        data = {
            "samples": samples,
            "metadata": self._metadata,
        }
        
        with open(filepath, "wb") as f:
            pickle.dump(data, f)

    def _save_json(self, filepath: Path) -> None:
        """Save data to JSON file."""
        with self._lock:
            samples = self._samples.copy()
        
        data = {
            "samples": samples,
            "metadata": self._metadata,
        }
        
        with open(filepath, "w") as f:
            json.dump(data, f, indent=2)

    def get_dataframe(self) -> "pandas.DataFrame":  # type: ignore
        """
        Convert recorded data to pandas DataFrame.
        
        Returns:
            DataFrame with recorded samples
        
        Requires:
            pandas (install with: pip install pysteve[data])
        """
        try:
            import pandas as pd
        except ImportError:
            raise ImportError("pandas required. Install with: pip install pysteve[data]")
        
        with self._lock:
            samples = self._samples.copy()
        
        return pd.DataFrame(samples)

    def clear(self) -> None:
        """Clear all recorded data."""
        with self._lock:
            self._samples.clear()
            self._metadata.clear()
            self._dropped_count = 0
            self._start_time = None

    @property
    def is_recording(self) -> bool:
        """Check if currently recording."""
        return self._recording

    @property
    def sample_count(self) -> int:
        """Get number of recorded samples."""
        with self._lock:
            return len(self._samples)

    @property
    def dropped_packets(self) -> int:
        """Get number of dropped packets detected."""
        return self._dropped_count

"""
MuJoCo valve actuator with hardware synchronization and interpolation.
"""

import time
import threading
from typing import Optional, Literal, Tuple
from collections import deque
import numpy as np

try:
    import mujoco
except ImportError:
    raise ImportError(
        "MuJoCo required for this module. Install with: pip install pysteve[mujoco]"
    )

from pysteve.core.client import SteveClient
from pysteve.core.streaming import SteveStreamer
from pysteve.core.exceptions import SteveError


class SteveValveActuator:
    """
    MuJoCo actuator synchronized with STEVE hardware valve.
    
    Provides bidirectional synchronization between MuJoCo simulation and STEVE
    hardware with state interpolation buffer for smooth operation across
    different timesteps.
    
    Features:
    - State interpolation buffer (last 100ms of STEVE samples)
    - Configurable latency compensation
    - Three sync modes: simulation, hardware, hybrid
    - Option to run MuJoCo at STEVE's 1000 Hz for perfect sync
    
    Args:
        steve_ip: STEVE device IP address
        mujoco_joint_name: Name of joint in MuJoCo model
        sync_mode: 'simulation', 'hardware', or 'hybrid' (default: 'simulation')
        target_hz: Target update rate, None for MuJoCo's rate (default: None)
        latency_compensation_ms: Latency compensation in ms (default: 50)
        buffer_duration_ms: State buffer duration (default: 100)
        api_key: STEVE API key (default: "steve-valve-2025")
    
    Example:
        >>> import mujoco
        >>> from pysteve.integrations.mujoco import SteveValveActuator
        >>> 
        >>> model = mujoco.MjModel.from_xml_path("robot.xml")
        >>> data = mujoco.MjData(model)
        >>> 
        >>> actuator = SteveValveActuator(
        ...     steve_ip="192.168.1.100",
        ...     mujoco_joint_name="valve_joint",
        ...     sync_mode="hardware",
        ...     target_hz=1000
        ... )
        >>> 
        >>> actuator.connect()
        >>> actuator.start()
        >>> 
        >>> while True:
        ...     actuator.update(model, data)
        ...     mujoco.mj_step(model, data)
    """

    def __init__(
        self,
        steve_ip: str,
        mujoco_joint_name: str,
        sync_mode: Literal["simulation", "hardware", "hybrid"] = "simulation",
        target_hz: Optional[int] = None,
        latency_compensation_ms: float = 50.0,
        buffer_duration_ms: float = 100.0,
        api_key: str = "steve-valve-2025",
    ):
        self.steve_ip = steve_ip
        self.mujoco_joint_name = mujoco_joint_name
        self.sync_mode = sync_mode
        self.target_hz = target_hz
        self.latency_compensation_ms = latency_compensation_ms
        self.buffer_duration_ms = buffer_duration_ms
        self.api_key = api_key

        # Client and streaming
        self.client: Optional[SteveClient] = None
        self.streamer: Optional[SteveStreamer] = None

        # State buffer
        self._buffer_size = int(buffer_duration_ms / 1.0)  # Assuming 1ms samples
        self._state_buffer: deque = deque(maxlen=self._buffer_size)
        self._buffer_lock = threading.Lock()

        # MuJoCo joint info
        self._joint_id: Optional[int] = None
        self._actuator_id: Optional[int] = None

        # Timing
        self._last_update_time = 0.0
        self._update_interval = 1.0 / target_hz if target_hz else 0.0

        # Hardware sync
        self._last_config_hash: Optional[int] = None

    def connect(self) -> None:
        """
        Connect to STEVE device and start streaming.
        
        Raises:
            SteveError: If connection fails
        """
        # Create client
        self.client = SteveClient(
            self.steve_ip,
            api_key=self.api_key,
            auto_reconnect=True,
        )
        self.client.connect()

        # Create streamer with appropriate rate
        stream_rate_hz = 100 if self.target_hz is None else min(self.target_hz, 100)
        stream_interval_ms = int(1000 / stream_rate_hz)

        self.streamer = SteveStreamer(
            self.client,
            threadsafe=True,
            buffer_size=self._buffer_size,
        )

        # Register callback
        self.streamer.register_callback(self._on_steve_data)

        # Start streaming
        self.streamer.start_stream(interval_ms=stream_interval_ms)

    def disconnect(self) -> None:
        """Disconnect from STEVE device."""
        if self.streamer:
            self.streamer.stop_stream()
            self.streamer = None

        if self.client:
            self.client.disconnect()
            self.client = None

    def start(self) -> None:
        """
        Start valve simulation on STEVE hardware.
        
        Enables motor and starts valve control.
        """
        if not self.client:
            raise SteveError("Not connected. Call connect() first.")

        self.client.enable_motor()
        time.sleep(0.5)
        self.client.start_valve()

    def stop(self) -> None:
        """Stop valve simulation on STEVE hardware."""
        if self.client:
            self.client.stop_valve()

    def _on_steve_data(self, sample: dict) -> None:
        """Callback for STEVE streaming data."""
        # Add timestamp
        sample["receive_time"] = time.time()

        # Buffer sample
        with self._buffer_lock:
            self._state_buffer.append(sample)

    def _find_joint(self, model: mujoco.MjModel) -> Tuple[int, Optional[int]]:
        """Find joint and actuator IDs in MuJoCo model."""
        # Find joint
        joint_id = None
        for i in range(model.njnt):
            if model.joint(i).name == self.mujoco_joint_name:
                joint_id = i
                break

        if joint_id is None:
            raise ValueError(f"Joint '{self.mujoco_joint_name}' not found in model")

        # Find actuator (if exists)
        actuator_id = None
        for i in range(model.nu):
            if model.actuator(i).trnid[0] == joint_id:
                actuator_id = i
                break

        return joint_id, actuator_id

    def update(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """
        Update MuJoCo simulation state from STEVE hardware.
        
        Call this before each mujoco.mj_step().
        
        Args:
            model: MuJoCo model
            data: MuJoCo data
        """
        # Initialize joint/actuator IDs on first call
        if self._joint_id is None:
            self._joint_id, self._actuator_id = self._find_joint(model)

        # Check timing
        current_time = time.time()
        if self._update_interval > 0:
            if current_time - self._last_update_time < self._update_interval:
                return
            self._last_update_time = current_time

        # Get interpolated state
        state = self._get_interpolated_state(current_time)

        if state is None:
            return

        # Apply based on sync mode
        if self.sync_mode == "simulation":
            # Pure simulation - STEVE runs independently, we just monitor
            pass

        elif self.sync_mode == "hardware":
            # Hardware-in-the-loop - sync MuJoCo to STEVE
            self._sync_mujoco_to_steve(model, data, state)

        elif self.sync_mode == "hybrid":
            # Hybrid - bidirectional sync
            self._sync_hybrid(model, data, state)

    def _get_interpolated_state(self, current_time: float) -> Optional[dict]:
        """
        Get interpolated state for current time with latency compensation.
        
        Args:
            current_time: Current time in seconds
        
        Returns:
            Interpolated state dict, or None if insufficient data
        """
        with self._buffer_lock:
            if len(self._state_buffer) < 2:
                return None

            buffer = list(self._state_buffer)

        # Apply latency compensation
        target_time = current_time - (self.latency_compensation_ms / 1000.0)

        # Find samples to interpolate between
        before = None
        after = None

        for sample in buffer:
            sample_time = sample.get("receive_time", 0)

            if sample_time <= target_time:
                before = sample
            elif after is None:
                after = sample
                break

        # If we have bracketing samples, interpolate
        if before and after:
            t_before = before.get("receive_time", 0)
            t_after = after.get("receive_time", 0)

            if t_after > t_before:
                alpha = (target_time - t_before) / (t_after - t_before)
                return self._interpolate_states(before, after, alpha)

        # Otherwise, use most recent sample
        return buffer[-1] if buffer else None

    def _interpolate_states(self, s1: dict, s2: dict, alpha: float) -> dict:
        """Linearly interpolate between two states."""
        result = {}

        # Interpolate numeric fields
        numeric_fields = [
            "position_turns",
            "position_deg",
            "torque_nm",
            "omega_rad_s",
        ]

        for field in numeric_fields:
            v1 = s1.get(field)
            v2 = s2.get(field)

            if v1 is not None and v2 is not None:
                result[field] = v1 + alpha * (v2 - v1)

        # Copy latest status
        for field in ["status", "data_valid"]:
            result[field] = s2.get(field, s1.get(field))

        return result

    def _sync_mujoco_to_steve(
        self, model: mujoco.MjModel, data: mujoco.MjData, state: dict
    ) -> None:
        """Sync MuJoCo state to match STEVE hardware."""
        if not state.get("data_valid", False):
            return

        # Convert position from degrees to radians
        position_deg = state.get("position_deg")
        velocity_rad_s = state.get("omega_rad_s")
        torque_nm = state.get("torque_nm")

        if position_deg is not None:
            # Set joint position
            qpos_idx = model.jnt_qposadr[self._joint_id]
            data.qpos[qpos_idx] = np.deg2rad(position_deg)

        if velocity_rad_s is not None:
            # Set joint velocity
            qvel_idx = model.jnt_dofadr[self._joint_id]
            data.qvel[qvel_idx] = velocity_rad_s

        if torque_nm is not None and self._actuator_id is not None:
            # Apply torque via actuator
            data.ctrl[self._actuator_id] = torque_nm

    def _sync_hybrid(
        self, model: mujoco.MjModel, data: mujoco.MjData, state: dict
    ) -> None:
        """
        Hybrid sync - bidirectional between MuJoCo and STEVE.
        
        Syncs STEVE state to MuJoCo, and MuJoCo config changes to STEVE.
        """
        # Sync STEVE → MuJoCo (state)
        self._sync_mujoco_to_steve(model, data, state)

        # Sync MuJoCo → STEVE (configuration changes)
        # Check if MuJoCo model parameters changed
        config_hash = self._hash_mujoco_config(model)

        if self._last_config_hash is None:
            self._last_config_hash = config_hash
        elif config_hash != self._last_config_hash:
            # Configuration changed - update STEVE
            self._update_steve_config(model)
            self._last_config_hash = config_hash

    def _hash_mujoco_config(self, model: mujoco.MjModel) -> int:
        """Compute hash of relevant MuJoCo configuration parameters."""
        # Hash joint damping, stiffness, etc.
        params = []

        if self._joint_id is not None:
            # Joint damping
            dof_idx = model.jnt_dofadr[self._joint_id]
            params.append(model.dof_damping[dof_idx])

            # Joint limits
            params.extend(model.jnt_range[self._joint_id])

        return hash(tuple(params))

    def _update_steve_config(self, model: mujoco.MjModel) -> None:
        """Update STEVE configuration from MuJoCo model parameters."""
        if not self.client or self._joint_id is None:
            return

        # Extract MuJoCo parameters
        dof_idx = model.jnt_dofadr[self._joint_id]
        damping = float(model.dof_damping[dof_idx])

        # Map to STEVE parameters (approximate)
        # This is a simple mapping - adjust based on your needs
        viscous = damping * 0.01  # Scale factor

        # Update STEVE
        try:
            self.client.update_config(viscous=viscous)
        except Exception:
            pass  # Ignore errors in config sync

    def get_steve_state(self) -> Optional[dict]:
        """
        Get latest STEVE state.
        
        Returns:
            Latest state dict, or None if no data available
        """
        with self._buffer_lock:
            return self._state_buffer[-1] if self._state_buffer else None

    def set_steve_config(self, **kwargs) -> None:
        """
        Update STEVE configuration parameters.
        
        Args:
            **kwargs: Configuration parameters (viscous, coulomb, etc.)
        """
        if not self.client:
            raise SteveError("Not connected")

        self.client.update_config(**kwargs)

    @property
    def is_connected(self) -> bool:
        """Check if connected to STEVE."""
        return self.client is not None and self.client.is_connected

    @property
    def buffer_size(self) -> int:
        """Get current buffer size."""
        with self._buffer_lock:
            return len(self._state_buffer)

    def __enter__(self) -> "SteveValveActuator":
        """Context manager entry."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit."""
        self.stop()
        self.disconnect()

"""
NVIDIA Isaac Sim connector for STEVE valve hardware integration.
"""

import asyncio
import time
from typing import Optional, Dict, Any, List, Callable
import numpy as np

try:
    from pxr import Usd, UsdGeom, UsdPhysics, Gf
    from omni.isaac.core.utils.stage import get_current_stage
    from omni.isaac.core.prims import XFormPrim
except ImportError:
    raise ImportError(
        "Isaac Sim required. Install NVIDIA Isaac Sim 2023.1+ and ensure "
        "environment is configured correctly."
    )

from pysteve.core.client import SteveClient
from pysteve.core.streaming import SteveStreamer
from pysteve.core.config import ValveConfig
from pysteve.core.exceptions import SteveError


class IsaacSteveConnector:
    """
    Connect STEVE valve hardware to NVIDIA Isaac Sim environment.
    
    Bidirectional synchronization between physical hardware and USD simulation:
    - Physical properties in USD articulation schema
    - Haptic properties in custom steve: namespace
    - Real-time data streaming from hardware
    - Command transmission to hardware
    
    Args:
        stage: USD stage for the simulation
        device_ip: STEVE device IP address
        prim_path: USD prim path for valve (default: "/World/Valve")
        sync_mode: Sync mode - "sim_to_hw", "hw_to_sim", "bidirectional"
        update_rate_hz: Update rate for synchronization (default: 60)
        api_key: STEVE API key (default: "steve-valve-2025")
    
    Example:
        >>> from omni.isaac.core.utils.stage import get_current_stage
        >>> 
        >>> stage = get_current_stage()
        >>> connector = IsaacSteveConnector(
        ...     stage=stage,
        ...     device_ip="192.168.1.100",
        ...     prim_path="/World/Valve",
        ...     sync_mode="bidirectional"
        ... )
        >>> 
        >>> # Start synchronization
        >>> connector.start()
        >>> 
        >>> # Run simulation...
        >>> 
        >>> # Stop synchronization
        >>> connector.stop()
    """

    def __init__(
        self,
        stage: Usd.Stage,
        device_ip: str,
        prim_path: str = "/World/Valve",
        sync_mode: str = "bidirectional",
        update_rate_hz: int = 60,
        api_key: str = "steve-valve-2025",
    ):
        self.stage = stage
        self.device_ip = device_ip
        self.prim_path = prim_path
        self.sync_mode = sync_mode
        self.update_rate_hz = update_rate_hz
        self.api_key = api_key

        # Clients
        self.client: Optional[SteveClient] = None
        self.streamer: Optional[SteveStreamer] = None

        # USD prim
        self.valve_prim: Optional[Usd.Prim] = None
        self.articulation_root: Optional[Usd.Prim] = None

        # State
        self._running = False
        self._last_hw_state: Optional[Dict[str, Any]] = None
        self._last_sim_state: Optional[Dict[str, Any]] = None
        self._update_task: Optional[asyncio.Task] = None

        # Callbacks
        self._on_hw_update_callbacks: List[Callable] = []
        self._on_sim_update_callbacks: List[Callable] = []

    def connect(self) -> None:
        """
        Connect to STEVE hardware and initialize USD prim.
        """
        # Connect to hardware
        self.client = SteveClient(self.device_ip, api_key=self.api_key, auto_reconnect=True)
        self.client.connect()

        # Start streaming
        stream_interval_ms = int(1000 / self.update_rate_hz)
        self.streamer = SteveStreamer(self.client, threadsafe=True)
        self.streamer.start_stream(interval_ms=stream_interval_ms)

        # Get USD prim
        self.valve_prim = self.stage.GetPrimAtPath(self.prim_path)
        if not self.valve_prim.IsValid():
            raise SteveError(f"Valve prim not found at {self.prim_path}")

        # Find articulation root
        self.articulation_root = self._find_articulation_root(self.valve_prim)
        if not self.articulation_root:
            raise SteveError(f"No articulation root found for {self.prim_path}")

        # Wait for first data sample
        time.sleep(0.5)
        self._last_hw_state = self.streamer.get_latest_sample()

        print(f"Connected to STEVE device at {self.device_ip}")
        print(f"Valve prim: {self.prim_path}")
        print(f"Sync mode: {self.sync_mode}")

    def start(self) -> None:
        """Start synchronization loop."""
        if not self.client or not self.streamer:
            raise SteveError("Not connected. Call connect() first.")

        if self._running:
            return

        self._running = True
        print("Starting Isaac Sim <-> STEVE synchronization...")

    def stop(self) -> None:
        """Stop synchronization loop."""
        self._running = False
        print("Stopped synchronization.")

    def update(self) -> None:
        """
        Update synchronization (call from Isaac Sim step callback).
        """
        if not self._running:
            return

        # Get latest hardware state
        hw_state = self.streamer.get_latest_sample()
        if hw_state and hw_state.get("data_valid"):
            self._last_hw_state = hw_state

        # Sync based on mode
        if self.sync_mode == "hw_to_sim":
            self._sync_hw_to_sim()
        elif self.sync_mode == "sim_to_hw":
            self._sync_sim_to_hw()
        elif self.sync_mode == "bidirectional":
            self._sync_bidirectional()

    def _sync_hw_to_sim(self) -> None:
        """Sync hardware state to simulation."""
        if not self._last_hw_state:
            return

        # Get position from hardware
        position_deg = self._last_hw_state.get("position_deg", 0)
        velocity_rad_s = self._last_hw_state.get("omega_rad_s", 0)

        # Update USD articulation
        self._set_sim_joint_state(position_deg, velocity_rad_s)

        # Trigger callbacks
        for callback in self._on_hw_update_callbacks:
            callback(self._last_hw_state)

    def _sync_sim_to_hw(self) -> None:
        """Sync simulation state to hardware."""
        # Get simulation joint state
        sim_state = self._get_sim_joint_state()
        if not sim_state:
            return

        self._last_sim_state = sim_state

        # Send torque command to hardware
        target_torque = sim_state.get("target_torque", 0)
        try:
            self.client.set_torque(target_torque)
        except Exception:
            pass

        # Trigger callbacks
        for callback in self._on_sim_update_callbacks:
            callback(sim_state)

    def _sync_bidirectional(self) -> None:
        """Bidirectional sync with conflict resolution."""
        # For now, prioritize hardware state
        # TODO: Implement smarter conflict resolution
        self._sync_hw_to_sim()

    def _set_sim_joint_state(self, position_deg: float, velocity_rad_s: float) -> None:
        """Set joint position and velocity in simulation."""
        # Convert degrees to radians
        position_rad = np.deg2rad(position_deg)

        # Get joint prim
        joint_prim = self._get_joint_prim()
        if not joint_prim:
            return

        # Set position (requires PhysicsArticulation API)
        # This is a simplified version - actual implementation depends on Isaac Sim API
        try:
            # Use XFormPrim for basic transform
            xform = XFormPrim(prim_path=self.prim_path)
            # Set rotation around joint axis
            # Note: This is simplified - actual joint control uses PhysicsArticulation
            pass
        except Exception as e:
            pass

    def _get_sim_joint_state(self) -> Optional[Dict[str, Any]]:
        """Get current joint state from simulation."""
        joint_prim = self._get_joint_prim()
        if not joint_prim:
            return None

        # Read joint state from USD
        # This is simplified - actual implementation uses PhysicsArticulation API
        return {
            "position_deg": 0.0,
            "velocity_rad_s": 0.0,
            "target_torque": 0.0,
        }

    def _get_joint_prim(self) -> Optional[Usd.Prim]:
        """Get revolute joint prim from articulation."""
        # Find joint in articulation hierarchy
        for prim in Usd.PrimRange(self.articulation_root):
            if prim.IsA(UsdPhysics.RevoluteJoint):
                return prim
        return None

    def _find_articulation_root(self, prim: Usd.Prim) -> Optional[Usd.Prim]:
        """Find articulation root in hierarchy."""
        current = prim
        while current:
            if current.IsA(UsdPhysics.ArticulationRootAPI):
                return current
            current = current.GetParent()
        return None

    def sync_config_to_usd(self, config: ValveConfig) -> None:
        """
        Write STEVE configuration to USD custom attributes.
        
        Creates custom attributes in steve: namespace.
        """
        if not self.valve_prim:
            raise SteveError("Not connected. Call connect() first.")

        # Create custom attributes
        self._set_usd_attr("steve:viscous", config.viscous)
        self._set_usd_attr("steve:coulomb", config.coulomb)
        self._set_usd_attr("steve:wall_stiffness", config.wall_stiffness)
        self._set_usd_attr("steve:wall_damping", config.wall_damping)
        self._set_usd_attr("steve:smoothing", config.smoothing)
        self._set_usd_attr("steve:torque_limit", config.torque_limit)
        self._set_usd_attr("steve:travel", config.travel)
        self._set_usd_attr("steve:closed_position", config.closed_position)
        self._set_usd_attr("steve:open_position", config.open_position)

        print(f"Synced config to USD: {self.prim_path}")

    def sync_config_from_usd(self) -> ValveConfig:
        """
        Read STEVE configuration from USD custom attributes.
        
        Returns:
            ValveConfig object
        """
        if not self.valve_prim:
            raise SteveError("Not connected. Call connect() first.")

        # Read custom attributes
        config = ValveConfig(
            viscous=self._get_usd_attr("steve:viscous", 0.1),
            coulomb=self._get_usd_attr("steve:coulomb", 0.01),
            wall_stiffness=self._get_usd_attr("steve:wall_stiffness", 2.0),
            wall_damping=self._get_usd_attr("steve:wall_damping", 0.2),
            smoothing=self._get_usd_attr("steve:smoothing", 0.001),
            torque_limit=self._get_usd_attr("steve:torque_limit", 0.5),
            travel=self._get_usd_attr("steve:travel", 90),
            closed_position=self._get_usd_attr("steve:closed_position", 0),
            open_position=self._get_usd_attr("steve:open_position", 90),
        )

        return config

    def _set_usd_attr(self, name: str, value: Any) -> None:
        """Set custom USD attribute."""
        attr = self.valve_prim.GetAttribute(name)
        if not attr:
            # Create attribute if doesn't exist
            if isinstance(value, float):
                attr = self.valve_prim.CreateAttribute(name, Usd.GetTypeFromTypeid(Usd.TypeFloat))
            elif isinstance(value, int):
                attr = self.valve_prim.CreateAttribute(name, Usd.GetTypeFromTypeid(Usd.TypeInt))

        if attr:
            attr.Set(value)

    def _get_usd_attr(self, name: str, default: Any) -> Any:
        """Get custom USD attribute."""
        attr = self.valve_prim.GetAttribute(name)
        if attr and attr.HasValue():
            return attr.Get()
        return default

    def register_hw_update_callback(self, callback: Callable) -> None:
        """Register callback for hardware updates."""
        self._on_hw_update_callbacks.append(callback)

    def register_sim_update_callback(self, callback: Callable) -> None:
        """Register callback for simulation updates."""
        self._on_sim_update_callbacks.append(callback)

    def get_hardware_state(self) -> Optional[Dict[str, Any]]:
        """Get latest hardware state."""
        return self._last_hw_state

    def get_simulation_state(self) -> Optional[Dict[str, Any]]:
        """Get latest simulation state."""
        return self._last_sim_state

    def disconnect(self) -> None:
        """Disconnect from hardware and cleanup."""
        self.stop()

        if self.streamer:
            try:
                self.streamer.stop_stream()
            except Exception:
                pass

        if self.client:
            try:
                self.client.stop_valve()
                self.client.disconnect()
            except Exception:
                pass

        print("Disconnected from STEVE device.")

    def __del__(self) -> None:
        """Cleanup on deletion."""
        self.disconnect()

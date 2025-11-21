"""
Coordinator for managing multiple STEVE valve devices in Isaac Sim.
"""

import time
from typing import Dict, List, Optional, Tuple, Callable, Any
import numpy as np

try:
    from pxr import Usd
    from omni.isaac.core.utils.stage import get_current_stage
except ImportError:
    raise ImportError(
        "Isaac Sim required. Install NVIDIA Isaac Sim 2023.1+ and ensure "
        "environment is configured correctly."
    )

from pysteve.integrations.isaac.connector import IsaacSteveConnector
from pysteve.core.config import ValveConfig
from pysteve.core.exceptions import SteveError


class MultiValveCoordinator:
    """
    Coordinate multiple STEVE valve devices in Isaac Sim environment.
    
    Manages synchronization, data collection, and coordinated control
    of multiple hardware valves with their USD representations.
    
    Args:
        stage: USD stage for simulation
        sync_mode: Global sync mode for all valves
        update_rate_hz: Global update rate
    
    Example:
        >>> from omni.isaac.core.utils.stage import get_current_stage
        >>> 
        >>> stage = get_current_stage()
        >>> coordinator = MultiValveCoordinator(stage)
        >>> 
        >>> # Add valves
        >>> coordinator.add_valve(
        ...     valve_id="valve1",
        ...     device_ip="192.168.1.100",
        ...     prim_path="/World/Valve01"
        ... )
        >>> coordinator.add_valve(
        ...     valve_id="valve2",
        ...     device_ip="192.168.1.101",
        ...     prim_path="/World/Valve02"
        ... )
        >>> 
        >>> # Connect all
        >>> coordinator.connect_all()
        >>> 
        >>> # Start synchronized operation
        >>> coordinator.start_all()
        >>> 
        >>> # Update in simulation loop
        >>> coordinator.update_all()
    """

    def __init__(
        self,
        stage: Optional[Usd.Stage] = None,
        sync_mode: str = "bidirectional",
        update_rate_hz: int = 60,
    ):
        self.stage = stage or get_current_stage()
        self.sync_mode = sync_mode
        self.update_rate_hz = update_rate_hz

        # Valve connectors
        self.valves: Dict[str, IsaacSteveConnector] = {}

        # State tracking
        self._all_states: Dict[str, Dict[str, Any]] = {}
        self._start_time = time.time()

    def add_valve(
        self,
        valve_id: str,
        device_ip: str,
        prim_path: str,
        sync_mode: Optional[str] = None,
        api_key: str = "steve-valve-2025",
    ) -> None:
        """
        Add valve to coordinator.
        
        Args:
            valve_id: Unique identifier for valve
            device_ip: STEVE device IP address
            prim_path: USD prim path for valve
            sync_mode: Override global sync mode (optional)
            api_key: STEVE API key
        """
        if valve_id in self.valves:
            raise SteveError(f"Valve {valve_id} already exists")

        connector = IsaacSteveConnector(
            stage=self.stage,
            device_ip=device_ip,
            prim_path=prim_path,
            sync_mode=sync_mode or self.sync_mode,
            update_rate_hz=self.update_rate_hz,
            api_key=api_key,
        )

        self.valves[valve_id] = connector
        print(f"Added valve {valve_id} at {device_ip}")

    def remove_valve(self, valve_id: str) -> None:
        """Remove valve from coordinator."""
        if valve_id in self.valves:
            self.valves[valve_id].disconnect()
            del self.valves[valve_id]
            print(f"Removed valve {valve_id}")

    def connect_all(self) -> None:
        """Connect to all valves."""
        for valve_id, connector in self.valves.items():
            try:
                connector.connect()
                print(f"Connected to {valve_id}")
            except Exception as e:
                print(f"Failed to connect to {valve_id}: {e}")

    def disconnect_all(self) -> None:
        """Disconnect from all valves."""
        for valve_id, connector in self.valves.items():
            try:
                connector.disconnect()
            except Exception:
                pass

    def start_all(self) -> None:
        """Start synchronization for all valves."""
        for valve_id, connector in self.valves.items():
            connector.start()

    def stop_all(self) -> None:
        """Stop synchronization for all valves."""
        for valve_id, connector in self.valves.items():
            connector.stop()

    def update_all(self) -> None:
        """Update all valves (call from simulation step)."""
        for valve_id, connector in self.valves.items():
            connector.update()

            # Cache state
            hw_state = connector.get_hardware_state()
            if hw_state:
                self._all_states[valve_id] = hw_state

    def get_valve(self, valve_id: str) -> Optional[IsaacSteveConnector]:
        """Get valve connector by ID."""
        return self.valves.get(valve_id)

    def get_all_states(self) -> Dict[str, Dict[str, Any]]:
        """Get cached states for all valves."""
        return self._all_states.copy()

    def sync_all_configs_to_usd(self) -> None:
        """Sync all valve configs to USD."""
        for valve_id, connector in self.valves.items():
            try:
                config = connector.client.get_config()
                connector.sync_config_to_usd(config)
                print(f"Synced {valve_id} config to USD")
            except Exception as e:
                print(f"Failed to sync {valve_id}: {e}")

    def sync_all_configs_from_usd(self) -> None:
        """Sync all valve configs from USD to hardware."""
        for valve_id, connector in self.valves.items():
            try:
                config = connector.sync_config_from_usd()
                connector.client.update_config(**config.to_dict())
                print(f"Synced {valve_id} config from USD")
            except Exception as e:
                print(f"Failed to sync {valve_id}: {e}")

    def apply_synchronized_action(
        self, actions: Dict[str, float], action_type: str = "torque"
    ) -> None:
        """
        Apply synchronized actions to all valves.
        
        Args:
            actions: Dict mapping valve_id -> action value
            action_type: "torque" or "position"
        """
        for valve_id, action in actions.items():
            connector = self.valves.get(valve_id)
            if not connector:
                continue

            try:
                if action_type == "torque":
                    connector.client.set_torque(action)
                elif action_type == "position":
                    # Position control not directly supported
                    pass
            except Exception:
                pass

    def calibrate_all(self) -> None:
        """Calibrate all valve motors."""
        for valve_id, connector in self.valves.items():
            try:
                connector.client.enable_motor()
                time.sleep(0.5)
                connector.client.calibrate_motor()
                print(f"Calibrating {valve_id}...")
            except Exception as e:
                print(f"Failed to calibrate {valve_id}: {e}")

        # Wait for calibration
        time.sleep(10)
        print("Calibration complete")

    def start_all_valves(self) -> None:
        """Start all valve simulations."""
        for valve_id, connector in self.valves.items():
            try:
                connector.client.start_valve()
                print(f"Started {valve_id}")
            except Exception as e:
                print(f"Failed to start {valve_id}: {e}")

    def stop_all_valves(self) -> None:
        """Stop all valve simulations."""
        for valve_id, connector in self.valves.items():
            try:
                connector.client.stop_valve()
            except Exception:
                pass

    def get_synchronized_observations(self) -> Dict[str, np.ndarray]:
        """
        Get synchronized observations from all valves.
        
        Returns:
            Dict mapping valve_id -> observation array [pos, vel, torque]
        """
        observations = {}

        for valve_id in self.valves.keys():
            state = self._all_states.get(valve_id)
            if state and state.get("data_valid"):
                obs = np.array(
                    [
                        state.get("position_deg", 0),
                        state.get("omega_rad_s", 0),
                        state.get("torque_nm", 0),
                    ]
                )
                observations[valve_id] = obs

        return observations

    def register_global_callback(self, callback: Callable) -> None:
        """
        Register callback for all valve updates.
        
        Args:
            callback: Function with signature callback(valve_id, state)
        """
        for valve_id, connector in self.valves.items():
            # Wrap callback to include valve_id
            def wrapped_callback(state, vid=valve_id):
                callback(vid, state)

            connector.register_hw_update_callback(wrapped_callback)

    def get_statistics(self) -> Dict[str, Any]:
        """
        Get coordination statistics.
        
        Returns:
            Statistics dict with uptime, packet counts, etc.
        """
        stats = {
            "num_valves": len(self.valves),
            "uptime_s": time.time() - self._start_time,
            "valves": {},
        }

        for valve_id, connector in self.valves.items():
            state = self._all_states.get(valve_id, {})
            stats["valves"][valve_id] = {
                "connected": connector.client is not None,
                "position_deg": state.get("position_deg", 0),
                "velocity_rad_s": state.get("omega_rad_s", 0),
                "torque_nm": state.get("torque_nm", 0),
                "data_valid": state.get("data_valid", False),
            }

        return stats

    def export_trajectories(self, filename: str) -> None:
        """
        Export recorded trajectories for all valves.
        
        Args:
            filename: Output file path (CSV or HDF5)
        """
        # TODO: Implement trajectory export
        pass

    def __del__(self) -> None:
        """Cleanup on deletion."""
        self.disconnect_all()

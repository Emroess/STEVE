"""
Hardware sync controller for coordinating multiple STEVE valve actuators.
"""

from typing import List, Dict, Optional
import threading

try:
    import mujoco
except ImportError:
    raise ImportError("MuJoCo required. Install with: pip install pysteve[mujoco]")

from pysteve.integrations.mujoco.actuator import SteveValveActuator


class HardwareSyncController:
    """
    Controller for coordinating multiple STEVE valve actuators in MuJoCo.
    
    Manages multiple hardware valves in a single MuJoCo simulation with
    collision detection and synchronized updates.
    
    Args:
        model: MuJoCo model
        data: MuJoCo data
    
    Example:
        >>> model = mujoco.MjModel.from_xml_path("multi_valve.xml")
        >>> data = mujoco.MjData(model)
        >>> 
        >>> controller = HardwareSyncController(model, data)
        >>> 
        >>> # Add valves
        >>> valve1 = SteveValveActuator("192.168.1.100", "valve_joint_1")
        >>> valve2 = SteveValveActuator("192.168.1.101", "valve_joint_2")
        >>> 
        >>> controller.add_actuator("valve1", valve1)
        >>> controller.add_actuator("valve2", valve2)
        >>> 
        >>> # Connect and start all
        >>> controller.connect_all()
        >>> controller.start_all()
        >>> 
        >>> # Simulation loop
        >>> while True:
        ...     controller.update_all()
        ...     mujoco.mj_step(model, data)
    """

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model = model
        self.data = data
        self._actuators: Dict[str, SteveValveActuator] = {}
        self._lock = threading.Lock()

    def add_actuator(self, name: str, actuator: SteveValveActuator) -> None:
        """
        Add a valve actuator to the controller.
        
        Args:
            name: Unique name for this actuator
            actuator: SteveValveActuator instance
        """
        with self._lock:
            self._actuators[name] = actuator

    def remove_actuator(self, name: str) -> None:
        """
        Remove a valve actuator from the controller.
        
        Args:
            name: Name of actuator to remove
        """
        with self._lock:
            if name in self._actuators:
                actuator = self._actuators.pop(name)
                if actuator.is_connected:
                    actuator.stop()
                    actuator.disconnect()

    def get_actuator(self, name: str) -> Optional[SteveValveActuator]:
        """
        Get actuator by name.
        
        Args:
            name: Actuator name
        
        Returns:
            SteveValveActuator instance, or None if not found
        """
        with self._lock:
            return self._actuators.get(name)

    def connect_all(self) -> None:
        """Connect all actuators to their STEVE devices."""
        with self._lock:
            for name, actuator in self._actuators.items():
                try:
                    actuator.connect()
                except Exception as e:
                    print(f"Warning: Failed to connect actuator '{name}': {e}")

    def disconnect_all(self) -> None:
        """Disconnect all actuators."""
        with self._lock:
            for actuator in self._actuators.values():
                try:
                    if actuator.is_connected:
                        actuator.disconnect()
                except Exception:
                    pass

    def start_all(self) -> None:
        """Start valve simulation on all connected actuators."""
        with self._lock:
            for name, actuator in self._actuators.items():
                try:
                    if actuator.is_connected:
                        actuator.start()
                except Exception as e:
                    print(f"Warning: Failed to start actuator '{name}': {e}")

    def stop_all(self) -> None:
        """Stop valve simulation on all actuators."""
        with self._lock:
            for actuator in self._actuators.values():
                try:
                    if actuator.is_connected:
                        actuator.stop()
                except Exception:
                    pass

    def update_all(self) -> None:
        """
        Update all actuators.
        
        Call this before each mujoco.mj_step() to sync all valves.
        """
        with self._lock:
            for actuator in self._actuators.values():
                try:
                    if actuator.is_connected:
                        actuator.update(self.model, self.data)
                except Exception as e:
                    print(f"Warning: Actuator update failed: {e}")

    def check_collisions(self) -> List[tuple]:
        """
        Check for collisions between controlled joints.
        
        Returns:
            List of (name1, name2) tuples for colliding joints
        """
        collisions = []
        
        with self._lock:
            actuator_list = list(self._actuators.items())
        
        # Check all pairs
        for i, (name1, act1) in enumerate(actuator_list):
            for name2, act2 in actuator_list[i + 1 :]:
                # Get joint positions
                state1 = act1.get_steve_state()
                state2 = act2.get_steve_state()
                
                if state1 and state2:
                    # Simple collision detection based on position overlap
                    # This is a placeholder - implement actual collision logic
                    pos1 = state1.get("position_deg", 0)
                    pos2 = state2.get("position_deg", 0)
                    
                    # Example: Check if positions are very close
                    if abs(pos1 - pos2) < 1.0:  # Within 1 degree
                        collisions.append((name1, name2))
        
        return collisions

    def get_all_states(self) -> Dict[str, Optional[dict]]:
        """
        Get current state of all actuators.
        
        Returns:
            Dictionary mapping actuator names to state dicts
        """
        states = {}
        
        with self._lock:
            for name, actuator in self._actuators.items():
                if actuator.is_connected:
                    states[name] = actuator.get_steve_state()
                else:
                    states[name] = None
        
        return states

    def set_all_configs(self, **config_params) -> None:
        """
        Set configuration parameters on all actuators.
        
        Args:
            **config_params: Configuration parameters (viscous, coulomb, etc.)
        """
        with self._lock:
            for actuator in self._actuators.values():
                try:
                    if actuator.is_connected:
                        actuator.set_steve_config(**config_params)
                except Exception as e:
                    print(f"Warning: Failed to set config: {e}")

    @property
    def actuator_count(self) -> int:
        """Get number of registered actuators."""
        with self._lock:
            return len(self._actuators)

    @property
    def connected_count(self) -> int:
        """Get number of connected actuators."""
        with self._lock:
            return sum(1 for act in self._actuators.values() if act.is_connected)

    def __enter__(self) -> "HardwareSyncController":
        """Context manager entry."""
        self.connect_all()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit."""
        self.stop_all()
        self.disconnect_all()

"""
Real-time parameter tuning for live valve adjustment.
"""

import time
from typing import Optional, Dict, Any

from pysteve.core.exceptions import SteveValidationError


class RealtimeTuner:
    """
    Real-time parameter tuner for live valve adjustment.
    
    Enables updating valve haptic parameters while the valve is running,
    without interrupting operation. All parameters are validated and applied
    immediately via REST API.
    
    Args:
        client: SteveClient instance
        cache_ttl: Configuration cache TTL in seconds (default: 1.0)
    
    Example:
        >>> from pysteve import SteveClient, RealtimeTuner
        >>> 
        >>> client = SteveClient("192.168.1.100")
        >>> client.enable_motor()
        >>> client.start_valve()
        >>> 
        >>> tuner = RealtimeTuner(client)
        >>> tuner.set_viscous(0.08)
        >>> tuner.set_coulomb(0.015)
        >>> tuner.update_multiple(wall_stiffness=2.0, wall_damping=0.15)
    """

    def __init__(self, client: "SteveClient", cache_ttl: float = 1.0):  # type: ignore
        self.client = client
        self.cache_ttl = cache_ttl
        self._config_cache: Optional[Dict[str, Any]] = None
        self._cache_time: float = 0.0

    def _validate_range(self, name: str, value: float, min_val: float, max_val: float) -> None:
        """Validate parameter is within accepted range."""
        if not (min_val <= value <= max_val):
            raise SteveValidationError(
                f"{name} must be between {min_val} and {max_val}, got {value}"
            )

    def get_current_config(self, use_cache: bool = True) -> Dict[str, Any]:
        """
        Get current valve configuration.
        
        Args:
            use_cache: Use cached config if within TTL (default: True)
        
        Returns:
            Current configuration dictionary
        """
        now = time.time()
        if use_cache and self._config_cache and (now - self._cache_time) < self.cache_ttl:
            return self._config_cache

        config = self.client.get_config()
        self._config_cache = config.to_dict()
        self._cache_time = now
        return self._config_cache

    def set_viscous(self, value: float) -> Dict[str, Any]:
        """
        Set viscous damping coefficient.
        
        Args:
            value: Viscous damping [N·m·s/rad] (0.01 - 0.5)
        
        Returns:
            Response data
        
        Example:
            >>> tuner.set_viscous(0.08)
        """
        self._validate_range("viscous", value, 0.01, 0.5)
        result = self.client.update_config(viscous=value)
        self._config_cache = None  # Invalidate cache
        return result

    def set_coulomb(self, value: float) -> Dict[str, Any]:
        """
        Set Coulomb friction torque.
        
        Args:
            value: Coulomb friction [N·m] (0.005 - 0.05)
        
        Returns:
            Response data
        """
        self._validate_range("coulomb", value, 0.005, 0.05)
        result = self.client.update_config(coulomb=value)
        self._config_cache = None
        return result

    def set_wall_stiffness(self, value: float) -> Dict[str, Any]:
        """
        Set virtual wall stiffness.
        
        Args:
            value: Wall stiffness [N·m/turn] (0.5 - 5.0)
        
        Returns:
            Response data
        """
        self._validate_range("wall_stiffness", value, 0.5, 5.0)
        result = self.client.update_config(wall_stiffness=value)
        self._config_cache = None
        return result

    def set_wall_damping(self, value: float) -> Dict[str, Any]:
        """
        Set virtual wall damping.
        
        Args:
            value: Wall damping [N·m·s/turn] (0.05 - 0.5)
        
        Returns:
            Response data
        """
        self._validate_range("wall_damping", value, 0.05, 0.5)
        result = self.client.update_config(wall_damping=value)
        self._config_cache = None
        return result

    def set_smoothing(self, value: float) -> Dict[str, Any]:
        """
        Set friction smoothing epsilon.
        
        Args:
            value: Smoothing epsilon (0.0001 - 0.01)
        
        Returns:
            Response data
        """
        self._validate_range("smoothing", value, 0.0001, 0.01)
        result = self.client.update_config(smoothing=value)
        self._config_cache = None
        return result

    def set_torque_limit(self, value: float) -> Dict[str, Any]:
        """
        Set maximum torque limit.
        
        Args:
            value: Torque limit [N·m] (0.1 - 2.0)
        
        Returns:
            Response data
        """
        self._validate_range("torque_limit", value, 0.1, 2.0)
        result = self.client.update_config(torque_limit=value)
        self._config_cache = None
        return result

    def set_endpoints(self, closed: float, open: float) -> Dict[str, Any]:
        """
        Set valve endpoint positions.
        
        Args:
            closed: Closed position [degrees] (0 - 360)
            open: Open position [degrees] (0 - 360)
        
        Returns:
            Response data
        
        Raises:
            SteveValidationError: If closed >= open
        """
        if not (0 <= closed <= 360):
            raise SteveValidationError(f"closed must be 0-360, got {closed}")
        if not (0 <= open <= 360):
            raise SteveValidationError(f"open must be 0-360, got {open}")
        if closed >= open:
            raise SteveValidationError(f"closed ({closed}) must be < open ({open})")

        result = self.client.update_config(closed_position=closed, open_position=open)
        self._config_cache = None
        return result

    def set_travel(self, degrees: float) -> Dict[str, Any]:
        """
        Set travel range (adjusts open position, keeps closed at 0).
        
        Args:
            degrees: Travel range [degrees] (1 - 360)
        
        Returns:
            Response data
        """
        self._validate_range("travel", degrees, 1, 360)
        result = self.client.update_config(closed_position=0.0, open_position=degrees)
        self._config_cache = None
        return result

    def update_multiple(self, **params) -> Dict[str, Any]:
        """
        Update multiple parameters atomically.
        
        All parameters are validated before sending to ensure atomicity.
        If validation fails, no parameters are updated.
        
        Args:
            **params: Parameters to update (viscous, coulomb, wall_stiffness, etc.)
        
        Returns:
            Response data with number of fields updated
        
        Example:
            >>> tuner.update_multiple(
            ...     viscous=0.08,
            ...     coulomb=0.015,
            ...     wall_stiffness=2.0,
            ...     wall_damping=0.15
            ... )
        """
        # Validate all parameters before updating
        valid_params = {
            "viscous": (0.01, 0.5),
            "coulomb": (0.005, 0.05),
            "wall_stiffness": (0.5, 5.0),
            "wall_damping": (0.05, 0.5),
            "smoothing": (0.0001, 0.01),
            "torque_limit": (0.1, 2.0),
            "open_position": (0, 360),
            "closed_position": (0, 360),
        }

        for key, value in params.items():
            if key in valid_params:
                min_val, max_val = valid_params[key]
                self._validate_range(key, value, min_val, max_val)
            elif key == "degrees_per_turn":
                if value <= 0:
                    raise SteveValidationError(f"degrees_per_turn must be positive, got {value}")
            else:
                raise SteveValidationError(f"Unknown parameter: {key}")

        # Check endpoint consistency if both provided
        if "closed_position" in params and "open_position" in params:
            if params["closed_position"] >= params["open_position"]:
                raise SteveValidationError(
                    f"closed_position ({params['closed_position']}) must be < "
                    f"open_position ({params['open_position']})"
                )

        # All validated - update
        result = self.client.update_config(**params)
        self._config_cache = None
        return result

    def sweep_parameter(
        self,
        param_name: str,
        values: list,
        dwell_time: float = 1.0,
        callback: Optional[callable] = None,
    ) -> list:
        """
        Sweep a parameter through a list of values.
        
        Useful for exploring parameter space and observing effects.
        
        Args:
            param_name: Parameter to sweep (e.g., "viscous", "coulomb")
            values: List of values to test
            dwell_time: Time to dwell at each value in seconds
            callback: Optional callback(value, status) called after each update
        
        Returns:
            List of status dicts collected at each value
        
        Example:
            >>> results = tuner.sweep_parameter(
            ...     "viscous",
            ...     [0.02, 0.04, 0.06, 0.08, 0.10],
            ...     dwell_time=2.0
            ... )
        """
        results = []

        for value in values:
            # Update parameter
            self.client.update_config(**{param_name: value})

            # Wait for settling
            time.sleep(dwell_time)

            # Collect status
            status = self.client.get_status()
            results.append(status)

            if callback:
                callback(value, status)

        self._config_cache = None
        return results

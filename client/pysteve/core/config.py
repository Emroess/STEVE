"""
Configuration dataclasses with validation for STEVE valve parameters.
"""

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class ValveConfig:
    """
    Valve configuration parameters.
    
    All parameters are validated on initialization to ensure they fall within
    STEVE firmware's accepted ranges.
    
    Attributes:
        viscous: Viscous damping coefficient [N·m·s/rad] (0.01 - 0.5)
        coulomb: Coulomb friction torque [N·m] (0.005 - 0.05)
        wall_stiffness: Virtual wall stiffness [N·m/turn] (0.5 - 5.0)
        wall_damping: Virtual wall damping [N·m·s/turn] (0.05 - 0.5)
        smoothing: Friction smoothing epsilon (0.0001 - 0.01)
        torque_limit: Maximum torque [N·m] (0.1 - 2.0)
        open_position: Fully open position [degrees] (0 - 360)
        closed_position: Fully closed position [degrees] (0 - 360)
        degrees_per_turn: Mechanical scaling [deg/turn] (> 0)
    """

    viscous: float = 0.05
    coulomb: float = 0.01
    wall_stiffness: float = 1.0
    wall_damping: float = 0.1
    smoothing: float = 0.001
    torque_limit: float = 0.5
    open_position: float = 90.0
    closed_position: float = 0.0
    degrees_per_turn: float = 360.0

    def __post_init__(self) -> None:
        """Validate parameters after initialization."""
        from pysteve.core.exceptions import SteveValidationError

        if not (0.01 <= self.viscous <= 0.5):
            raise SteveValidationError(
                f"viscous must be between 0.01 and 0.5, got {self.viscous}"
            )

        if not (0.005 <= self.coulomb <= 0.05):
            raise SteveValidationError(
                f"coulomb must be between 0.005 and 0.05, got {self.coulomb}"
            )

        if not (0.5 <= self.wall_stiffness <= 5.0):
            raise SteveValidationError(
                f"wall_stiffness must be between 0.5 and 5.0, got {self.wall_stiffness}"
            )

        if not (0.05 <= self.wall_damping <= 0.5):
            raise SteveValidationError(
                f"wall_damping must be between 0.05 and 0.5, got {self.wall_damping}"
            )

        if not (0.0001 <= self.smoothing <= 0.01):
            raise SteveValidationError(
                f"smoothing must be between 0.0001 and 0.01, got {self.smoothing}"
            )

        if not (0.1 <= self.torque_limit <= 2.0):
            raise SteveValidationError(
                f"torque_limit must be between 0.1 and 2.0, got {self.torque_limit}"
            )

        if not (0 <= self.open_position <= 360):
            raise SteveValidationError(
                f"open_position must be between 0 and 360, got {self.open_position}"
            )

        if not (0 <= self.closed_position <= 360):
            raise SteveValidationError(
                f"closed_position must be between 0 and 360, got {self.closed_position}"
            )

        if self.closed_position >= self.open_position:
            raise SteveValidationError(
                f"closed_position ({self.closed_position}) must be less than "
                f"open_position ({self.open_position})"
            )

        if self.degrees_per_turn <= 0:
            raise SteveValidationError(
                f"degrees_per_turn must be positive, got {self.degrees_per_turn}"
            )

    def to_dict(self) -> dict:
        """Convert to dictionary for API requests."""
        return {
            "viscous": self.viscous,
            "coulomb": self.coulomb,
            "wall_stiffness": self.wall_stiffness,
            "wall_damping": self.wall_damping,
            "smoothing": self.smoothing,
            "torque_limit": self.torque_limit,
            "open_position": self.open_position,
            "closed_position": self.closed_position,
            "degrees_per_turn": self.degrees_per_turn,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "ValveConfig":
        """Create ValveConfig from API response dictionary."""
        return cls(
            viscous=data.get("viscous", 0.05),
            coulomb=data.get("coulomb", 0.01),
            wall_stiffness=data.get("wall_stiffness", 1.0),
            wall_damping=data.get("wall_damping", 0.1),
            smoothing=data.get("smoothing", 0.001),
            torque_limit=data.get("torque_limit", 0.5),
            open_position=data.get("open_position", 90.0),
            closed_position=data.get("closed_position", 0.0),
            degrees_per_turn=data.get("degrees_per_turn", 360.0),
        )


@dataclass
class PresetConfig:
    """
    Preset configuration parameters.
    
    Presets are stored in STEVE firmware's non-volatile memory and can be
    loaded to quickly switch between different valve configurations.
    
    Attributes:
        index: Preset slot (0-3)
        name: Preset name (max 15 characters)
        viscous: Viscous damping coefficient [N·m·s/rad]
        coulomb: Coulomb friction torque [N·m]
        wall_stiffness: Virtual wall stiffness [N·m/turn]
        wall_damping: Virtual wall damping [N·m·s/turn]
        travel: Total angular travel [degrees] (1-360)
        torque_limit: Maximum torque [N·m]
        smoothing: Friction smoothing epsilon
    """

    index: int
    name: str = "custom"
    viscous: float = 0.05
    coulomb: float = 0.01
    wall_stiffness: float = 1.0
    wall_damping: float = 0.1
    travel: float = 90.0
    torque_limit: float = 0.5
    smoothing: float = 0.001

    def __post_init__(self) -> None:
        """Validate parameters after initialization."""
        from pysteve.core.exceptions import SteveValidationError

        if not (0 <= self.index <= 3):
            raise SteveValidationError(f"index must be between 0 and 3, got {self.index}")

        if len(self.name) > 15:
            raise SteveValidationError(
                f"name must be 15 characters or less, got {len(self.name)}"
            )

        if not (1 <= self.travel <= 360):
            raise SteveValidationError(
                f"travel must be between 1 and 360, got {self.travel}"
            )

        # Reuse ValveConfig validation for haptic parameters
        try:
            ValveConfig(
                viscous=self.viscous,
                coulomb=self.coulomb,
                wall_stiffness=self.wall_stiffness,
                wall_damping=self.wall_damping,
                smoothing=self.smoothing,
                torque_limit=self.torque_limit,
            )
        except SteveValidationError as e:
            raise SteveValidationError(f"Invalid preset parameter: {e}")

    def to_dict(self) -> dict:
        """Convert to dictionary for API requests."""
        return {
            "index": self.index,
            "name": self.name,
            "viscous": self.viscous,
            "coulomb": self.coulomb,
            "wall_stiffness": self.wall_stiffness,
            "wall_damping": self.wall_damping,
            "travel": self.travel,
            "torque_limit": self.torque_limit,
            "smoothing": self.smoothing,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "PresetConfig":
        """Create PresetConfig from API response dictionary."""
        return cls(
            index=data["index"],
            name=data.get("name", "custom"),
            viscous=data.get("viscous", 0.05),
            coulomb=data.get("coulomb", 0.01),
            wall_stiffness=data.get("wall_stiffness", 1.0),
            wall_damping=data.get("wall_damping", 0.1),
            travel=data.get("travel", 90.0),
            torque_limit=data.get("torque_limit", 0.5),
            smoothing=data.get("smoothing", 0.001),
        )

"""Utility modules for STEVE client."""

from .stream_buffer import StreamBuffer
from .plotting import RealtimePlotter, StaticPlotter
from .validation import (
    validate_config,
    validate_position,
    validate_torque,
    deg_to_rad,
    rad_to_deg,
    clamp,
    normalize_angle,
)

__all__ = [
    "StreamBuffer",
    "RealtimePlotter",
    "StaticPlotter",
    "validate_config",
    "validate_position",
    "validate_torque",
    "deg_to_rad",
    "rad_to_deg",
    "clamp",
    "normalize_angle",
]

# Optional ROS2 bridge
try:
    from .ros_bridge import SteveRosBridge, launch_bridge

    __all__.extend(["SteveRosBridge", "launch_bridge"])
except ImportError:
    pass


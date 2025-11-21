"""
MJCF model builder utilities for adding STEVE valves to MuJoCo models.
"""

from typing import Optional
from xml.etree import ElementTree as ET

try:
    import mujoco
except ImportError:
    raise ImportError("MuJoCo required. Install with: pip install pysteve[mujoco]")


def add_valve_joint(
    body_element: ET.Element,
    name: str,
    joint_type: str = "hinge",
    axis: str = "0 0 1",
    range: tuple = (0, 90),
    damping: float = 0.1,
    stiffness: float = 0.0,
    limited: bool = True,
) -> ET.Element:
    """
    Add a valve joint to an MJCF body element.
    
    Args:
        body_element: Parent body XML element
        name: Joint name
        joint_type: Joint type (default: "hinge")
        axis: Rotation axis (default: "0 0 1" for Z-axis)
        range: Joint limits in degrees (default: (0, 90))
        damping: Joint damping coefficient (default: 0.1)
        stiffness: Joint stiffness (default: 0.0)
        limited: Whether joint is limited (default: True)
    
    Returns:
        Created joint XML element
    
    Example:
        >>> import xml.etree.ElementTree as ET
        >>> body = ET.Element("body", name="valve_body")
        >>> joint = add_valve_joint(body, "valve_joint", range=(0, 180))
    """
    joint = ET.SubElement(body_element, "joint")
    joint.set("name", name)
    joint.set("type", joint_type)
    joint.set("axis", axis)
    joint.set("damping", str(damping))
    joint.set("stiffness", str(stiffness))
    
    if limited:
        joint.set("limited", "true")
        joint.set("range", f"{range[0]} {range[1]}")
    else:
        joint.set("limited", "false")
    
    return joint


def add_valve_actuator(
    actuator_parent: ET.Element,
    joint_name: str,
    actuator_name: Optional[str] = None,
    actuator_type: str = "motor",
    gear: float = 1.0,
    ctrl_range: Optional[tuple] = None,
    force_range: Optional[tuple] = None,
) -> ET.Element:
    """
    Add a valve actuator to MJCF model.
    
    Args:
        actuator_parent: Parent actuator container element
        joint_name: Name of joint to actuate
        actuator_name: Actuator name (default: joint_name + "_actuator")
        actuator_type: Actuator type (default: "motor")
        gear: Gear ratio (default: 1.0)
        ctrl_range: Control range in Nm (default: (-0.5, 0.5))
        force_range: Force range in Nm (default: (-2.0, 2.0))
    
    Returns:
        Created actuator XML element
    
    Example:
        >>> actuator_container = ET.Element("actuator")
        >>> motor = add_valve_actuator(
        ...     actuator_container,
        ...     "valve_joint",
        ...     ctrl_range=(-1.0, 1.0)
        ... )
    """
    if actuator_name is None:
        actuator_name = f"{joint_name}_actuator"
    
    if ctrl_range is None:
        ctrl_range = (-0.5, 0.5)
    
    if force_range is None:
        force_range = (-2.0, 2.0)
    
    actuator = ET.SubElement(actuator_parent, actuator_type)
    actuator.set("name", actuator_name)
    actuator.set("joint", joint_name)
    actuator.set("gear", str(gear))
    actuator.set("ctrlrange", f"{ctrl_range[0]} {ctrl_range[1]}")
    actuator.set("forcerange", f"{force_range[0]} {force_range[1]}")
    
    return actuator


def add_valve_sensor(
    sensor_parent: ET.Element,
    joint_name: str,
    sensor_type: str = "jointpos",
    sensor_name: Optional[str] = None,
) -> ET.Element:
    """
    Add a sensor for valve joint.
    
    Args:
        sensor_parent: Parent sensor container element
        joint_name: Name of joint to sense
        sensor_type: Sensor type ('jointpos', 'jointvel', 'actuatorfrc')
        sensor_name: Sensor name (default: joint_name + "_" + sensor_type)
    
    Returns:
        Created sensor XML element
    
    Example:
        >>> sensor_container = ET.Element("sensor")
        >>> pos_sensor = add_valve_sensor(
        ...     sensor_container,
        ...     "valve_joint",
        ...     "jointpos"
        ... )
    """
    if sensor_name is None:
        sensor_name = f"{joint_name}_{sensor_type}"
    
    sensor = ET.SubElement(sensor_parent, sensor_type)
    sensor.set("name", sensor_name)
    sensor.set("joint", joint_name)
    
    return sensor


def create_valve_body(
    parent_element: ET.Element,
    name: str,
    pos: str = "0 0 0",
    quat: str = "1 0 0 0",
    joint_name: Optional[str] = None,
    joint_range: tuple = (0, 90),
) -> tuple:
    """
    Create complete valve body with joint and visual geometry.
    
    Args:
        parent_element: Parent body or worldbody element
        name: Body name
        pos: Position (default: "0 0 0")
        quat: Orientation quaternion (default: "1 0 0 0")
        joint_name: Joint name (default: name + "_joint")
        joint_range: Joint limits in degrees (default: (0, 90))
    
    Returns:
        Tuple of (body_element, joint_element)
    
    Example:
        >>> worldbody = ET.Element("worldbody")
        >>> body, joint = create_valve_body(
        ...     worldbody,
        ...     "valve1",
        ...     pos="0 0 0.5",
        ...     joint_range=(0, 180)
        ... )
    """
    if joint_name is None:
        joint_name = f"{name}_joint"
    
    # Create body
    body = ET.SubElement(parent_element, "body")
    body.set("name", name)
    body.set("pos", pos)
    body.set("quat", quat)
    
    # Add visual geometry (cylinder for valve handle)
    geom = ET.SubElement(body, "geom")
    geom.set("type", "cylinder")
    geom.set("size", "0.02 0.1")
    geom.set("rgba", "0.8 0.2 0.2 1")
    geom.set("mass", "0.1")
    
    # Add joint
    joint = add_valve_joint(body, joint_name, range=joint_range)
    
    return body, joint


def inject_valve_into_model(
    model_path: str,
    output_path: str,
    body_name: str,
    joint_name: str,
    valve_pos: str = "0 0 0",
    joint_range: tuple = (0, 90),
    add_actuator: bool = True,
    add_sensors: bool = True,
) -> None:
    """
    Inject STEVE valve into existing MJCF model file.
    
    Modifies XML to add valve body, joint, actuator, and sensors.
    
    Args:
        model_path: Path to input MJCF file
        output_path: Path to output MJCF file
        body_name: Name for valve body
        joint_name: Name for valve joint
        valve_pos: Valve position (default: "0 0 0")
        joint_range: Joint limits (default: (0, 90))
        add_actuator: Add motor actuator (default: True)
        add_sensors: Add position/velocity sensors (default: True)
    
    Example:
        >>> inject_valve_into_model(
        ...     "robot.xml",
        ...     "robot_with_valve.xml",
        ...     "valve1",
        ...     "valve_joint",
        ...     valve_pos="0.5 0 1.0",
        ...     joint_range=(0, 180)
        ... )
    """
    # Parse XML
    tree = ET.parse(model_path)
    root = tree.getroot()
    
    # Find or create worldbody
    worldbody = root.find("worldbody")
    if worldbody is None:
        worldbody = ET.SubElement(root, "worldbody")
    
    # Add valve body and joint
    body, joint = create_valve_body(
        worldbody,
        body_name,
        pos=valve_pos,
        joint_name=joint_name,
        joint_range=joint_range,
    )
    
    # Add actuator if requested
    if add_actuator:
        actuator_container = root.find("actuator")
        if actuator_container is None:
            actuator_container = ET.SubElement(root, "actuator")
        
        add_valve_actuator(actuator_container, joint_name)
    
    # Add sensors if requested
    if add_sensors:
        sensor_container = root.find("sensor")
        if sensor_container is None:
            sensor_container = ET.SubElement(root, "sensor")
        
        add_valve_sensor(sensor_container, joint_name, "jointpos")
        add_valve_sensor(sensor_container, joint_name, "jointvel")
    
    # Write output
    tree.write(output_path, encoding="utf-8", xml_declaration=True)

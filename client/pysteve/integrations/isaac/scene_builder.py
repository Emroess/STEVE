"""
USD scene builder for STEVE valve integration in Isaac Sim.
"""

from typing import Optional, Tuple
import numpy as np

try:
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Gf, Sdf
    from omni.isaac.core.utils.stage import get_current_stage
except ImportError:
    raise ImportError(
        "Isaac Sim required. Install NVIDIA Isaac Sim 2023.1+ and ensure "
        "environment is configured correctly."
    )

from pysteve.core.config import ValveConfig


class IsaacSceneBuilder:
    """
    Build USD scenes with STEVE valve representations.
    
    Creates complete valve assemblies with:
    - Visual geometry (handle, housing, shaft)
    - Physics articulation (revolute joint)
    - Collision meshes
    - Material properties
    - Custom STEVE haptic parameters
    
    Args:
        stage: USD stage for scene construction
    
    Example:
        >>> from omni.isaac.core.utils.stage import get_current_stage
        >>> 
        >>> stage = get_current_stage()
        >>> builder = IsaacSceneBuilder(stage)
        >>> 
        >>> # Create valve with config
        >>> config = ValveConfig(viscous=0.1, coulomb=0.01)
        >>> valve_path = builder.create_valve(
        ...     parent_path="/World",
        ...     valve_name="Valve01",
        ...     position=(0, 0, 0.5),
        ...     config=config
        ... )
    """

    def __init__(self, stage: Optional[Usd.Stage] = None):
        self.stage = stage or get_current_stage()

    def create_valve(
        self,
        parent_path: str = "/World",
        valve_name: str = "Valve",
        position: Tuple[float, float, float] = (0, 0, 0),
        config: Optional[ValveConfig] = None,
        visual_scale: float = 1.0,
    ) -> str:
        """
        Create complete valve assembly in USD scene.
        
        Args:
            parent_path: Parent prim path
            valve_name: Name for valve assembly
            position: World position (x, y, z)
            config: Valve configuration (optional)
            visual_scale: Scale factor for visual geometry
        
        Returns:
            Path to created valve root prim
        """
        valve_path = f"{parent_path}/{valve_name}"

        # Create root Xform
        valve_root = UsdGeom.Xform.Define(self.stage, valve_path)
        valve_root.AddTranslateOp().Set(Gf.Vec3f(*position))

        # Create articulation root
        articulation_api = UsdPhysics.ArticulationRootAPI.Apply(valve_root.GetPrim())

        # Create base (fixed)
        base_path = f"{valve_path}/Base"
        self._create_base(base_path, visual_scale)

        # Create handle (moving)
        handle_path = f"{valve_path}/Handle"
        self._create_handle(handle_path, visual_scale)

        # Create revolute joint
        joint_path = f"{valve_path}/RevoluteJoint"
        self._create_joint(joint_path, base_path, handle_path, config)

        # Apply STEVE config as custom attributes
        if config:
            self._apply_steve_config(valve_root.GetPrim(), config)

        print(f"Created valve at {valve_path}")
        return valve_path

    def _create_base(self, base_path: str, scale: float) -> None:
        """Create valve base (housing)."""
        # Create Xform for base
        base = UsdGeom.Xform.Define(self.stage, base_path)

        # Create cylinder for housing
        housing_path = f"{base_path}/Housing"
        housing = UsdGeom.Cylinder.Define(self.stage, housing_path)
        housing.GetRadiusAttr().Set(0.05 * scale)
        housing.GetHeightAttr().Set(0.1 * scale)
        housing.GetAxisAttr().Set("Z")

        # Position housing
        housing.AddTranslateOp().Set(Gf.Vec3f(0, 0, 0))

        # Add collision
        collision_api = UsdPhysics.CollisionAPI.Apply(housing.GetPrim())

        # Add rigid body (fixed)
        mass_api = UsdPhysics.MassAPI.Apply(housing.GetPrim())
        mass_api.GetMassAttr().Set(1.0)

        # Apply material
        self._apply_material(housing.GetPrim(), (0.3, 0.3, 0.3))

    def _create_handle(self, handle_path: str, scale: float) -> None:
        """Create valve handle (moving part)."""
        # Create Xform for handle
        handle = UsdGeom.Xform.Define(self.stage, handle_path)
        handle.AddTranslateOp().Set(Gf.Vec3f(0, 0, 0.06 * scale))

        # Create cylinder for shaft
        shaft_path = f"{handle_path}/Shaft"
        shaft = UsdGeom.Cylinder.Define(self.stage, shaft_path)
        shaft.GetRadiusAttr().Set(0.02 * scale)
        shaft.GetHeightAttr().Set(0.08 * scale)
        shaft.GetAxisAttr().Set("Z")

        # Create handle grip
        grip_path = f"{handle_path}/Grip"
        grip = UsdGeom.Cylinder.Define(self.stage, grip_path)
        grip.GetRadiusAttr().Set(0.08 * scale)
        grip.GetHeightAttr().Set(0.02 * scale)
        grip.GetAxisAttr().Set("Z")
        grip.AddTranslateOp().Set(Gf.Vec3f(0, 0, 0.05 * scale))

        # Add collision to shaft
        collision_api = UsdPhysics.CollisionAPI.Apply(shaft.GetPrim())

        # Add rigid body
        mass_api = UsdPhysics.MassAPI.Apply(handle.GetPrim())
        mass_api.GetMassAttr().Set(0.2)

        # Apply material
        self._apply_material(shaft.GetPrim(), (0.8, 0.2, 0.2))
        self._apply_material(grip.GetPrim(), (0.2, 0.2, 0.8))

    def _create_joint(
        self,
        joint_path: str,
        base_path: str,
        handle_path: str,
        config: Optional[ValveConfig],
    ) -> None:
        """Create revolute joint between base and handle."""
        # Create revolute joint
        joint = UsdPhysics.RevoluteJoint.Define(self.stage, joint_path)

        # Set body relationships
        joint.GetBody0Rel().SetTargets([Sdf.Path(base_path)])
        joint.GetBody1Rel().SetTargets([Sdf.Path(handle_path)])

        # Set joint axis (Z-axis)
        joint.GetAxisAttr().Set("Z")

        # Set joint limits if config provided
        if config:
            # Convert degrees to radians
            lower_rad = np.deg2rad(config.closed_position)
            upper_rad = np.deg2rad(config.open_position)

            joint.GetLowerLimitAttr().Set(lower_rad)
            joint.GetUpperLimitAttr().Set(upper_rad)

            # Enable limits
            limit_api = UsdPhysics.ArticulationJointAPI.Apply(joint.GetPrim())

        # Set damping (approximate viscous damping)
        if config:
            drive_api = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
            drive_api.GetDampingAttr().Set(config.viscous * 100)  # Scale for Isaac Sim
            drive_api.GetStiffnessAttr().Set(0.0)  # No spring for valve

    def _apply_steve_config(self, prim: Usd.Prim, config: ValveConfig) -> None:
        """Apply STEVE configuration as custom USD attributes."""
        # Create steve: namespace attributes
        prim.CreateAttribute("steve:viscous", Sdf.ValueTypeNames.Float).Set(config.viscous)
        prim.CreateAttribute("steve:coulomb", Sdf.ValueTypeNames.Float).Set(config.coulomb)
        prim.CreateAttribute("steve:wall_stiffness", Sdf.ValueTypeNames.Float).Set(
            config.wall_stiffness
        )
        prim.CreateAttribute("steve:wall_damping", Sdf.ValueTypeNames.Float).Set(
            config.wall_damping
        )
        prim.CreateAttribute("steve:smoothing", Sdf.ValueTypeNames.Float).Set(config.smoothing)
        prim.CreateAttribute("steve:torque_limit", Sdf.ValueTypeNames.Float).Set(
            config.torque_limit
        )
        prim.CreateAttribute("steve:travel", Sdf.ValueTypeNames.Int).Set(config.travel)
        prim.CreateAttribute("steve:closed_position", Sdf.ValueTypeNames.Int).Set(
            config.closed_position
        )
        prim.CreateAttribute("steve:open_position", Sdf.ValueTypeNames.Int).Set(
            config.open_position
        )

    def _apply_material(
        self, prim: Usd.Prim, color: Tuple[float, float, float]
    ) -> None:
        """Apply simple colored material to prim."""
        # Create material
        material_path = f"{prim.GetPath()}_material"
        material = UsdShade.Material.Define(self.stage, material_path)

        # Create shader
        shader = UsdShade.Shader.Define(self.stage, f"{material_path}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.1)

        # Connect shader to material
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

        # Bind material to prim
        UsdShade.MaterialBindingAPI(prim).Bind(material)

    def create_valve_array(
        self,
        parent_path: str = "/World",
        array_name: str = "ValveArray",
        num_valves: int = 4,
        spacing: float = 0.3,
        config: Optional[ValveConfig] = None,
    ) -> list:
        """
        Create array of valves for multi-device testing.
        
        Args:
            parent_path: Parent prim path
            array_name: Name for array
            num_valves: Number of valves to create
            spacing: Spacing between valves (meters)
            config: Shared valve configuration
        
        Returns:
            List of valve prim paths
        """
        array_path = f"{parent_path}/{array_name}"
        array_root = UsdGeom.Xform.Define(self.stage, array_path)

        valve_paths = []
        for i in range(num_valves):
            # Calculate position in grid
            row = i // 2
            col = i % 2

            x = col * spacing
            y = row * spacing
            position = (x, y, 0)

            # Create valve
            valve_path = self.create_valve(
                parent_path=array_path,
                valve_name=f"Valve{i:02d}",
                position=position,
                config=config,
            )
            valve_paths.append(valve_path)

        print(f"Created valve array with {num_valves} valves at {array_path}")
        return valve_paths

    def add_camera(
        self,
        parent_path: str = "/World",
        camera_name: str = "Camera",
        position: Tuple[float, float, float] = (0.5, 0.5, 0.5),
        look_at: Tuple[float, float, float] = (0, 0, 0),
    ) -> str:
        """
        Add camera to scene for visualization.
        
        Args:
            parent_path: Parent prim path
            camera_name: Camera name
            position: Camera position
            look_at: Target point to look at
        
        Returns:
            Camera prim path
        """
        camera_path = f"{parent_path}/{camera_name}"
        camera = UsdGeom.Camera.Define(self.stage, camera_path)

        # Set position
        camera.AddTranslateOp().Set(Gf.Vec3f(*position))

        # Calculate look-at rotation
        forward = np.array(look_at) - np.array(position)
        forward = forward / np.linalg.norm(forward)

        # Simple rotation (simplified - should use proper quaternion)
        # This is a placeholder - actual implementation needs proper rotation

        print(f"Created camera at {camera_path}")
        return camera_path

    def add_lighting(self, parent_path: str = "/World") -> None:
        """Add basic lighting to scene."""
        # Dome light
        dome_light_path = f"{parent_path}/DomeLight"
        dome_light = UsdGeom.DomeLight.Define(self.stage, dome_light_path)
        dome_light.CreateIntensityAttr().Set(1000)

        # Distant light (sun)
        sun_light_path = f"{parent_path}/SunLight"
        sun_light = UsdGeom.DistantLight.Define(self.stage, sun_light_path)
        sun_light.CreateIntensityAttr().Set(3000)
        sun_light.AddRotateXYZOp().Set(Gf.Vec3f(-45, 0, 0))

        print(f"Added lighting to {parent_path}")

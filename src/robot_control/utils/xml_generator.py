"""Generate MuJoCo XML for NAMO planning from ArUco observations.

Converts robot_control Observations to the XML format used by namo_cpp.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
from xml.dom import minidom
from xml.etree import ElementTree as ET

from robot_control.utils.wavefront import WavefrontPlanner, WavefrontConfig


@dataclass
class WorkspaceBounds:
    """Workspace boundaries in meters."""
    x_min: float
    x_max: float
    y_min: float
    y_max: float


@dataclass
class ObjectSpec:
    """Object specification for XML generation."""
    x: float  # meters
    y: float  # meters
    theta: float  # degrees
    half_width: float  # meters (half-dimension)
    half_depth: float  # meters (half-dimension)
    half_height: float = 0.025  # meters (default 2.5cm, 5cm total)
    is_static: bool = False  # True for walls


@dataclass
class RobotSpec:
    """Robot specification for XML generation."""
    x: float  # meters
    y: float  # meters


@dataclass
class GoalSpec:
    """Goal specification for XML generation."""
    x: float  # meters
    y: float  # meters


class NAMOXMLGenerator:
    # Last wavefront planner (for debugging/visualization)
    _last_wavefront: Optional[WavefrontPlanner] = None
    """Generate MuJoCo XML for NAMO planning.

    Matches the format of env_config_1416a.xml exactly.

    Usage:
        generator = NAMOXMLGenerator()

        # From ArUco observations (cm, degrees)
        xml_str = generator.from_observation(
            robot_x_cm=10.0, robot_y_cm=20.0,
            objects={
                "brick_1": (15.0, 25.0, 45.0, 8.0, 4.0),  # x, y, theta, width_cm, depth_cm
            },
            goal_x_cm=50.0, goal_y_cm=40.0,
            workspace_bounds_cm=(0, 60, 0, 80),
        )

        # Scale up to simulation scale (6x) for physics compatibility:
        xml_str = generator.from_observation(..., scale_factor=6.0)

        # Or directly in meters
        xml_str = generator.generate(
            robot=RobotSpec(x=0.1, y=0.2),
            objects={"obstacle_1_movable": ObjectSpec(...)},
            goal=GoalSpec(x=0.5, y=0.4),
            bounds=WorkspaceBounds(-0.3, 0.3, -0.4, 0.4),
        )
    """

    # Robot parameters (real robot scale - will be multiplied by scale_factor)
    ROBOT_RADIUS_BASE = 0.025  # 2.5cm radius sphere (25mm)
    ROBOT_MASS = 5.0
    ROBOT_FRICTION = "1.0 0.005 0.0001"

    # Object parameters (real robot scale - will be multiplied by scale_factor)
    OBJECT_HEIGHT_BASE = 0.025  # 2.5cm half-height (5cm total)
    OBJECT_MASS = 0.1
    OBJECT_FRICTION = "0.0 0.005 0.001"
    OBJECT_COLOR = "1 1 0 1"  # yellow

    # Wall parameters (real robot scale - will be multiplied by scale_factor)
    WALL_THICKNESS_BASE = 0.01  # 1cm half-thickness (2cm total)
    WALL_HEIGHT_BASE = 0.05  # 5cm half-height (10cm total)
    WALL_COLOR = "0.800000011920929 0.800000011920929 0.800000011920929 1.0"

    # Floor friction
    FLOOR_FRICTION = "0.5 0.005 0.001"

    # Collision avoidance
    MIN_SEPARATION_BASE = 0.005  # 5mm margin (matches namo_cpp wavefront inflation)
    COLLISION_EXTRA_MARGIN = 0.08  # Extra 8cm margin when resolving robot collisions

    def __init__(self, scale_factor: float = 1.0):
        """Initialize generator with optional scale factor.

        Args:
            scale_factor: Multiply all positions and sizes by this factor.
                          Use 6.0 to scale from real robot (2.5cm) to simulation (15cm).
                          Physics parameters (mass, friction) remain unchanged.
        """
        self.scale_factor = scale_factor

        # Scaled parameters
        self.ROBOT_RADIUS = self.ROBOT_RADIUS_BASE * scale_factor
        self.OBJECT_HEIGHT = self.OBJECT_HEIGHT_BASE * scale_factor
        self.WALL_THICKNESS = self.WALL_THICKNESS_BASE * scale_factor
        self.WALL_HEIGHT = self.WALL_HEIGHT_BASE * scale_factor
        # Fixed 5mm margin (NOT scaled) - matches namo_cpp's wavefront inflation
        self.MIN_SEPARATION_M = 0.005

        # Last resolved robot position (after collision resolution)
        self._last_robot_pos: Optional[Tuple[float, float]] = None

    def get_resolved_robot_pos(self) -> Optional[Tuple[float, float]]:
        """Get the last collision-resolved robot position (x, y) in meters."""
        return self._last_robot_pos

    def from_observation(
        self,
        robot_x_cm: float,
        robot_y_cm: float,
        objects: Dict[str, Tuple[float, float, float, float, float, float, bool]],
        goal_x_cm: float,
        goal_y_cm: float,
        workspace_bounds_cm: Tuple[float, float, float, float],
        internal_walls: Optional[List[Tuple[float, float, float, float]]] = None,
    ) -> str:
        """Generate XML from ArUco observations in centimeters.

        Workspace has origin at bottom-left with +Y up (same as MuJoCo).
        All positions and sizes are scaled by self.scale_factor (set in __init__).

        Args:
            robot_x_cm: Robot X position in cm
            robot_y_cm: Robot Y position in cm
            objects: Dict mapping object name to (x_cm, y_cm, theta_deg, width_cm, depth_cm, height_cm, is_static)
            goal_x_cm: Goal X position in cm
            goal_y_cm: Goal Y position in cm
            workspace_bounds_cm: (x_min, x_max, y_min, y_max) in cm
            internal_walls: Optional list of (x_cm, y_cm, width_cm, depth_cm) for internal walls

        Returns:
            XML string
        """
        s = self.scale_factor  # shorthand

        # No Y-flip needed - workspace already has bottom-left origin with +Y up
        # Convert to meters and apply scale factor
        robot = RobotSpec(
            x=robot_x_cm / 100.0 * s,
            y=robot_y_cm / 100.0 * s,
        )

        obj_specs = {}
        movable_count = 0
        static_count = 0
        for name, obj_data in objects.items():
            # Support both old format (5 values) and new format (7 values)
            if len(obj_data) == 5:
                x, y, theta, w, d = obj_data
                h = 5.0  # default height 5cm
                is_static = False
            else:
                x, y, theta, w, d, h, is_static = obj_data

            if is_static:
                static_count += 1
                obj_name = f"wall_{4 + static_count}"  # wall_5, wall_6, etc. (1-4 are boundary walls)
            else:
                movable_count += 1
                obj_name = f"obstacle_{movable_count}_movable"

            obj_specs[obj_name] = ObjectSpec(
                x=x / 100.0 * s,
                y=y / 100.0 * s,
                theta=theta,  # angles don't scale
                half_width=d / 200.0 * s,  # swap: depth -> X
                half_depth=w / 200.0 * s,  # swap: width -> Y
                half_height=h / 200.0 * s,
                is_static=is_static,
            )

        goal = GoalSpec(
            x=goal_x_cm / 100.0 * s,
            y=goal_y_cm / 100.0 * s,
        )

        bounds = WorkspaceBounds(
            x_min=workspace_bounds_cm[0] / 100.0 * s,
            x_max=workspace_bounds_cm[1] / 100.0 * s,
            y_min=workspace_bounds_cm[2] / 100.0 * s,
            y_max=workspace_bounds_cm[3] / 100.0 * s,
        )

        walls = None
        if internal_walls:
            walls = [
                (x / 100.0 * s, y / 100.0 * s, w / 200.0 * s, d / 200.0 * s)
                for x, y, w, d in internal_walls
            ]

        return self.generate(robot, obj_specs, goal, bounds, walls)

    def generate(
        self,
        robot: RobotSpec,
        objects: Dict[str, ObjectSpec],
        goal: GoalSpec,
        bounds: WorkspaceBounds,
        internal_walls: Optional[List[Tuple[float, float, float, float]]] = None,
    ) -> str:
        """Generate XML from specifications in meters.

        Args:
            robot: Robot position
            objects: Dict mapping "obstacle_N_movable" to ObjectSpec
            goal: Goal position
            bounds: Workspace boundaries
            internal_walls: Optional list of (x, y, half_width, half_depth) in meters

        Returns:
            XML string
        """
        root = ET.Element("mujoco", model="generated_environment")

        # Option
        ET.SubElement(root, "option",
                      timestep="0.01",
                      integrator="RK4",
                      cone="elliptic")

        # Default
        default = ET.SubElement(root, "default")
        ET.SubElement(default, "geom", density="1")

        # Asset
        asset = ET.SubElement(root, "asset")
        ET.SubElement(asset, "texture",
                      builtin="gradient", height="3072",
                      rgb1="0.3 0.5 0.7", rgb2="0 0 0",
                      type="skybox", width="512")
        ET.SubElement(asset, "texture",
                      builtin="checker", height="300",
                      mark="edge", markrgb="0.8 0.8 0.8",
                      name="groundplane",
                      rgb1="0.2 0.3 0.4", rgb2="0.1 0.2 0.3",
                      type="2d", width="300")
        ET.SubElement(asset, "material",
                      name="groundplane", reflectance="0.2",
                      texrepeat="5 5", texture="groundplane",
                      texuniform="true")
        ET.SubElement(asset, "material",
                      name="robot", rgba="1.0 1.0 0.0 1.0")

        # Worldbody
        worldbody = ET.SubElement(root, "worldbody")

        # Light
        ET.SubElement(worldbody, "light",
                      dir="0 0 -1", directional="true", pos="0 0 1.5")

        # Floor
        ET.SubElement(worldbody, "geom",
                      condim="4", friction=self.FLOOR_FRICTION,
                      material="groundplane", name="floor",
                      size="0 0 0.05", type="plane")

        # Walls
        self._add_walls(worldbody, bounds, internal_walls)

        # Place robot randomly in its reachable region (based on wavefront)
        robot = self._resolve_robot_collisions(robot, objects, bounds, internal_walls)

        # Store resolved robot position for later access
        self._last_robot_pos = (robot.x, robot.y)

        # Robot
        self._add_robot(worldbody, robot)

        # Objects
        for name, obj in objects.items():
            self._add_object(worldbody, name, obj)

        # Goal (size scales with scale_factor)
        goal_size = 0.05 * self.scale_factor
        ET.SubElement(worldbody, "site",
                      name="goal", type="sphere",
                      size=f"{goal_size} {goal_size} {goal_size}",
                      rgba="0 1 0 0.5",
                      pos=f"{goal.x} {goal.y} 0.0")

        # Actuator
        actuator = ET.SubElement(root, "actuator")
        ET.SubElement(actuator, "motor",
                      name="actuator_x", joint="joint_x",
                      gear="1", ctrlrange="-1 1")
        ET.SubElement(actuator, "motor",
                      name="actuator_y", joint="joint_y",
                      gear="1", ctrlrange="-1 1")

        # Pretty print
        xml_str = ET.tostring(root, encoding="unicode")
        dom = minidom.parseString(xml_str)
        return dom.toprettyxml(indent="  ").replace('<?xml version="1.0" ?>\n', '<?xml version="1.0" ?>\n')

    def _add_walls(
        self,
        worldbody: ET.Element,
        bounds: WorkspaceBounds,
        internal_walls: Optional[List[Tuple[float, float, float, float]]] = None,
    ) -> None:
        """Add boundary walls and optional internal walls.

        Walls are placed OUTSIDE the bounds so inner space is exactly bounds size.
        """
        walls_body = ET.SubElement(worldbody, "body", name="walls")

        x_center = (bounds.x_min + bounds.x_max) / 2
        y_center = (bounds.y_min + bounds.y_max) / 2
        x_extent = (bounds.x_max - bounds.x_min) / 2
        y_extent = (bounds.y_max - bounds.y_min) / 2
        t = self.WALL_THICKNESS  # half-thickness

        # Left wall (placed outside, inner edge at x_min)
        ET.SubElement(walls_body, "geom",
                      name="wall_1", condim="4",
                      pos=f"{bounds.x_min - t} {y_center} {self.WALL_HEIGHT}",
                      rgba=self.WALL_COLOR,
                      size=f"{t} {y_extent + 2*t} {self.WALL_HEIGHT}",
                      type="box")

        # Right wall (placed outside, inner edge at x_max)
        ET.SubElement(walls_body, "geom",
                      name="wall_2", condim="4",
                      pos=f"{bounds.x_max + t} {y_center} {self.WALL_HEIGHT}",
                      rgba=self.WALL_COLOR,
                      size=f"{t} {y_extent + 2*t} {self.WALL_HEIGHT}",
                      type="box")

        # Bottom wall (placed outside, inner edge at y_min)
        ET.SubElement(walls_body, "geom",
                      name="wall_3", condim="4",
                      pos=f"{x_center} {bounds.y_min - t} {self.WALL_HEIGHT}",
                      rgba=self.WALL_COLOR,
                      size=f"{x_extent} {t} {self.WALL_HEIGHT}",
                      type="box")

        # Top wall (placed outside, inner edge at y_max)
        ET.SubElement(walls_body, "geom",
                      name="wall_4", condim="4",
                      pos=f"{x_center} {bounds.y_max + t} {self.WALL_HEIGHT}",
                      rgba=self.WALL_COLOR,
                      size=f"{x_extent} {t} {self.WALL_HEIGHT}",
                      type="box")

        # Internal walls
        if internal_walls:
            for i, (x, y, hw, hd) in enumerate(internal_walls, start=5):
                ET.SubElement(walls_body, "geom",
                              name=f"wall_{i}", condim="4",
                              pos=f"{x} {y} {self.WALL_HEIGHT}",
                              rgba=self.WALL_COLOR,
                              size=f"{hw} {hd} {self.WALL_HEIGHT}",
                              type="box")

    def _add_robot(self, worldbody: ET.Element, robot: RobotSpec) -> None:
        """Add robot body with slide joints."""
        robot_body = ET.SubElement(worldbody, "body", name="robot")

        ET.SubElement(robot_body, "joint",
                      name="joint_x", type="slide",
                      pos="0 0 0", axis="1 0 0")
        ET.SubElement(robot_body, "joint",
                      name="joint_y", type="slide",
                      pos="0 0 0", axis="0 1 0")

        ET.SubElement(robot_body, "geom",
                      name="robot", type="sphere",
                      pos=f"{robot.x} {robot.y} {self.ROBOT_RADIUS}",
                      size=f"{self.ROBOT_RADIUS} {self.ROBOT_RADIUS} {self.ROBOT_RADIUS}",
                      mass=str(self.ROBOT_MASS),
                      friction=self.ROBOT_FRICTION,
                      condim="4")

    def _add_object(self, worldbody: ET.Element, name: str, obj: ObjectSpec) -> None:
        """Add object body (static or movable)."""
        obj_body = ET.SubElement(worldbody, "body", name=name)

        # Use object-specific height, or default
        half_height = obj.half_height if obj.half_height > 0 else self.OBJECT_HEIGHT
        z_pos = half_height  # center is at half-height above ground

        if obj.is_static:
            # Static object (wall) - no joint, gray color
            ET.SubElement(obj_body, "geom",
                          name=name, condim="4",
                          pos=f"{obj.x} {obj.y} {z_pos}",
                          euler=f"0 0 {obj.theta}",
                          rgba=self.WALL_COLOR,
                          size=f"{obj.half_width} {obj.half_depth} {half_height}",
                          type="box")
        else:
            # Movable object - free joint, yellow color
            ET.SubElement(obj_body, "geom",
                          name=name, condim="4",
                          pos=f"{obj.x} {obj.y} {z_pos}",
                          euler=f"0 0 {obj.theta}",
                          friction=self.OBJECT_FRICTION,
                          rgba=self.OBJECT_COLOR,
                          size=f"{obj.half_width} {obj.half_depth} {half_height}",
                          type="box",
                          mass=str(self.OBJECT_MASS))
            ET.SubElement(obj_body, "joint", type="free")

    def _resolve_robot_collisions(
        self,
        robot: RobotSpec,
        objects: Dict[str, ObjectSpec],
        bounds: WorkspaceBounds,
        internal_walls: Optional[List[Tuple[float, float, float, float]]] = None,
    ) -> RobotSpec:
        """Place robot randomly in its reachable region using wavefront planner.

        Args:
            robot: Robot position
            objects: Dict of objects
            bounds: Workspace boundaries
            internal_walls: Optional list of (x, y, half_width, half_depth) in meters

        Returns:
            Random position in region reachable from robot's current position
        """
        # Build wavefront grid (fixed 1cm resolution, matching namo_cpp's WavefrontGrid)
        # Use extra margin for collision resolution to keep robot further from obstacles
        config = WavefrontConfig(
            resolution=0.01,  # 1cm grid (matches namo_cpp WavefrontGrid.kResolution)
            robot_radius=self.ROBOT_RADIUS,
            inflation_margin=self.MIN_SEPARATION_M + self.COLLISION_EXTRA_MARGIN,
        )
        planner = WavefrontPlanner(config)

        # Convert objects to format expected by wavefront
        obj_data = {}
        for name, obj in objects.items():
            obj_data[name] = (obj.x, obj.y, obj.half_width, obj.half_depth, obj.theta)

        # Add internal walls to wavefront (axis-aligned, theta=0)
        if internal_walls:
            for i, (x, y, hw, hd) in enumerate(internal_walls, start=5):
                obj_data[f"wall_{i}"] = (x, y, hw, hd, 0)

        # Add boundary walls to wavefront
        x_center = (bounds.x_min + bounds.x_max) / 2
        y_center = (bounds.y_min + bounds.y_max) / 2
        x_extent = (bounds.x_max - bounds.x_min) / 2
        y_extent = (bounds.y_max - bounds.y_min) / 2
        t = self.WALL_THICKNESS

        # Walls placed OUTSIDE bounds (inner edge at boundary)
        obj_data["wall_1"] = (bounds.x_min - t, y_center, t, y_extent + 2*t, 0)  # Left
        obj_data["wall_2"] = (bounds.x_max + t, y_center, t, y_extent + 2*t, 0)  # Right
        obj_data["wall_3"] = (x_center, bounds.y_min - t, x_extent, t, 0)  # Bottom
        obj_data["wall_4"] = (x_center, bounds.y_max + t, x_extent, t, 0)  # Top

        # Expand bounds slightly to include walls in the grid
        expanded_bounds = (
            bounds.x_min - 2*t,
            bounds.x_max + 2*t,
            bounds.y_min - 2*t,
            bounds.y_max + 2*t,
        )

        planner.build_grid(expanded_bounds, obj_data)

        # Store for later access
        self._last_wavefront = planner

        # If robot position is free (not close to objects), keep it
        if planner.is_free(robot.x, robot.y):
            return robot

        # Robot is too close to objects - find nearest free cell as BFS start
        nearest = planner.find_nearest_free(robot.x, robot.y)
        if nearest is None:
            print(f"\n[WARNING] Robot at ({robot.x*100:.1f}, {robot.y*100:.1f}) cm - no free space found!")
            return robot

        # Place robot randomly in region reachable from nearest free position
        result = planner.get_random_reachable_cell(nearest)
        if result is not None:
            new_x, new_y = result
            dist = math.sqrt((new_x - robot.x)**2 + (new_y - robot.y)**2)
            print(f"\n[RANDOM PLACEMENT]")
            print(f"  Robot too close to obstacles - placing randomly in reachable region")
            print(f"  Original position: ({robot.x*100:.1f}, {robot.y*100:.1f}) cm")
            print(f"  New position:      ({new_x*100:.1f}, {new_y*100:.1f}) cm")
            print(f"  Moved by:          {dist*100:.1f} cm\n")
            return RobotSpec(x=new_x, y=new_y)

        print(f"\n[WARNING] Could not find reachable region from robot position!")
        return robot

    def save(self, xml_str: str, filepath: str) -> None:
        """Save XML string to file."""
        with open(filepath, "w") as f:
            f.write(xml_str)

    def get_wavefront(self) -> Optional[WavefrontPlanner]:
        """Get the last wavefront planner used for collision resolution."""
        return self._last_wavefront

    def save_wavefront(self, filepath: str, robot_pos: Optional[Tuple[float, float]] = None) -> None:
        """Save the wavefront grid to file.

        Args:
            filepath: Output path (.png for image, .npy for numpy)
            robot_pos: Optional robot position (x, y) in meters to mark
        """
        if self._last_wavefront is not None:
            self._last_wavefront.save(filepath, robot_pos)
        else:
            print("[XMLGenerator] No wavefront available to save")

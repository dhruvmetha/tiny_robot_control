"""Generate MuJoCo XML for NAMO planning from ArUco observations.

Converts robot_control Observations to the XML format used by namo_cpp.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
from xml.dom import minidom
from xml.etree import ElementTree as ET

from robot_control.utils.wavefront_inflation_config import get_wavefront_inflation_config
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
    ROBOT_RADIUS_BASE = 0.03  # 3cm radius sphere (inflated from 2.5cm for safety margin)

    # Object parameters (real robot scale - will be multiplied by scale_factor)
    OBJECT_HEIGHT_BASE = 0.025  # 2.5cm half-height (5cm total)
    OBJECT_MASS = 0.1
    OBJECT_FRICTION = "1 0.005 0.0001"   # Exact match to movable obstacle friction in
                                          # namo_cpp/data/nominal_primitive_scene_*_1x_car.xml.
                                          # Was "0 0.1 0.1" (frictionless slide, high torsional/rolling)
                                          # → runtime planning sim disagreed with the primitive DB.
    OBJECT_COLOR = "1 1 0 1"  # yellow

    # Wall parameters (real robot scale - will be multiplied by scale_factor)
    WALL_THICKNESS_BASE = 0.01  # 1cm half-thickness (2cm total)
    WALL_HEIGHT_BASE = 0.05  # 5cm half-height (10cm total)
    WALL_COLOR = "0.800000011920929 0.800000011920929 0.800000011920929 1.0"
    STATIC_FRICTION = "1 0.005 0.0001"   # Matches namo_cpp primitive scene walls, which have
                                          # no explicit friction attribute → MuJoCo defaults
                                          # (1, 0.005, 0.0001). Was "100000 100000 100000" — walls
                                          # don't move (no joint) so the magnitude is cosmetic, but
                                          # contact dynamics during wall-grazing differ; matching
                                          # exactly keeps planner-sim and primitive-sim in lockstep.

    # Floor friction
    FLOOR_FRICTION = "0.5 0.005 0.001"

    # Collision avoidance defaults (overridden by wavefront_inflation.yaml when present)
    MIN_SEPARATION_BASE = 0.005
    COLLISION_EXTRA_MARGIN = 0.08

    def __init__(
        self,
        scale_factor: float = 1.0,
        robot_radius_cm: Optional[float] = None,
        robot_model: str = "car",
    ):
        """Initialize generator with optional scale factor.

        Args:
            scale_factor: Fixed at 1.0.
            robot_radius_cm: Robot radius in cm. If None, uses ROBOT_RADIUS_BASE (3cm).
                            For rectangular robots, use the rotation-safe diagonal radius.
            robot_model: Fixed at ``car``.
        """
        if robot_model != "car" or abs(scale_factor - 1.0) > 1e-9:
            raise ValueError(
                "NAMO XML generation supports only robot_model='car' "
                "with scale_factor=1.0"
            )
        self._robot_model = robot_model

        # Resolve the absolute path to little_car.xml once at construction
        # time. We use an absolute path because the generated planning XML
        # ends up in a tempfile (/tmp/namo_env_*.xml) and a relative
        # <include> would resolve from /tmp.
        #
        # The car body inside little_car.xml fixes the freejoint spawn at
        # (0, 0, 0.01) — we can't override that through a top-level
        # <include>. The planning_service handles this by constructing the
        # env with skip_warmup=True, calling env.set_robot_pose() to
        # teleport to the live observation pose, then calling env.warm_up()
        # so the 3 physics ticks integrate from the correct starting state
        # instead of from inside whatever obstacle happens to be at the
        # workspace origin.
        from pathlib import Path as _P
        from robot_control.planner.namo_binding_loader import resolve_namo_cpp_dir
        here = _P(__file__).resolve()
        candidate = (
            resolve_namo_cpp_dir(here)
            / "test_xml"
            / "little-car-modeling-package"
            / "assets"
            / "mjcf"
            / "little_car.xml"
        )
        if not candidate.exists():
            raise FileNotFoundError(
                f"robot_model='car' requires little_car.xml at "
                f"{candidate}, but the file is not there. Check the "
                f"NAMO checkout is available."
            )
        self._car_xml_abs_path = str(candidate)
        self.scale_factor = scale_factor

        # Use provided robot radius or default
        if robot_radius_cm is not None:
            robot_radius_base = robot_radius_cm / 100.0
        else:
            robot_radius_base = self.ROBOT_RADIUS_BASE

        # Scaled parameters
        self.ROBOT_RADIUS = robot_radius_base * scale_factor
        self.OBJECT_HEIGHT = self.OBJECT_HEIGHT_BASE * scale_factor
        self.WALL_THICKNESS = self.WALL_THICKNESS_BASE * scale_factor
        self.WALL_HEIGHT = self.WALL_HEIGHT_BASE * scale_factor

        inflation_cfg = get_wavefront_inflation_config()
        self.MIN_SEPARATION_M = inflation_cfg.xml_min_separation_m
        self.COLLISION_EXTRA_MARGIN = inflation_cfg.xml_collision_additional_margin_m

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

        # Option block. Sphere uses RK4 at 10 ms — classic NAMO setting.
        # Car needs implicitfast + smaller timestep + more solver iterations
        # to keep wheel-floor contact stable through pushes. Matches the
        # namo_cpp/data/nominal_primitive_scene_*_1x_car.xml option block.
        if self._robot_model == "car":
            ET.SubElement(root, "option",
                          cone="elliptic",
                          integrator="implicitfast",
                          iterations="100",
                          timestep="0.002")
        else:
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

        # Car body: <include> at TOP LEVEL (not inside worldbody). The
        # included file carries a <compiler> element which MuJoCo only
        # accepts as a model-level child — a worldbody-nested include
        # errors out with "Schema violation: unrecognized element
        # compiler". The car body itself ends up inside worldbody after
        # MuJoCo merges the include.
        if self._robot_model == "car":
            ET.SubElement(root, "include", file=self._car_xml_abs_path)

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

        # The car is included at top level above. Its body lives inside
        # little_car.xml with a fixed spawn pos, and
        # the planning_service teleports it to (robot.x, robot.y) after env
        # construction — before warm_up runs — so the physics warm-up sees
        # the correct starting state.

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
                      friction=self.STATIC_FRICTION,
                      rgba=self.WALL_COLOR,
                      size=f"{t} {y_extent + 2*t} {self.WALL_HEIGHT}",
                      type="box")

        # Right wall (placed outside, inner edge at x_max)
        ET.SubElement(walls_body, "geom",
                      name="wall_2", condim="4",
                      pos=f"{bounds.x_max + t} {y_center} {self.WALL_HEIGHT}",
                      friction=self.STATIC_FRICTION,
                      rgba=self.WALL_COLOR,
                      size=f"{t} {y_extent + 2*t} {self.WALL_HEIGHT}",
                      type="box")

        # Bottom wall (placed outside, inner edge at y_min)
        ET.SubElement(walls_body, "geom",
                      name="wall_3", condim="4",
                      pos=f"{x_center} {bounds.y_min - t} {self.WALL_HEIGHT}",
                      friction=self.STATIC_FRICTION,
                      rgba=self.WALL_COLOR,
                      size=f"{x_extent} {t} {self.WALL_HEIGHT}",
                      type="box")

        # Top wall (placed outside, inner edge at y_max)
        ET.SubElement(walls_body, "geom",
                      name="wall_4", condim="4",
                      pos=f"{x_center} {bounds.y_max + t} {self.WALL_HEIGHT}",
                      friction=self.STATIC_FRICTION,
                      rgba=self.WALL_COLOR,
                      size=f"{x_extent} {t} {self.WALL_HEIGHT}",
                      type="box")

        # Internal walls
        if internal_walls:
            for i, (x, y, hw, hd) in enumerate(internal_walls, start=5):
                ET.SubElement(walls_body, "geom",
                              name=f"wall_{i}", condim="4",
                              pos=f"{x} {y} {self.WALL_HEIGHT}",
                              friction=self.STATIC_FRICTION,
                              rgba=self.WALL_COLOR,
                              size=f"{hw} {hd} {self.WALL_HEIGHT}",
                              type="box")

    def _add_object(self, worldbody: ET.Element, name: str, obj: ObjectSpec) -> None:
        """Add object body (static or movable)."""
        obj_body = ET.SubElement(worldbody, "body", name=name)

        # Use object-specific height, or default
        half_height = obj.half_height if obj.half_height > 0 else self.OBJECT_HEIGHT
        z_pos = half_height  # center is at half-height above ground

        # Use quaternion instead of euler. Euler angles are subject to MuJoCo's
        # <compiler angle="..."> setting (degree vs radian). When the car body
        # is <include>d via little_car.xml, the included file declares
        # angle="radian" and that setting wins model-wide — making the parent's
        # euler="0 0 9.58" attributes get interpreted as 9.58 *radians*,
        # rotating objects by ~180° vs the intended 9.58°. Quaternions are
        # unit-free, immune to this.
        theta_rad = math.radians(obj.theta)
        qw = math.cos(theta_rad / 2.0)
        qz = math.sin(theta_rad / 2.0)

        if obj.is_static:
            # Static object (wall) - no joint, gray color, very high friction
            ET.SubElement(obj_body, "geom",
                          name=name, condim="4",
                          pos=f"{obj.x} {obj.y} {z_pos}",
                          quat=f"{qw} 0 0 {qz}",
                          friction=self.STATIC_FRICTION,
                          rgba=self.WALL_COLOR,
                          size=f"{obj.half_width} {obj.half_depth} {half_height}",
                          type="box")
        else:
            # Movable object - free joint, yellow color
            ET.SubElement(obj_body, "geom",
                          name=name, condim="4",
                          pos=f"{obj.x} {obj.y} {z_pos}",
                          quat=f"{qw} 0 0 {qz}",
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

        # Robot is in an inflated-obstacle cell. The previous behavior was
        # find_nearest_free → get_random_reachable_cell, which is Euclidean-
        # nearest and can land in a free cell in a *different* connected
        # component (i.e. on the other side of an obstacle). That produced a
        # planner that said "goal REACHABLE" because the virtual robot was
        # in the goal's region, but the real robot at its actual position
        # could not navigate to the goal because the regions were disconnected.
        # See run log 2026-05-20 002529 for the failure mode.
        #
        # Replace with apply_trapped_start_recovery: same approach the C++
        # wavefront uses. It clears cells around the robot's real position
        # so the BFS can escape via free neighbors. Robot stays where it is;
        # no cross-component teleport.
        planner.apply_trapped_start_recovery((robot.x, robot.y))
        if planner.is_free(robot.x, robot.y):
            print(
                f"\n[Trapped-start recovery] Robot at "
                f"({robot.x*100:.1f}, {robot.y*100:.1f}) cm was in an inflated-"
                f"obstacle cell; cleared local cells so the planner can BFS "
                f"from here. Position unchanged.\n"
            )
            return robot

        print(
            f"\n[WARNING] Robot at ({robot.x*100:.1f}, {robot.y*100:.1f}) cm — "
            f"deeply trapped, even local recovery didn't help. Planner may "
            f"report goal as unreachable from here, which will force another "
            f"push to clear the way.\n"
        )
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

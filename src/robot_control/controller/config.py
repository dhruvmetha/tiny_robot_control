"""Controller configuration dataclasses."""

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import yaml


@dataclass
class NavigationConfig:
    """Configuration for NavigationController."""

    max_speed: float = 0.3
    pre_rotation_skip_angle: float = 45.0  # degrees
    robot_geometry_scale: float = 1.0  # RVG robot inflation (1.0 = half robot size)
    goal_tolerance_ratio: float = 0.5  # goal_tolerance = ratio * car_size

    # Rotation parameters
    rotation_tolerance_deg: float = 1.0
    rotation_stable_time: float = 0.5  # seconds
    rotation_speed_max: float = 0.25
    rotation_speed_min: float = 0.15
    wheel_deadband: float = 0.05


@dataclass
class PushConfig:
    """Configuration for PushController."""

    max_speed: float = 0.3
    standoff_multiplier: float = 0.5  # standoff = multiplier * robot_size
    wheel_deadband: float = 0.05
    lookahead_ratio: float = 0.3
    push_steps: int = 300  # control ticks per push (~10 seconds at 30Hz)
    dynamic_direction: bool = True  # update push direction every step vs fix at start

    # Approach phase parameters
    approach_skip_distance: float = 3.0  # cm
    approach_skip_angle: float = 30.0  # degrees

    # Edge point generation (matches namo_cpp)
    # 1 = legacy (4 edges), 3 = standard (12 edges), 15 = high-res (60 edges)
    points_per_face: int = 1

    # Advance phase parameters (close gap before pushing)
    advance_speed: float = 0.15  # Forward speed
    advance_steps: int = 30  # Ticks to advance (~1 second at 30Hz)

    # Retreat phase parameters (back up after push to clear object)
    retreat_speed: float = 0.15  # Backup speed (applied as negative)
    retreat_steps: int = 30  # Ticks to back up (~1 second at 30Hz)


@dataclass
class ControllerConfigs:
    """Container for all controller configs."""

    navigation: NavigationConfig
    push: PushConfig

    @classmethod
    def from_yaml(cls, yaml_path: Path) -> "ControllerConfigs":
        """Load configs from YAML file."""
        with open(yaml_path) as f:
            data = yaml.safe_load(f)

        nav_data = data.get("navigation", {})
        push_data = data.get("push", {})

        return cls(
            navigation=NavigationConfig(
                max_speed=nav_data.get("max_speed", 0.3),
                pre_rotation_skip_angle=nav_data.get("pre_rotation_skip_angle", 45.0),
                robot_geometry_scale=nav_data.get("robot_geometry_scale", 1.0),
                goal_tolerance_ratio=nav_data.get("goal_tolerance_ratio", 0.5),
                rotation_tolerance_deg=nav_data.get("rotation_tolerance_deg", 1.0),
                rotation_stable_time=nav_data.get("rotation_stable_time", 0.5),
                rotation_speed_max=nav_data.get("rotation_speed_max", 0.25),
                rotation_speed_min=nav_data.get("rotation_speed_min", 0.15),
                wheel_deadband=nav_data.get("wheel_deadband", 0.05),
            ),
            push=PushConfig(
                max_speed=push_data.get("max_speed", 0.3),
                standoff_multiplier=push_data.get("standoff_multiplier", 0.5),
                wheel_deadband=push_data.get("wheel_deadband", 0.05),
                lookahead_ratio=push_data.get("lookahead_ratio", 0.3),
                push_steps=push_data.get("push_steps", 300),
                dynamic_direction=push_data.get("dynamic_direction", True),
                approach_skip_distance=push_data.get("approach_skip_distance", 3.0),
                approach_skip_angle=push_data.get("approach_skip_angle", 30.0),
                points_per_face=push_data.get("points_per_face", 1),
                advance_speed=push_data.get("advance_speed", 0.15),
                advance_steps=push_data.get("advance_steps", 30),
                retreat_speed=push_data.get("retreat_speed", 0.15),
                retreat_steps=push_data.get("retreat_steps", 30),
            ),
        )

    @classmethod
    def defaults(cls) -> "ControllerConfigs":
        """Create with default values."""
        return cls(
            navigation=NavigationConfig(),
            push=PushConfig(),
        )


# Default config path
DEFAULT_CONTROLLER_YAML = Path(__file__).parent.parent.parent.parent / "config" / "controller.yaml"


def load_controller_configs(yaml_path: Optional[Path] = None) -> ControllerConfigs:
    """Load controller configs from YAML, falling back to defaults."""
    path = yaml_path or DEFAULT_CONTROLLER_YAML
    if path.exists():
        return ControllerConfigs.from_yaml(path)
    return ControllerConfigs.defaults()

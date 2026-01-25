"""Core types for robot control."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict


@dataclass
class ObjectPose:
    """Pose and size of an object in the workspace."""

    x: float
    y: float
    theta: float
    width: float = 0.0   # Object width (0 = unknown, use default)
    height: float = 0.0  # Object height (0 = unknown, use default)


@dataclass
class Observation:
    """Observation from the environment."""

    robot_x: float
    robot_y: float
    robot_theta: float
    objects: Dict[str, ObjectPose]
    timestamp: float


@dataclass
class Action:
    """Differential drive action with normalized wheel speeds."""

    left_speed: float  # [-1, 1]
    right_speed: float  # [-1, 1]

    @classmethod
    def stop(cls) -> Action:
        """Create a stop action (zero speeds)."""
        return cls(left_speed=0.0, right_speed=0.0)


@dataclass
class Subgoal:
    """Base class for subgoals. Concrete subgoals define specific targets."""

    pass


@dataclass
class WorkspaceConfig:
    """
    Configuration for workspace and robot geometry.

    This is a generic config that can be used for both simulation and real
    camera setups. It contains fields needed for rendering and control.
    """

    # Workspace dimensions (in workspace units, e.g., cm)
    width: float
    height: float

    # Robot dimensions
    car_width: float
    car_height: float

    # Robot center offset from bottom-left corner
    offset_w: float  # Offset from left edge
    offset_h: float  # Offset from bottom edge

    # Robot dynamics (needed for control)
    wheel_base: float = 30.0  # Distance between wheels (default matches typical robot)

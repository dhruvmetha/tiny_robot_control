"""Utility classes for robot_control."""

from robot_control.utils.camera_recorder import CameraRecorder
from robot_control.utils.wavefront import WavefrontPlanner, WavefrontConfig
from robot_control.utils.robot_geometry import (
    # Canonical names (prefer these in new code):
    effective_robot_size_cm,
    effective_robot_radius_cm,
    effective_robot_radius_m_from_cm,
    scaled_half_extents_m_from_full_extents_cm,
    # Backwards-compatible aliases (deprecated; do not use in new code):
    robot_diagonal_cm,
    rotation_safe_radius_cm,
    rotation_safe_radius_m_from_cm,
)
from robot_control.utils.xml_generator import (
    NAMOXMLGenerator,
    ObjectSpec,
    RobotSpec,
    GoalSpec,
    WorkspaceBounds,
)

__all__ = [
    "CameraRecorder",
    "NAMOXMLGenerator",
    "ObjectSpec",
    "RobotSpec",
    "GoalSpec",
    "WorkspaceBounds",
    "WavefrontPlanner",
    "WavefrontConfig",
    "effective_robot_size_cm",
    "effective_robot_radius_cm",
    "effective_robot_radius_m_from_cm",
    "scaled_half_extents_m_from_full_extents_cm",
    # Deprecated aliases (kept for back-compat):
    "robot_diagonal_cm",
    "rotation_safe_radius_cm",
    "rotation_safe_radius_m_from_cm",
]

"""Robot geometry helpers for consistent footprint semantics.

All robot width/height inputs in robot_control are full extents (cm).
"""

from __future__ import annotations

import math
from typing import Tuple


def robot_diagonal_cm(robot_width_cm: float, robot_height_cm: float) -> float:
    """Return full diagonal length (cm) from full extents."""
    return math.hypot(abs(float(robot_width_cm)), abs(float(robot_height_cm)))


def rotation_safe_radius_cm(robot_width_cm: float, robot_height_cm: float) -> float:
    """Return rotation-safe circular radius (cm) from full extents."""
    return 0.5 * robot_diagonal_cm(robot_width_cm, robot_height_cm)


def rotation_safe_radius_m_from_cm(robot_width_cm: float, robot_height_cm: float) -> float:
    """Return rotation-safe circular radius in meters from full extents (cm)."""
    return rotation_safe_radius_cm(robot_width_cm, robot_height_cm) / 100.0


def scaled_half_extents_m_from_full_extents_cm(
    robot_width_cm: float,
    robot_height_cm: float,
    scale_factor: float,
) -> Tuple[float, float]:
    """Convert full extents in cm to scaled half-extents in meters.

    This matches namo_cpp's `planning.robot_size` semantics:
    `[half_extent_x_m, half_extent_y_m]` in the simulation-scaled frame.
    """
    sx = (abs(float(robot_width_cm)) / 200.0) * float(scale_factor)
    sy = (abs(float(robot_height_cm)) / 200.0) * float(scale_factor)
    return sx, sy

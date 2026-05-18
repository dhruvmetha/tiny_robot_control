"""Robot geometry helpers — single source of truth for the robot's
characteristic scalar "size" and the derived wavefront inflation radius.

All robot width/height inputs in robot_control are full extents (cm).
Callers MUST NOT compute their own size from raw width/height — they
should call into the helpers here so a future formula change is a
one-line edit.

Functions exported:
    effective_robot_size_cm(w, h)          -> scalar "size" in cm
    effective_robot_radius_cm(w, h)        -> inflation radius in cm
    effective_robot_radius_m_from_cm(w, h) -> inflation radius in meters
    scaled_half_extents_m_from_full_extents_cm(w, h, scale_factor)
                                           -> (half_x_m, half_y_m) for
                                              namo_cpp planning.robot_size

Compatibility aliases (do NOT use in new code; kept so existing imports
still work after the merge):
    robot_diagonal_cm           -> effective_robot_size_cm
    rotation_safe_radius_cm     -> effective_robot_radius_cm
    rotation_safe_radius_m_from_cm -> effective_robot_radius_m_from_cm
"""

from __future__ import annotations

from typing import Tuple


# ───────────────────────────────────────────────────────────────────────
# THE single formula. Change here, all of robot_control picks it up.
# ───────────────────────────────────────────────────────────────────────
def effective_robot_size_cm(
    robot_width_cm: float, robot_height_cm: float
) -> float:
    """Effective scalar "size" of the robot footprint, in cm.

    This is the canonical scalar length used throughout robot_control
    wherever code needs a single number for the robot's physical extent.
    Direct consumers (via aliases below) include:
      - wavefront inflation radius (half of this value)
      - push controller standoff distance
      - pure-pursuit lookahead distance
      - path duplicate-point filter min distance

    The formula is `max(w, h)` (axis-aligned, the inscribed-square
    convention). The diagonal `sqrt(w² + h²)` was attempted during the
    2026-05-18 real-robot merge but was ~√2× larger than physical reality
    and made push approach poses sit too far from target objects (with
    obstacles' inflated envelopes overlapping past usable corridors).
    Reverted to the pre-merge local convention.

    To change the formula, edit this function. All call sites — including
    the C++ side via `compute_rotation_safe_robot_radius_m` in
    `namo_cpp/include/wavefront/goal_tolerance_utils.hpp` — must be kept
    in sync. They currently use `max(hx, hy)` so both sides agree.
    """
    return max(abs(float(robot_width_cm)), abs(float(robot_height_cm)))


def effective_robot_radius_cm(
    robot_width_cm: float, robot_height_cm: float
) -> float:
    """Effective robot circular radius (cm) for wavefront inflation.

    Returns `effective_robot_size_cm(...) / 2`. Use this when you need
    the inflation RADIUS (wavefront grids, obstacle expansion).
    """
    return effective_robot_size_cm(robot_width_cm, robot_height_cm) / 2.0


def effective_robot_radius_m_from_cm(
    robot_width_cm: float, robot_height_cm: float
) -> float:
    """Same as `effective_robot_radius_cm` but in meters."""
    return effective_robot_radius_cm(robot_width_cm, robot_height_cm) / 100.0


def scaled_half_extents_m_from_full_extents_cm(
    robot_width_cm: float,
    robot_height_cm: float,
    scale_factor: float,
) -> Tuple[float, float]:
    """Convert full extents in cm to scaled half-extents in meters.

    This matches namo_cpp's `planning.robot_size` semantics:
    `[half_extent_x_m, half_extent_y_m]` in the simulation-scaled frame.
    Used by the bridge to inject runtime robot dimensions into the C++
    planner's YAML at plan-time.
    """
    sx = (abs(float(robot_width_cm)) / 200.0) * float(scale_factor)
    sy = (abs(float(robot_height_cm)) / 200.0) * float(scale_factor)
    return sx, sy


# ───────────────────────────────────────────────────────────────────────
# Backwards-compatibility aliases.
# Do NOT use these in new code. They exist so external imports of the
# old names continue to work after the merge.
# ───────────────────────────────────────────────────────────────────────
robot_diagonal_cm = effective_robot_size_cm
rotation_safe_radius_cm = effective_robot_radius_cm
rotation_safe_radius_m_from_cm = effective_robot_radius_m_from_cm

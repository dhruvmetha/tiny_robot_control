"""Edge point generation for push operations.

This module generates robot approach points (edge points) around rectangular
objects for push operations.

Dimension convention (matching GUI/micromvp):
    - depth = X dimension (along marker heading when theta=0)
    - width = Y dimension (perpendicular to marker heading)

For a 12cm x 4cm object defined as width=12, depth=4:
    - When theta=0: 4cm along X (depth), 12cm along Y (width)
    - Top/Bottom faces are at y = ±(width/2), span X from -depth/2 to +depth/2
    - Right/Left faces are at x = ±(depth/2), span Y from -width/2 to +width/2

Index layout for points_per_face=3 (12 total points):
    Index | Face       | Sample | Push Direction
    ------|------------|--------|---------------
    0     | Top (+Y)   | 0      | toward -Y
    1     | Bottom (-Y)| 0      | toward +Y
    2     | Top (+Y)   | 1      | toward -Y
    3     | Bottom (-Y)| 1      | toward +Y
    4     | Top (+Y)   | 2      | toward -Y
    5     | Bottom (-Y)| 2      | toward +Y
    6     | Right (+X) | 0      | toward -X
    7     | Left (-X)  | 0      | toward +X
    8     | Right (+X) | 1      | toward -X
    9     | Left (-X)  | 1      | toward +X
    10    | Right (+X) | 2      | toward -X
    11    | Left (-X)  | 2      | toward +X

Consecutive pairs (0-1, 2-3, ...) share a mid-point which defines the
push direction target.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

from robot_control.core.types import ObjectPose

Point = Tuple[float, float]


@dataclass
class EdgePoint:
    """A robot approach point for pushing an object."""

    edge_idx: int  # Global index (0 to 4n-1)
    face_idx: int  # Which face (0=Top/+Y, 1=Bottom/-Y, 2=Right/+X, 3=Left/-X)
    sample_idx: int  # Position along face (0 to n-1)
    position: Point  # World coordinates (x, y) in cm
    mid_point: Point  # Push direction target (world coords)
    approach_theta: float  # Robot orientation (degrees)


def sample_lin(a: float, b: float, n: int, i: int) -> float:
    """Linear interpolation matching namo_cpp.

    Args:
        a: Start value
        b: End value
        n: Number of samples
        i: Sample index (0 to n-1)

    Returns:
        Interpolated value
    """
    if n <= 1:
        return (a + b) * 0.5
    return a + (b - a) * (float(i) / float(n - 1))


def generate_edge_points(
    obj: ObjectPose,
    standoff: float,
    points_per_face: int = 3,
) -> List[EdgePoint]:
    """Generate edge points matching namo_cpp exactly.

    The algorithm follows namo_push_controller.cpp lines 66-139:
    1. Generate Top/Bottom pairs sampling along width (Y direction)
    2. Generate Right/Left pairs sampling along depth (X direction)
    3. Consecutive pairs share a mid-point for push direction

    Args:
        obj: Object pose with position, orientation, and dimensions
        standoff: Distance from object face to approach point (cm)
        points_per_face: Number of sample points per face (1, 3, or 15)

    Returns:
        List of EdgePoint objects, indexed 0 to 4*points_per_face - 1
    """
    # Half dimensions from full width/depth
    # Convention (matching GUI/micromvp):
    #   depth = X dimension (along marker heading when theta=0)
    #   width = Y dimension (perpendicular to heading)
    hd = obj.depth / 2.0 if obj.depth > 0 else 2.5  # half-depth = X extent
    hw = obj.width / 2.0 if obj.width > 0 else 2.5  # half-width = Y extent

    # Transform parameters
    theta_rad = math.radians(obj.theta)
    cos_t = math.cos(theta_rad)
    sin_t = math.sin(theta_rad)
    n = points_per_face

    # Generate local edge point coordinates
    # Top/Bottom pairs, then Right/Left pairs
    local_edges: List[Point] = []

    # Top/Bottom pairs: faces perpendicular to Y, sample along X (depth)
    # Top face at y = +hw, Bottom face at y = -hw
    for j in range(n):
        u = sample_lin(-hd, hd, n, j)  # X coordinate (along depth)
        local_edges.append((u, hw + standoff))   # Top (+Y)
        local_edges.append((u, -hw - standoff))  # Bottom (-Y)

    # Right/Left pairs: faces perpendicular to X, sample along Y (width)
    # Right face at x = +hd, Left face at x = -hd
    for k in range(n):
        v = sample_lin(-hw, hw, n, k)  # Y coordinate (along width)
        local_edges.append((hd + standoff, v))   # Right (+X)
        local_edges.append((-hd - standoff, v))  # Left (-X)

    # Compute mid-points using consecutive pairing (i XOR 1 = mate)
    local_mids: List[Point] = []
    for i in range(len(local_edges)):
        mate = i + 1 if i % 2 == 0 else i - 1
        mid = (
            0.5 * (local_edges[i][0] + local_edges[mate][0]),
            0.5 * (local_edges[i][1] + local_edges[mate][1]),
        )
        local_mids.append(mid)

    # Transform to world coordinates and create EdgePoints
    edge_points: List[EdgePoint] = []
    for idx in range(len(local_edges)):
        lx, ly = local_edges[idx]
        mx, my = local_mids[idx]

        # Transform edge point to world
        wx = obj.x + lx * cos_t - ly * sin_t
        wy = obj.y + lx * sin_t + ly * cos_t

        # Transform mid-point to world
        mwx = obj.x + mx * cos_t - my * sin_t
        mwy = obj.y + mx * sin_t + my * cos_t

        # Determine face_idx and sample_idx
        if idx < 2 * n:
            # Top/Bottom region
            face_idx = 0 if idx % 2 == 0 else 1
            sample_idx = idx // 2
        else:
            # Right/Left region
            face_idx = 2 if (idx - 2 * n) % 2 == 0 else 3
            sample_idx = (idx - 2 * n) // 2

        # Compute approach orientation (robot faces toward mid_point)
        approach_theta = math.degrees(math.atan2(mwy - wy, mwx - wx))

        edge_points.append(
            EdgePoint(
                edge_idx=idx,
                face_idx=face_idx,
                sample_idx=sample_idx,
                position=(wx, wy),
                mid_point=(mwx, mwy),
                approach_theta=approach_theta,
            )
        )

    return edge_points


def get_edge_point(
    obj: ObjectPose,
    edge_idx: int,
    standoff: float,
    points_per_face: int = 3,
) -> EdgePoint:
    """Get a specific edge point by index.

    Args:
        obj: Object pose
        edge_idx: Edge index (0 to 4*points_per_face - 1)
        standoff: Distance from object face to approach point (cm)
        points_per_face: Number of sample points per face

    Returns:
        EdgePoint for the specified index

    Raises:
        ValueError: If edge_idx is out of range
    """
    max_idx = 4 * points_per_face - 1
    if edge_idx < 0 or edge_idx > max_idx:
        raise ValueError(f"edge_idx {edge_idx} out of range [0, {max_idx}]")

    all_points = generate_edge_points(obj, standoff, points_per_face)
    return all_points[edge_idx]


def get_face_name(face_idx: int) -> str:
    """Get human-readable face name.

    Args:
        face_idx: Face index (0-3)

    Returns:
        Face name string
    """
    names = {
        0: "Top (+Y)",
        1: "Bottom (-Y)",
        2: "Right (+X)",
        3: "Left (-X)",
    }
    return names.get(face_idx, f"Unknown ({face_idx})")

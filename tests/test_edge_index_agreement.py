"""Edge index N means the same physical point in robot_control as in namo_cpp.

The runtime and the planner exchange pushes as ``(object_id, edge_idx, depth)``
and nothing else. The index is the whole contract. namo_cpp builds the points in
C++ (``namo_push_controller.cpp:176-189``) and robot_control mirrors that in
Python (``controller/edge_points.py``). If the mirror drifts, every layer keeps
working and starts meaning something different: a blacklist blocks the wrong
contact, a dispatched push approaches the wrong face, and nothing raises.

The risk is not hypothetical. The two sides name the object's axes with opposite
words. C++ takes MuJoCo half-extents where ``obj_size[0]`` is the local X
half-extent; Python takes an ``ObjectPose`` whose ``depth`` is the *full* X
extent and whose ``width`` is the full Y extent. Getting that mapping backwards
is silent and, in this author's case, was made twice while debugging.

So this reimplements the C++ generator literally from its source and asserts the
Python mirror agrees point for point. Deliberately duplicated rather than
imported: a test that shares the code under test proves nothing.

To verify:
  cd robot_control && pytest tests/test_edge_index_agreement.py -v
"""

from __future__ import annotations

import math

import pytest

from robot_control.controller.edge_points import generate_edge_points
from robot_control.core.types import ObjectPose

# ─── Named constants ────────────────────────────────────────────────────

# Production primitive set: 4 faces x 15 samples, indices 0..59.
POINTS_PER_FACE = 15

# Standoff namo_push_controller.cpp:163 uses, rotation-safe radius plus
# push_offset_margin_ (namo_push_controller.hpp:116, 0.02 m). Centimetres here.
ROBOT_ROTATION_SAFE_RADIUS_CM = math.hypot(3.5, 3.5)
PUSH_OFFSET_MARGIN_CM = 2.0
STANDOFF_CM = ROBOT_ROTATION_SAFE_RADIUS_CM + PUSH_OFFSET_MARGIN_CM

# Half-extents of obj_4 as the captured scene XML states them, centimetres.
HALF_X_CM = 3.5
HALF_Y_CM = 7.5

# Millimetre agreement is far tighter than the 5 mm grid either side rasterises
# into, so a real drift cannot hide under it.
TOLERANCE_CM = 1e-6


def _sample_lin(a: float, b: float, n: int, i: int) -> float:
    """namo_push_controller.cpp:169-172, verbatim."""
    if n <= 1:
        return (a + b) * 0.5
    return a + (b - a) * (float(i) / float(n - 1))


def _cpp_edge_points(cx, cy, yaw_rad, w, d, offset, n):
    """namo_push_controller.cpp:176-197 reimplemented in Python.

    ``w`` and ``d`` are MuJoCo half-extents along local X and Y, matching
    ``obj_size[0]`` and ``obj_size[1]`` on the C++ side.
    """
    local = []
    for j in range(n):                      # Top/Bottom, sampled along X
        u = _sample_lin(-w, w, n, j)
        local.append((u, d + offset))       # Top
        local.append((u, -d - offset))      # Bottom
    for k in range(n):                      # Right/Left, sampled along Y
        v = _sample_lin(-d, d, n, k)
        local.append((w + offset, v))       # Right
        local.append((-w - offset, v))      # Left

    cos_t, sin_t = math.cos(yaw_rad), math.sin(yaw_rad)
    return [
        (cx + lx * cos_t - ly * sin_t, cy + lx * sin_t + ly * cos_t)
        for lx, ly in local
    ]


def _python_edge_points(cx, cy, yaw_deg, half_x, half_y, n):
    """The mirror, driven through its own public API.

    ObjectPose takes FULL extents: depth is the X extent, width is the Y one.
    """
    obj = ObjectPose(
        x=cx, y=cy, theta=yaw_deg,
        depth=2.0 * half_x, width=2.0 * half_y,
    )
    return [p.position for p in generate_edge_points(
        obj, standoff=STANDOFF_CM, points_per_face=n)]


# ─── Tests ──────────────────────────────────────────────────────────────


@pytest.mark.parametrize(
    "cx,cy,yaw_deg",
    [
        (24.5, 38.0, 0.0),        # centred, axis aligned
        (4.72, 21.99, -179.28),   # the wall-hugging scene from 2026-08-22
        (30.0, 20.0, 45.0),       # off-axis, so a swapped axis cannot cancel
        (12.0, 60.0, 91.0),       # near-quarter turn, the reset scene
        (8.9, 22.8, 21.0),        # arbitrary
    ],
)
def test_every_edge_index_lands_on_the_same_point_as_the_cpp_generator(cx, cy, yaw_deg):
    cpp = _cpp_edge_points(
        cx, cy, math.radians(yaw_deg),
        HALF_X_CM, HALF_Y_CM, STANDOFF_CM, POINTS_PER_FACE,
    )
    py = _python_edge_points(cx, cy, yaw_deg, HALF_X_CM, HALF_Y_CM, POINTS_PER_FACE)

    assert len(py) == len(cpp) == 4 * POINTS_PER_FACE
    for idx, ((ex, ey), (ax, ay)) in enumerate(zip(cpp, py)):
        assert ax == pytest.approx(ex, abs=TOLERANCE_CM), f"edge {idx} x"
        assert ay == pytest.approx(ey, abs=TOLERANCE_CM), f"edge {idx} y"


def test_a_swapped_object_axis_would_be_caught():
    """The mistake this test exists to catch has to actually fail it.

    Feeding the extents the wrong way round must move the points, or the
    parametrized test above proves nothing about the axis mapping.
    """
    right = _python_edge_points(24.5, 38.0, 30.0, HALF_X_CM, HALF_Y_CM, POINTS_PER_FACE)
    swapped = _python_edge_points(24.5, 38.0, 30.0, HALF_Y_CM, HALF_X_CM, POINTS_PER_FACE)

    assert right != swapped


def test_pair_mates_are_opposite_faces_across_the_object_centre_line():
    """i and i XOR 1 face each other, which is how push direction is derived.

    Their midpoint sits on the object's centre line, not at the centre point:
    the pair shares a sample position along the face and differs only in which
    side it approaches from. With yaw 0 that means top/bottom pairs (indices
    below 30) share the centre's y, and right/left pairs share its x. Pairing
    them any other way would aim every push somewhere else.
    """
    cx, cy = 24.5, 38.0
    pts = _python_edge_points(cx, cy, 0.0, HALF_X_CM, HALF_Y_CM, POINTS_PER_FACE)
    top_bottom_count = 2 * POINTS_PER_FACE

    for i in range(0, len(pts), 2):
        mx = 0.5 * (pts[i][0] + pts[i ^ 1][0])
        my = 0.5 * (pts[i][1] + pts[i ^ 1][1])
        if i < top_bottom_count:
            assert my == pytest.approx(cy, abs=1e-6), f"pair {i} not across y"
        else:
            assert mx == pytest.approx(cx, abs=1e-6), f"pair {i} not across x"

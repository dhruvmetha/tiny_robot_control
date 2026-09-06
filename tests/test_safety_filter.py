"""The safety filter's distance grid and entering rule.

Every number here is in cm on the real 40 x 60 table with the 7 x 7 cm robot
footprint from config/real.yaml.
"""

from __future__ import annotations

import math
import time

import numpy as np
import pytest

from robot_control.controller.safety_filter import (
    ARUCO_NOISE_BAND_CM,
    CELL_CM,
    HALF_CELL_DIAGONAL_CM,
    HARD_STOP_CM,
    WORKSPACE_EDGE,
    SafetyFilter,
    StaticBox,
    outline_points,
    point_to_box_cm,
)
from robot_control.core.types import ObjectPose, Observation, WorkspaceConfig

TABLE_W = 40.0
TABLE_H = 60.0
ROBOT_SIDE = 7.0
MARGIN = 1.5
# A 19.5 x 5.5 cm wall brick from config/objects.yaml.
BRICK_W = 19.5
BRICK_D = 5.5


def _workspace() -> WorkspaceConfig:
    return WorkspaceConfig(
        width=TABLE_W, height=TABLE_H,
        car_width=ROBOT_SIDE, car_height=ROBOT_SIDE,
        offset_w=ROBOT_SIDE / 2, offset_h=ROBOT_SIDE / 2,
    )


def _wall(name, x, y, theta=0.0, width=BRICK_W, depth=BRICK_D) -> ObjectPose:
    return ObjectPose(x=x, y=y, theta=theta, width=width, depth=depth, height=5.0, is_static=True)


def _obs(**objects) -> Observation:
    return Observation(robot_x=20, robot_y=30, robot_theta=0, objects=objects, timestamp=0.0)


def _filter(*walls: ObjectPose, margin=MARGIN, cell=CELL_CM) -> SafetyFilter:
    f = SafetyFilter(_workspace(), margin_cm=margin, cell_cm=cell)
    f.add_statics(_obs(**{f"wall_{i}": w for i, w in enumerate(walls)}))
    return f


def _exact(f: SafetyFilter, x, y) -> float:
    edge = min(x, y, TABLE_W - x, TABLE_H - y)
    return min([edge] + [point_to_box_cm(x, y, b) for b in f.statics])


# ---------------------------------------------------------------------------
# point_to_box_cm
# ---------------------------------------------------------------------------

BOX = StaticBox("b", cx=0.0, cy=0.0, theta_deg=0.0, half_x=2.0, half_y=1.0)


@pytest.mark.parametrize("px,py,expected", [
    (5.0, 0.0, 3.0),                    # beside the +x face
    (0.0, 4.0, 3.0),                    # beside the +y face
    (5.0, 4.0, math.hypot(3.0, 3.0)),   # off a corner
    (0.0, 0.0, -1.0),                   # centre: 1 cm to the nearest face
    (1.5, 0.0, -0.5),                   # inside, 0.5 from the +x face
])
def test_point_to_box_hand_values(px, py, expected):
    assert point_to_box_cm(px, py, BOX) == pytest.approx(expected)


@pytest.mark.parametrize("theta", [0.0, 30.0, 45.0, 90.0, 137.0])
def test_point_to_box_rotates_with_the_box(theta):
    box = StaticBox("b", cx=10.0, cy=10.0, theta_deg=theta, half_x=2.0, half_y=1.0)
    c, s = math.cos(math.radians(theta)), math.sin(math.radians(theta))
    # 3 cm out along the box's +x face normal, level with its centre.
    px, py = 10.0 + c * 5.0, 10.0 + s * 5.0
    assert point_to_box_cm(px, py, box) == pytest.approx(3.0)
    # 3 cm out along the +y face normal.
    px, py = 10.0 - s * 4.0, 10.0 + c * 4.0
    assert point_to_box_cm(px, py, box) == pytest.approx(3.0)


def test_point_to_box_accepts_arrays():
    d = point_to_box_cm(np.array([5.0, 0.0]), np.array([0.0, 0.0]), BOX)
    assert d.tolist() == pytest.approx([3.0, -1.0])


# ---------------------------------------------------------------------------
# Grid lookups
# ---------------------------------------------------------------------------

def test_lookup_is_never_optimistic_and_within_a_cell_diagonal():
    f = _filter(_wall("a", 20.0, 30.0, 25.0), _wall("b", 8.0, 50.0, 100.0))
    rng = np.random.default_rng(7)
    xs = rng.uniform(0.0, TABLE_W, 500)
    ys = rng.uniform(0.0, TABLE_H, 500)
    for x, y in zip(xs, ys):
        exact = _exact(f, x, y)
        got = f.lookup(x, y).distance_cm
        assert got <= exact + 1e-9
        assert got >= exact - 2 * HALF_CELL_DIAGONAL_CM - 1e-9


@pytest.mark.parametrize("x,y", [(-1.0, 30.0), (41.0, 30.0), (20.0, -2.0), (20.0, 61.0), (-3.0, -4.0)])
def test_lookup_outside_the_table_is_negative_and_named_edge(x, y):
    f = _filter(_wall("a", 20.0, 30.0))
    c = f.lookup(x, y)
    assert c.distance_cm < 0.0
    assert c.static_name == WORKSPACE_EDGE


def test_nearest_static_wins():
    f = _filter(_wall("a", 5.0, 30.0, 90.0), _wall("b", 35.0, 30.0, 90.0))
    assert f.lookup(9.0, 30.0).static_name == "wall_0"
    assert f.lookup(31.0, 30.0).static_name == "wall_1"
    assert f.lookup(20.0, 1.0).static_name == WORKSPACE_EDGE


def test_add_statics_records_each_wall_once_over_several_frames():
    f = SafetyFilter(_workspace(), margin_cm=MARGIN)
    assert f.add_statics(_obs(wall_a=_wall("a", 10.0, 10.0))) is True
    assert f.add_statics(_obs(wall_b=_wall("b", 30.0, 50.0))) is True
    assert f.add_statics(_obs(wall_a=_wall("a", 10.0, 10.0))) is False
    assert f.add_statics(_obs(movable=ObjectPose(x=20, y=30, theta=0, width=12, depth=4))) is False
    assert sorted(b.name for b in f.statics) == ["wall_a", "wall_b"]
    # Both walls shape the grid, whichever frame introduced them.
    assert f.lookup(10.0, 13.5).static_name == "wall_a"
    assert f.lookup(30.0, 46.5).static_name == "wall_b"


# ---------------------------------------------------------------------------
# Robot outline
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("theta", [0.0, 45.0, 137.0])
def test_outline_covers_the_perimeter_at_cell_pitch(theta):
    xs, ys = outline_points(20.0, 30.0, theta, ROBOT_SIDE, ROBOT_SIDE)
    assert len(xs) == 4 * math.ceil(ROBOT_SIDE / CELL_CM)
    c, s = math.cos(math.radians(theta)), math.sin(math.radians(theta))
    lx = c * (xs - 20.0) + s * (ys - 30.0)
    ly = -s * (xs - 20.0) + c * (ys - 30.0)
    on_edge = np.maximum(np.abs(lx), np.abs(ly))
    assert on_edge == pytest.approx(np.full_like(on_edge, ROBOT_SIDE / 2))
    step = np.hypot(np.diff(xs, append=xs[0]), np.diff(ys, append=ys[0]))
    assert step.max() <= CELL_CM + 1e-9


def test_a_brick_corner_touching_the_middle_of_a_face_is_seen():
    # 45-degree brick whose corner sits exactly on the robot's +x face middle.
    half = 2.0
    f = _filter(_wall("a", 20.0 + ROBOT_SIDE / 2 + half * math.sqrt(2), 30.0, 45.0,
                      width=2 * half, depth=2 * half))
    assert f.robot_clearance(20.0, 30.0, 0.0).distance_cm < CELL_CM / 2 + 1e-9


# ---------------------------------------------------------------------------
# Entering rule
# ---------------------------------------------------------------------------

def _wall_ahead(cell=CELL_CM) -> SafetyFilter:
    # Brick across the robot's path, its near face at x = 30.
    return _filter(_wall("a", 30.0 + BRICK_D / 2, 30.0, 0.0), cell=cell)


# The entering rule is about differences of a few mm, so these tests use a
# 1 mm grid to keep the lookup's rounding (up to one cell diagonal) out of
# the way. The production 5 mm grid only makes every stop earlier.


def test_a_robot_that_started_clear_stops_the_first_tick_inside_the_band():
    f = _wall_ahead()
    start = f.robot_clearance(10.0, 30.0, 0.0).distance_cm
    assert start > MARGIN
    x = 10.0
    fired_at = None
    for tick in range(200):
        x += 0.2
        v = f.check_entering(x, 30.0, 0.0, start)
        if v is not None:
            fired_at = tick
            assert v.static_name == "wall_0"
            assert v.distance_now_cm < MARGIN
            assert v.distance_at_start_cm == start
            break
        assert f.robot_clearance(x, 30.0, 0.0).distance_cm >= MARGIN
    assert fired_at is not None
    # Physical gap at the stop: robot face is at x + 3.5, wall face at 30.
    assert 30.0 - (x + ROBOT_SIDE / 2) >= MARGIN - CELL_CM - 0.2


def test_a_robot_that_started_inside_passes_until_it_closes_past_the_noise_band():
    f = _wall_ahead(cell=0.1)
    # Park the robot face 1.2 cm from the wall: inside the 1.5 cm band.
    x0 = 30.0 - ROBOT_SIDE / 2 - 1.2
    start = f.robot_clearance(x0, 30.0, 0.0).distance_cm
    assert HARD_STOP_CM + ARUCO_NOISE_BAND_CM + 0.1 < start < MARGIN
    assert f.check_entering(x0, 30.0, 0.0, start) is None
    # Sliding sideways along the wall keeps the clearance: passes.
    assert f.check_entering(x0, 34.0, 0.0, start) is None
    # Backing away: passes.
    assert f.check_entering(x0 - 3.0, 30.0, 0.0, start) is None
    # Creeping in by less than the noise band: passes.
    assert f.check_entering(x0 + ARUCO_NOISE_BAND_CM * 0.5, 30.0, 0.0, start) is None
    # Closing past the band while still above the hard floor: stops.
    v = f.check_entering(x0 + 0.7, 30.0, 0.0, start)
    assert v is not None
    assert v.distance_now_cm > HARD_STOP_CM


def test_the_hard_floor_fires_from_any_start():
    f = _wall_ahead(cell=0.1)
    x0 = 30.0 - ROBOT_SIDE / 2 - 0.5
    start = f.robot_clearance(x0, 30.0, 0.0).distance_cm
    assert HARD_STOP_CM < start < MARGIN
    # Only 0.25 cm closer, under the noise band, but under the hard floor.
    v = f.check_entering(x0 + 0.25, 30.0, 0.0, start)
    assert v is not None
    assert v.distance_now_cm < HARD_STOP_CM


def test_no_start_clearance_means_started_clear():
    f = _wall_ahead()
    x_inside = 30.0 - ROBOT_SIDE / 2 - 1.0
    assert f.check_entering(x_inside, 30.0, 0.0, None) is not None
    assert f.check_entering(10.0, 30.0, 0.0, None) is None


def test_the_table_edge_counts_as_a_wall():
    f = SafetyFilter(_workspace(), margin_cm=MARGIN)
    v = f.check_entering(ROBOT_SIDE / 2 + 0.5, 30.0, 0.0, 10.0)
    assert v is not None
    assert v.static_name == WORKSPACE_EDGE


# ---------------------------------------------------------------------------
# Cost
# ---------------------------------------------------------------------------

def test_build_and_lookup_are_cheap():
    walls = [_wall(str(i), 5.0 + 5.0 * i, 10.0 + 6.0 * i, 20.0 * i) for i in range(7)]
    t0 = time.perf_counter()
    f = _filter(*walls)
    build_s = time.perf_counter() - t0
    assert build_s < 1.0, build_s
    n = 200
    t0 = time.perf_counter()
    for _ in range(n):
        f.robot_clearance(20.0, 30.0, 17.0)
    per_call = (time.perf_counter() - t0) / n
    # Measured 40-60 us on the dev box; the bound leaves room for a busy CPU
    # while staying far under the 33 ms tick.
    assert per_call < 1e-3, per_call

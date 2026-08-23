"""Contact points the executor cannot reach never get offered to the planner.

The planner and the executor each decide reachability from their own occupancy
grid. Both inflate by rotation-safe robot radius plus the tier-1 margin, but
they rasterise it differently, so near a wall the answers differ by a cell or
two. Measured 2026-08-22: the planner offered edges 14 and 16 at x = 4.90 and
4.40 cm while the executor's boundary band starts at 5.45 cm. With a 4.95 cm
rotation-safe radius and the wall face at x = 0, both poses put a corner
through the wall, and the executor rejected them after the search had already
paid for them.

``unreachable_contact_points`` closes that by asking the executor's own
wavefront first. What the tests pin:

  it finds the real offenders     edges 14 and 16 on the measured scene
  it is not a blanket ban         an object in open floor keeps its contacts
  walls are not push targets      static objects contribute nothing
  the shape matches the consumer  (object_id, edge_idx), what
                                  NAMOPlanBridge.plan(failed_pushes=...) takes

No binding, no checkpoint, no hardware.

To verify:
  cd robot_control && pytest tests/test_reachability_filter.py -v
"""

from __future__ import annotations

import math

import pytest

from robot_control.core.types import ObjectPose, Observation
from robot_control.planner.reachability_filter import (
    EDGE_COUNT,
    POINTS_PER_FACE,
    robot_inflation_radius_cm,
    unreachable_contact_points,
)

# ─── Named constants ────────────────────────────────────────────────────

# The real arena, from config/real.yaml workspace.
WORKSPACE_W_CM = 49.0
WORKSPACE_H_CM = 77.5
# 7x7 car footprint, real.yaml robot.width_cm / height_cm.
ROBOT_W_CM = ROBOT_H_CM = 7.0

# obj_4 as the camera and the scene XML agree on it: half-extents 3.5 x 7.5 cm.
OBJ_X_EXTENT_CM = 7.0
OBJ_Y_EXTENT_CM = 15.0

# The scene where planner and executor disagreed, block hard against the left
# wall at x = 4.72 cm.
WALL_HUGGING_X_CM = 4.72
WALL_HUGGING_Y_CM = 21.99
WALL_HUGGING_YAW_DEG = math.degrees(-3.129)

# Contacts whose approach pose sits inside the boundary band, x < 4.0 cm for
# the 7x7 car. Measured on the wall-hugging scene: edge 18 lands at x = 3.87,
# edge 20 at 3.37. Edges 14 and 16, at 4.87 and 4.37, are OUTSIDE the band and
# must stay reachable; an earlier version of this filter inflated by the
# half-diagonal and wrongly rejected them.
EDGES_INSIDE_THE_BOUNDARY_BAND = (18, 20, 22)
EDGES_JUST_OUTSIDE_THE_BAND = (14, 16)


def _observation(objects, robot=(23.7, 7.4, 97.4)):
    return Observation(
        timestamp=0.0, robot_x=robot[0], robot_y=robot[1], robot_theta=robot[2],
        objects=objects,
    )


def _block(x, y, yaw=0.0, static=False):
    return ObjectPose(
        x=x, y=y, theta=yaw,
        width=OBJ_Y_EXTENT_CM, depth=OBJ_X_EXTENT_CM, is_static=static,
    )


def _blocked_edges(obs):
    return {edge for _name, edge in unreachable_contact_points(
        obs, WORKSPACE_W_CM, WORKSPACE_H_CM, ROBOT_W_CM, ROBOT_H_CM)}


# ─── Tests ──────────────────────────────────────────────────────────────


def test_the_inflation_radius_matches_the_planner_not_the_diagonal():
    """3.5 cm for the 7x7 car, max half-extent, NOT hypot(3.5,3.5)=4.95.

    namo_cpp settled on max(hx, hy) because the diagonal crowded out push
    approaches (wavefront/goal_tolerance_utils.hpp:12). Using the diagonal here
    silently rejected contacts the planner could reach.
    """
    assert robot_inflation_radius_cm(7.0, 7.0) == pytest.approx(3.5)
    assert robot_inflation_radius_cm(7.0, 7.0) != pytest.approx(math.hypot(3.5, 3.5))


@pytest.mark.parametrize("edge", EDGES_INSIDE_THE_BOUNDARY_BAND)
def test_contacts_inside_the_boundary_band_are_filtered(edge):
    """Approach poses the robot cannot occupy without overlapping the wall."""
    obs = _observation({"obj_4": _block(
        WALL_HUGGING_X_CM, WALL_HUGGING_Y_CM, WALL_HUGGING_YAW_DEG)})

    assert edge in _blocked_edges(obs)


@pytest.mark.parametrize("edge", EDGES_JUST_OUTSIDE_THE_BAND)
def test_contacts_just_outside_the_band_survive(edge):
    """The regression that made this filter reject usable pushes.

    Inflating by the half-diagonal, 4.95 instead of 3.5, moved the band from
    4.0 cm to 5.45 cm and swallowed these two. The planner could reach them
    the whole time.
    """
    obs = _observation({"obj_4": _block(
        WALL_HUGGING_X_CM, WALL_HUGGING_Y_CM, WALL_HUGGING_YAW_DEG)})

    assert edge not in _blocked_edges(obs)


def test_an_object_in_open_floor_keeps_most_of_its_contacts():
    """A filter that blocks everything would silently empty the search."""
    obs = _observation({"obj_4": _block(WORKSPACE_W_CM / 2, WORKSPACE_H_CM / 2)})

    blocked = _blocked_edges(obs)

    assert len(blocked) < EDGE_COUNT / 2, (
        f"{len(blocked)} of {EDGE_COUNT} blocked for an object in open floor"
    )


def test_a_wall_hugging_object_loses_more_contacts_than_an_open_one():
    """The whole premise: proximity to a wall is what removes contact points."""
    open_floor = _blocked_edges(_observation(
        {"obj_4": _block(WORKSPACE_W_CM / 2, WORKSPACE_H_CM / 2)}))
    against_wall = _blocked_edges(_observation(
        {"obj_4": _block(WALL_HUGGING_X_CM, WALL_HUGGING_Y_CM, WALL_HUGGING_YAW_DEG)}))

    assert len(against_wall) > len(open_floor)


def test_static_objects_are_never_offered_as_push_targets():
    """Walls are obstacles in the grid, not things to push."""
    obs = _observation({"wall_9": _block(19.9, 31.1, static=True)})

    assert unreachable_contact_points(
        obs, WORKSPACE_W_CM, WORKSPACE_H_CM, ROBOT_W_CM, ROBOT_H_CM) == set()


def test_the_result_plugs_into_the_bridge_blacklist():
    """NAMOPlanBridge.plan(failed_pushes=...) takes (object_id, edge_idx)."""
    obs = _observation({"obj_4": _block(
        WALL_HUGGING_X_CM, WALL_HUGGING_Y_CM, WALL_HUGGING_YAW_DEG)})

    blocked = unreachable_contact_points(
        obs, WORKSPACE_W_CM, WORKSPACE_H_CM, ROBOT_W_CM, ROBOT_H_CM)

    assert blocked, "the measured scene has unreachable contacts"
    for name, edge in blocked:
        assert name == "obj_4"
        assert 0 <= edge < EDGE_COUNT


def test_the_edge_count_matches_the_production_primitive_set():
    """60 edges, 4 faces x 15 points. Drift here means indices mean new things."""
    assert POINTS_PER_FACE == 15
    assert EDGE_COUNT == 60


def test_an_empty_scene_blocks_nothing():
    assert unreachable_contact_points(
        _observation({}), WORKSPACE_W_CM, WORKSPACE_H_CM, ROBOT_W_CM, ROBOT_H_CM) == set()


def test_a_failed_grid_raises_instead_of_reading_as_an_empty_result():
    """A filter that could not run must not be mistaken for one that found nothing.

    Both used to return ``set()``, so a broken wavefront looked exactly like a
    scene where every contact was fine, and the planner would run unfiltered
    without anyone noticing. Regression for that.
    """
    obs = _observation({"box": _block(24.0, 40.0)})

    class _NoGrid:
        def __init__(self, *_a, **_kw):
            pass

        def build_grid(self, *_a, **_kw):
            pass

        def apply_trapped_start_recovery(self, *_a, **_kw):
            pass

        def get_grid(self):
            return None

    import robot_control.planner.reachability_filter as rf

    real = rf.WavefrontPlanner
    rf.WavefrontPlanner = _NoGrid
    try:
        with pytest.raises(RuntimeError, match="grid unavailable"):
            unreachable_contact_points(
                obs,
                workspace_width_cm=WORKSPACE_W_CM,
                workspace_height_cm=WORKSPACE_H_CM,
                robot_width_cm=ROBOT_W_CM,
                robot_height_cm=ROBOT_H_CM,
            )
    finally:
        rf.WavefrontPlanner = real

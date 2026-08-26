"""A pushed object covering the goal point should not wedge the runtime or
burn a full NAMO push search when the region around the goal is open.

Two real trials on 2026-08-23 (real_trials/easy_002/trial2/run.log,
real_trials/hard_004/trial1/run.log) hit exactly this: the object the robot
had just pushed ended up sitting on the goal coordinate, the goal's region
was otherwise clear, and the planner either wedged trying to navigate
straight at the covered point or ran an unnecessary push to shove the
object off it.

``find_goal_retarget`` answers a narrower question: is there free,
robot-reachable floor within GOAL_RETARGET_CAP_CM of the goal point? What
these tests pin:

  a covered goal with open floor around it   retarget within the cap, and
                                              within the blocking object's
                                              own escape distance
  a free goal point                          no retarget -- nothing to do
  a goal buried under something bigger        no retarget within the cap --
  than the cap allows for                     keep planning pushes instead

No binding, no checkpoint, no hardware.

To verify:
  cd robot_control && pytest tests/test_goal_retarget.py -v
"""

from __future__ import annotations

import pytest

from robot_control.core.types import ObjectPose, Observation
from robot_control.planner.goal_retarget import find_goal_retarget
from robot_control.planner.namo_planner import GOAL_RETARGET_CAP_CM
from robot_control.planner.reachability_filter import robot_inflation_radius_cm
from robot_control.utils.wavefront_inflation_config import get_wavefront_inflation_config

# ─── Named constants ────────────────────────────────────────────────────

# A modest synthetic arena -- large enough to hold the scenarios below with
# room to spare, small enough to keep the 5mm grid (GRID_RESOLUTION_M in
# reachability_filter.py, matching the real navigator) fast in a test.
WORKSPACE_W_CM = 60.0
WORKSPACE_H_CM = 60.0

# 7x7 car footprint, matching real.yaml robot.width_cm / height_cm.
ROBOT_W_CM = ROBOT_H_CM = 7.0

GOAL_CM = (30.0, 30.0)

# The largest movable object's long side, per GOAL_RETARGET_CAP_CM's own
# provenance comment in namo_planner.py (15/2 = 7.5 of the cap's 11.5 total).
LARGEST_MOVABLE_LONG_SIDE_CM = 15.0

# Worst-case distance a single pushed object of that size can put between
# the goal point and free space: half its long side, plus the same robot
# inflation radius and tier1 margin the cap itself is built from. A correct
# retarget must never exceed this, or it isn't explainable by the object
# actually sitting on the goal.
_ROBOT_RADIUS_CM = robot_inflation_radius_cm(ROBOT_W_CM, ROBOT_H_CM)
_TIER1_MARGIN_CM = get_wavefront_inflation_config().tier1_base_inflation_margin_m * 100.0
OBJECT_ESCAPE_DISTANCE_CM = (
    LARGEST_MOVABLE_LONG_SIDE_CM / 2.0 + _ROBOT_RADIUS_CM + _TIER1_MARGIN_CM
)


def _observation(objects, robot=(10.0, 10.0)):
    return Observation(
        timestamp=0.0, robot_x=robot[0], robot_y=robot[1], robot_theta=0.0,
        objects=objects,
    )


def _block(x, y, depth, width, theta=0.0, static=False):
    """depth = full extent along local X, width = full extent along local Y."""
    return ObjectPose(x=x, y=y, theta=theta, width=width, depth=depth, is_static=static)


def _retarget(objects, robot=(10.0, 10.0)):
    return find_goal_retarget(
        _observation(objects, robot=robot),
        GOAL_CM,
        GOAL_RETARGET_CAP_CM,
        WORKSPACE_W_CM,
        WORKSPACE_H_CM,
        ROBOT_W_CM,
        ROBOT_H_CM,
    )


# ─── Tests ──────────────────────────────────────────────────────────────


def test_a_pushed_object_on_the_goal_point_gets_a_nearby_retarget():
    """The exact real-trial shape: one movable sitting on the goal point."""
    objects = {
        "obj_1": _block(
            GOAL_CM[0], GOAL_CM[1],
            depth=LARGEST_MOVABLE_LONG_SIDE_CM, width=7.0,
        ),
    }

    result = _retarget(objects)

    assert result is not None, "goal region is open; a retarget must be found"
    x, y, dist_cm = result
    assert 0.0 < dist_cm <= GOAL_RETARGET_CAP_CM
    assert dist_cm <= OBJECT_ESCAPE_DISTANCE_CM, (
        f"{dist_cm:.2f}cm retarget exceeds what a "
        f"{LARGEST_MOVABLE_LONG_SIDE_CM}cm object can explain"
    )
    # The retarget must actually be outside the object's own footprint,
    # not some other cell that happens to be close by coincidence.
    assert abs(x - GOAL_CM[0]) > LARGEST_MOVABLE_LONG_SIDE_CM / 2.0 or \
        abs(y - GOAL_CM[1]) > 3.5


def test_a_free_goal_point_needs_no_retarget():
    """Nothing on the goal cell -- the caller should just navigate straight."""
    objects = {"obj_1": _block(50.0, 50.0, depth=15.0, width=7.0)}

    assert _retarget(objects) is None


def test_an_empty_scene_needs_no_retarget():
    assert _retarget({}) is None


def test_a_goal_buried_under_something_bigger_than_the_cap_finds_nothing():
    """Beyond the cap, it isn't the pushed object anymore -- keep pushing.

    A slab whose half-extent plus inflation clears GOAL_RETARGET_CAP_CM in
    every direction leaves no free cell near the goal at all, which is the
    "walled off by statics" case: the caller must fall back to NAMO
    planning instead of retargeting into an obstacle.
    """
    objects = {
        "wall_9": _block(GOAL_CM[0], GOAL_CM[1], depth=40.0, width=40.0, static=True),
    }

    assert _retarget(objects, robot=(5.0, 5.0)) is None


def test_the_edges_of_the_slab_confirm_the_cap_is_actually_load_bearing():
    """Same slab, but small enough that its far edge is inside the cap.

    Regression guard for a retarget function that always returns None near
    a static obstacle regardless of size -- this must find something.
    """
    small_wall = _block(GOAL_CM[0], GOAL_CM[1], depth=8.0, width=8.0, static=True)

    result = _retarget({"wall_9": small_wall}, robot=(5.0, 5.0))

    assert result is not None
    _x, _y, dist_cm = result
    assert dist_cm <= GOAL_RETARGET_CAP_CM

"""The pure-navigation baseline, driven through the stack the NAMO arms use.

Two things are worth proving here. First, that a subgoal can carry its own
route: the baseline's variants differ in what a movable cell COSTS, and
`NavigationController`'s planner takes obstacles, which block, so the route has
to be planned by the baseline and handed over. Second, that handing one over
changes nothing for a subgoal that does not, because every NAMO subgoal is one
of those.

No camera, no MuJoCo, no robot. A fake path planner stands in for the
wavefront where only the plumbing is under test.

To verify:
  cd robot_control && python -m pytest tests/test_navigation_baseline_planner.py -v
"""

from __future__ import annotations

import pytest

from robot_control.controller.navigation import NavigationController, NavigationState
from robot_control.core.types import (
    NavigateSubgoal,
    Observation,
    ObjectPose,
    PushSubgoal,
    WorkspaceConfig,
)
from robot_control.executor import SubgoalExecutor
from robot_control.planner.navigation_baseline import (
    MOVABLE_COST_IGNORED,
    MOVABLE_COST_PENALISED,
)
from robot_control.planner.navigation_baseline_planner import (
    GOAL_TOLERANCE_CM,
    MODE_COSTS,
    NavigationBaselinePlanner,
)

# A table the size of the real one, so routes here have the same room the
# robot does.
WIDTH_CM = 49.0
HEIGHT_CM = 77.5
BOUNDS_M = (0.0, WIDTH_CM / 100.0, 0.0, HEIGHT_CM / 100.0)
ROBOT_W_CM = 7.0
ROBOT_H_CM = 7.0

START = (24.0, 12.0)
GOAL = (24.0, 65.0)


class _RecordingPlanner:
    """Reports whether anyone asked it to plan."""

    def __init__(self):
        self.calls = 0

    def plan(self, start, goal, obstacles):
        self.calls += 1
        return [start, goal]


def _workspace() -> WorkspaceConfig:
    return WorkspaceConfig(
        width=WIDTH_CM, height=HEIGHT_CM,
        car_width=ROBOT_W_CM, car_height=ROBOT_H_CM,
        offset_w=0.0, offset_h=0.0,
    )


def _obs(x=START[0], y=START[1], theta=90.0, objects=None) -> Observation:
    return Observation(
        timestamp=0.0, robot_x=x, robot_y=y, robot_theta=theta,
        objects=objects or {},
    )


def _block(x, y, w=10.0, d=10.0, static=False) -> ObjectPose:
    return ObjectPose(x=x, y=y, theta=0.0, width=w, depth=d, height=4.0,
                      is_static=static)


def _planner(mode="ignore", goal=GOAL) -> NavigationBaselinePlanner:
    return NavigationBaselinePlanner(
        goal_cm=goal, workspace_bounds_m=BOUNDS_M,
        robot_width_cm=ROBOT_W_CM, robot_height_cm=ROBOT_H_CM, mode=mode,
    )


# ─── A subgoal can carry its own route ──────────────────────────────────


def test_a_subgoal_with_a_route_does_not_ask_the_path_planner():
    """The whole point: the baseline's route is the one that gets driven."""
    path_planner = _RecordingPlanner()
    controller = NavigationController(_workspace(), path_planner)

    accepted = controller.navigate_to(
        GOAL[0], GOAL[1], None, START, [],
        path=[START, (24.0, 40.0), GOAL],
    )

    assert accepted
    assert path_planner.calls == 0
    assert controller.state is not NavigationState.FAILED


def test_a_subgoal_without_a_route_still_asks_the_path_planner():
    """Every NAMO subgoal is one of these, so this must not have changed."""
    path_planner = _RecordingPlanner()
    controller = NavigationController(_workspace(), path_planner)

    accepted = controller.navigate_to(GOAL[0], GOAL[1], None, START, [])

    assert accepted
    assert path_planner.calls == 1


def test_the_executor_forwards_the_route_it_was_given():
    """Without this the route is planned here and dropped one call later."""
    path_planner = _RecordingPlanner()
    controller = NavigationController(_workspace(), path_planner)
    executor = SubgoalExecutor(_workspace(), controller)

    executor.set_subgoal(
        NavigateSubgoal(x=GOAL[0], y=GOAL[1], path=[START, GOAL]), _obs()
    )

    assert path_planner.calls == 0


def test_a_route_the_controller_cannot_use_fails_rather_than_silently_planning():
    """A one-point route is not drivable. It must not fall back to planning,
    or a broken baseline route would quietly become the navigator's own and
    the run would measure the wrong thing."""
    path_planner = _RecordingPlanner()
    controller = NavigationController(_workspace(), path_planner)

    accepted = controller.navigate_to(GOAL[0], GOAL[1], None, START, [], path=[START])

    assert accepted is False
    assert path_planner.calls == 0
    assert controller.state is NavigationState.FAILED


# ─── What the baseline planner emits ────────────────────────────────────


def test_it_emits_one_navigate_subgoal_carrying_a_route():
    planner = _planner()

    subgoal = planner.plan(_obs())

    assert isinstance(subgoal, NavigateSubgoal)
    assert (subgoal.x, subgoal.y) == GOAL
    assert len(subgoal.path) >= 2
    assert subgoal.path[-1] == pytest.approx(GOAL, abs=1.0)


def test_it_never_emits_a_push():
    """The baseline exists to show what happens without pushing."""
    planner = _planner()
    obs = _obs(objects={"box": _block(24.0, 40.0)})

    for _ in range(5):
        subgoal = planner.plan(obs)
        assert not isinstance(subgoal, PushSubgoal)


def test_it_does_not_re_emit_the_drive_every_tick():
    """Re-emitting restarts the drive, and the robot never gets anywhere."""
    planner = _planner()
    obs = _obs()

    first = planner.plan(obs)
    second = planner.plan(obs)

    assert first is not None
    assert second is None


def test_it_does_not_replan_after_the_drive_ends():
    """One drive, one verdict. Replanning would turn a stall into a retry
    loop and the run would never produce the result it exists to produce."""
    planner = _planner()
    obs = _obs()
    planner.plan(obs)

    planner.notify_subgoal_done(obs, failed=True)

    assert planner.plan(obs) is None
    assert planner.is_complete(obs)


# ─── The two variants differ, and differ the right way ──────────────────


def test_ignore_drives_through_a_block_on_the_straight_line():
    planner = _planner(mode="ignore")

    planner.plan(_obs(objects={"box": _block(24.0, 40.0)}))

    assert planner.outcome.route_crosses.get("box", 0) > 0


def test_penalise_goes_around_the_same_block():
    planner = _planner(mode="penalise")

    planner.plan(_obs(objects={"box": _block(24.0, 40.0)}))

    assert planner.outcome.route_crosses == {}, (
        f"expected a detour, got {planner.outcome.route_crosses}"
    )


def test_a_wall_blocks_both_variants_and_the_run_ends():
    """Static geometry is impassable however cheap movables are, and a run
    with no route must finish rather than sit waiting for one."""
    wall = {"wall": _block(24.0, 40.0, w=WIDTH_CM * 2, d=4.0, static=True)}

    for mode in MODE_COSTS:
        planner = _planner(mode=mode)
        obs = _obs(objects=wall)

        assert planner.plan(obs) is None
        assert planner.is_complete(obs)
        assert planner.outcome.failure == "no_route_around_static_geometry"


def test_a_movable_spanning_the_table_is_still_driven_through():
    """Geometrically the same as the wall above. A block is something the
    robot may drive into, which is the distinction the baseline rests on."""
    planner = _planner(mode="penalise")

    subgoal = planner.plan(_obs(objects={"plug": _block(24.0, 40.0, w=WIDTH_CM * 2, d=4.0)}))

    assert subgoal is not None
    assert planner.outcome.route_crosses.get("plug", 0) > 0


def test_the_two_modes_price_a_movable_differently():
    assert MODE_COSTS["ignore"] == MOVABLE_COST_IGNORED
    assert MODE_COSTS["penalise"] == MOVABLE_COST_PENALISED
    assert MODE_COSTS["penalise"] > MODE_COSTS["ignore"]


def test_an_unknown_mode_is_refused():
    with pytest.raises(ValueError, match="Unknown mode"):
        _planner(mode="shove")


# ─── Arrival is decided by distance ─────────────────────────────────────


def test_arrival_is_decided_by_distance_not_by_the_controllers_verdict():
    """A controller reports done when it runs out of path. A robot stalled
    against a block does that too, and calling it an arrival would report
    the baseline solving exactly the scenes it cannot."""
    planner = _planner()
    planner.plan(_obs())
    stalled = _obs(x=GOAL[0], y=GOAL[1] - 20.0)

    planner.notify_subgoal_done(stalled, failed=False)

    assert planner.is_complete(stalled)
    assert planner.outcome.reached is False
    assert planner.outcome.failure == "stopped_short_of_goal"


def test_a_robot_that_arrives_is_recorded_as_reaching():
    planner = _planner()
    planner.plan(_obs())
    arrived = _obs(x=GOAL[0], y=GOAL[1] - GOAL_TOLERANCE_CM / 2.0)

    planner.notify_subgoal_done(arrived, failed=False)

    assert planner.is_complete(arrived)
    assert planner.outcome.reached is True
    assert planner.outcome.failure is None


def test_it_records_which_blocks_the_robot_shoved():
    """On the ignore variant this is the evidence the robot hit something."""
    planner = _planner()
    planner.plan(_obs(objects={"box": _block(24.0, 40.0)}))

    planner.notify_subgoal_done(
        _obs(objects={"box": _block(24.0, 47.0)}), failed=True
    )

    assert planner.outcome.objects_moved_cm["box"] == pytest.approx(7.0, abs=0.1)


def test_the_row_names_the_arm_it_belongs_to():
    """It sits in a table beside the NAMO arms, so it has to say which it is."""
    planner = _planner(mode="penalise")

    assert planner.outcome.as_row()["arm"] == "nav_baseline_penalise"

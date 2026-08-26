"""A failed navigate_to() must end the subgoal, not wedge the runtime.

Real logs, 2026-08-23 (real_trials/easy_002/trial2/run.log,
real_trials/hard_004/trial1/run.log): a NavigateSubgoal is dispatched, the
path planner returns no usable path ("planner returned 0 raw points, 0
after dedup"), and the robot then sits idle forever with no further log
lines.

Root cause: on a planning failure, navigate_to() set the state machine to
NavigationState.IDLE and returned False -- but NavigationController.is_done()
only ever checked for NavigationState.FINISHED. IDLE is not FINISHED, so
is_done() stayed False forever. runtime.py's autonomous loop only calls
executor.is_done() to decide whether to notify the planner and ask for the
next subgoal (runtime.py:1039), so a controller that never reports "done"
after a failure is a controller that never lets the loop replan. Action.stop()
was returned every tick after that -- functionally correct (the robot didn't
move), but nothing in the loop ever moved forward either.

The fix adds a FAILED terminal state, distinct from FINISHED, so is_done()
recognizes a failed navigate_to() as over and did_fail() can tell the runtime
which way it ended -- the same is_done()/did_fail() contract PushController
already implements correctly (controller/push.py:1085-1104).

These tests exercise NavigationController directly, with a fake path planner
standing in for RVG/wavefront, so no MuJoCo/RVG binding or hardware is
needed.

To verify:
  cd robot_control && pytest tests/test_navigation_failure_is_terminal.py -v
"""

from __future__ import annotations

from robot_control.controller.navigation import NavigationController, NavigationState
from robot_control.core.types import NavigateSubgoal, Observation, WorkspaceConfig


class _NoPathPlanner:
    """Stands in for RVG/wavefront: always fails to find a path."""

    def plan(self, start, goal, obstacles):
        return []


class _StraightLinePlanner:
    """Always succeeds with a trivial two-point path."""

    def plan(self, start, goal, obstacles):
        return [start, goal]


def _workspace_config() -> WorkspaceConfig:
    return WorkspaceConfig(
        width=70.0, height=55.0,
        car_width=7.0, car_height=7.0,
        offset_w=0.0, offset_h=0.0,
    )


def _obs(x=10.0, y=10.0, theta=0.0) -> Observation:
    return Observation(
        timestamp=0.0, robot_x=x, robot_y=y, robot_theta=theta, objects={},
    )


def _controller(planner) -> NavigationController:
    return NavigationController(_workspace_config(), planner)


# ─── Tests ──────────────────────────────────────────────────────────────


def test_a_failed_plan_is_terminal_immediately():
    """navigate_to() itself must land in a state is_done() recognizes."""
    controller = _controller(_NoPathPlanner())
    obs = _obs()

    accepted = controller.navigate_to(50.0, 40.0, None, (obs.robot_x, obs.robot_y), [])

    assert accepted is False
    assert controller.state == NavigationState.FAILED
    assert controller.is_done(obs, NavigateSubgoal(x=50.0, y=40.0)) is True
    assert controller.did_fail() is True


def test_a_failed_plan_never_reports_still_running():
    """The exact freeze this regresses: is_done() stuck False forever.

    Ticking step() after a failed navigate_to() must not resurrect a
    non-terminal state -- the robot stops, and stays reported as done.
    """
    controller = _controller(_NoPathPlanner())
    obs = _obs()
    subgoal = NavigateSubgoal(x=50.0, y=40.0)
    controller.navigate_to(50.0, 40.0, None, (obs.robot_x, obs.robot_y), [])

    for _ in range(5):
        action = controller.step(obs, subgoal)
        assert action.left_speed == 0.0
        assert action.right_speed == 0.0
        assert controller.is_done(obs, subgoal) is True
        assert controller.did_fail() is True


def test_a_successful_plan_is_not_reported_as_failed():
    """did_fail() must distinguish FINISHED from FAILED, not just "terminal"."""
    controller = _controller(_StraightLinePlanner())
    obs = _obs()

    accepted = controller.navigate_to(30.0, 30.0, None, (obs.robot_x, obs.robot_y), [])

    assert accepted is True
    assert controller.state != NavigationState.FAILED
    assert controller.did_fail() is False


def test_reset_clears_a_failed_state():
    """A controller must be reusable for the next subgoal after a failure."""
    controller = _controller(_NoPathPlanner())
    obs = _obs()
    controller.navigate_to(50.0, 40.0, None, (obs.robot_x, obs.robot_y), [])
    assert controller.state == NavigationState.FAILED

    controller.reset()

    assert controller.state == NavigationState.IDLE
    assert controller.did_fail() is False

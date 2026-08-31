"""A wedged robot must end its own drive, and back out before it does.

Nothing else stops it. FollowPathController finishes only when it consumes
its path, a stuck robot never advances the path index, and Runtime polls
is_complete only between subgoals. The pure-navigation baseline drove into a
block on 2026-08-31 and hung until a wall-clock kill.

No camera, no MuJoCo, no robot. Time is injected, so a 3 s window costs
microseconds to test.

To verify:
  cd robot_control && python -m pytest tests/test_navigation_stuck_retreat.py -v
"""

from __future__ import annotations

import pytest

from robot_control.controller.navigation import (
    NO_PROGRESS_HEADING_DEG,
    NO_PROGRESS_POSITION_CM,
    NO_PROGRESS_WINDOW_S,
    PHYSICAL_MOVE_COMMAND,
    NavigationController,
    NavigationState,
    _NoProgressWatchdog,
)
from robot_control.controller.retreat import (
    blind_reverse,
    find_retreat_target,
    reverse_toward,
)
from robot_control.core.types import Action, NavigateSubgoal, Observation, WorkspaceConfig

# Enough command to turn the motors, and not enough. The physical threshold on
# this car is 0.3, well above config's wheel_deadband of 0.05.
DRIVING = Action(left_speed=0.4, right_speed=0.4)
CRAWLING = Action(left_speed=0.1, right_speed=0.1)


def _obs(x=20.0, y=20.0, theta=0.0) -> Observation:
    return Observation(timestamp=0.0, robot_x=x, robot_y=y, robot_theta=theta, objects={})


# ─── The watchdog itself ────────────────────────────────────────────────


def test_a_robot_that_is_moving_is_never_called_stuck():
    w = _NoProgressWatchdog()
    for tick in range(200):
        # 1 cm/s, slow but real.
        cause = w.update(_obs(y=20.0 + tick * 0.033), DRIVING, tick * 0.033)
        assert cause is None


def test_a_robot_that_stops_dead_is_called_blocked():
    w = _NoProgressWatchdog()
    w.update(_obs(), DRIVING, 0.0)

    assert w.update(_obs(), DRIVING, NO_PROGRESS_WINDOW_S - 0.1) is None
    assert w.update(_obs(), DRIVING, NO_PROGRESS_WINDOW_S + 0.1) == w.BLOCKED


def test_a_robot_never_commanded_hard_enough_is_not_called_blocked():
    """config/controller.yaml says the deadband is 0.05, but this car does not
    move below 0.3. A watchdog trusting the config number would report an
    under-commanded robot as sitting against an obstacle."""
    w = _NoProgressWatchdog()
    w.update(_obs(), CRAWLING, 0.0)

    assert w.update(_obs(), CRAWLING, NO_PROGRESS_WINDOW_S + 0.1) == w.UNDER_COMMANDED
    assert CRAWLING.left_speed < PHYSICAL_MOVE_COMMAND


def test_turning_on_the_spot_counts_as_progress():
    """Pure pursuit turns the robot through a tight corner with barely any
    translation. Position alone would read that as wedged."""
    w = _NoProgressWatchdog()
    w.update(_obs(theta=0.0), DRIVING, 0.0)

    turned = w.update(_obs(theta=NO_PROGRESS_HEADING_DEG + 1.0), DRIVING, 1.0)
    assert turned is None
    assert w.update(_obs(theta=NO_PROGRESS_HEADING_DEG + 1.0), DRIVING,
                    NO_PROGRESS_WINDOW_S + 0.5) is None


def test_progress_restarts_the_window():
    """Otherwise a robot that crawls forward every two seconds trips it."""
    w = _NoProgressWatchdog()
    w.update(_obs(), DRIVING, 0.0)
    w.update(_obs(), DRIVING, NO_PROGRESS_WINDOW_S - 0.2)

    w.update(_obs(y=20.0 + NO_PROGRESS_POSITION_CM + 0.1), DRIVING, NO_PROGRESS_WINDOW_S)

    assert w.update(_obs(y=20.0 + NO_PROGRESS_POSITION_CM + 0.1), DRIVING,
                    NO_PROGRESS_WINDOW_S + 0.5) is None


def test_the_threshold_clears_the_measured_camera_noise():
    """ArUco position noise on a stationary robot measured about 0.1 cm peak
    to peak from camera_service on 2026-08-31."""
    measured_noise_cm = 0.1
    assert NO_PROGRESS_POSITION_CM >= 4 * measured_noise_cm


# ─── Backing out ────────────────────────────────────────────────────────


class _Wavefront:
    """Free only to the left of x=15 cm, so the way out is one direction."""

    def __init__(self, free_below_x_m: float):
        self._limit = free_below_x_m

    def is_free(self, x_m, y_m):
        return x_m < self._limit


def test_the_backward_cone_is_preferred():
    """Reaching a cell behind needs no rotation, which matters when the robot
    is wedged and may not be able to turn."""
    target, is_backward = find_retreat_target(
        _Wavefront(0.15), robot_xy=(20.0, 20.0), heading_deg=0.0,
        cone_half_angle_deg=45.0, min_dist_cm=5.0, max_dist_cm=15.0,
    )

    assert is_backward
    assert target[0] < 20.0


def test_navigation_never_gets_a_forward_target():
    """Its only way to reach one is to navigate there, and navigating out of a
    failed navigation is a retreat inside a retreat."""
    only_ahead = _Wavefront(999.0)

    target, is_backward = find_retreat_target(
        only_ahead, robot_xy=(20.0, 20.0), heading_deg=0.0,
        cone_half_angle_deg=45.0, min_dist_cm=5.0, max_dist_cm=15.0,
        allow_forward=False,
    )

    assert is_backward or target is None


def test_no_free_cell_anywhere_reports_none():
    walled_in = _Wavefront(-999.0)

    target, _ = find_retreat_target(
        walled_in, robot_xy=(20.0, 20.0), heading_deg=0.0,
        cone_half_angle_deg=45.0, min_dist_cm=5.0, max_dist_cm=15.0,
    )

    assert target is None


def test_reverse_and_blind_reverse_both_drive_backwards():
    reversing = reverse_toward(_obs(), (10.0, 20.0), speed=0.15, steer_gain=0.5)
    blind = blind_reverse(0.15)

    assert reversing.left_speed < 0 and reversing.right_speed < 0
    assert blind.left_speed == blind.right_speed < 0


def test_reversing_steers_toward_a_target_off_to_one_side():
    straight_back = reverse_toward(_obs(theta=0.0), (10.0, 20.0), 0.15, 0.5)
    off_to_a_side = reverse_toward(_obs(theta=0.0), (10.0, 26.0), 0.15, 0.5)

    assert straight_back.left_speed == pytest.approx(straight_back.right_speed)
    assert off_to_a_side.left_speed != pytest.approx(off_to_a_side.right_speed)


# ─── End to end through the controller ──────────────────────────────────


class _StraightLinePlanner:
    def plan(self, start, goal, obstacles):
        return [start, goal]


class _Clock:
    """Advances a fixed step per read, so the 3 s window costs no wall time."""

    def __init__(self, step_s: float = 0.05):
        self.t = 0.0
        self.step = step_s

    def __call__(self) -> float:
        self.t += self.step
        return self.t


def _controller(now_fn=None) -> NavigationController:
    config = WorkspaceConfig(width=49.0, height=77.5, car_width=7.0, car_height=7.0,
                             offset_w=0.0, offset_h=0.0)
    return NavigationController(config, _StraightLinePlanner(), now_fn=now_fn or _Clock())


def test_a_wedged_drive_retreats_and_then_reports_failed():
    """The whole point. Without this the runtime waits forever on a subgoal
    that can never finish."""
    controller = _controller()
    subgoal = NavigateSubgoal(x=20.0, y=70.0)
    controller.navigate_to(20.0, 70.0, None, (20.0, 20.0), [], path=[(20.0, 20.0), (20.0, 70.0)])
    controller._state = NavigationState.FOLLOWING

    stuck = _obs(x=20.0, y=20.0)
    for _ in range(400):
        controller.step(stuck, subgoal)
        if controller.state is NavigationState.FAILED:
            break

    assert controller.state is NavigationState.FAILED
    assert controller.is_done(stuck, subgoal)
    assert controller.did_fail()
    assert controller._failure_cause in ("blocked", "under_commanded")

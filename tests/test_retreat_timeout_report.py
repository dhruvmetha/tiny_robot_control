"""What the retreat reports when it runs out of steps.

The retreat sets PushState.FINISHED on two paths, reaching its target and
running out of steps. Both look identical from outside the controller, so a
push that left the robot sitting against the object it just moved is
indistinguishable from a clean one. Six of ten real closed-loop pushes in
closed_loop_sessions/ took the timeout path, every one of them burning the full
200-step budget, so this is the common case rather than an edge case.

The timeout line therefore has to carry where the robot actually stopped. That
number is what decides whether the tolerance is set below what the controller
can hold, or whether the robot is not reversing at all.
"""

from types import SimpleNamespace

import pytest

from robot_control.controller.push import PushController, PushState

# Matches config/controller.yaml: 100 ticks, doubled by the safety cap, and a
# 2 cm arrival tolerance against a 5-15 cm retreat.
RETREAT_STEPS = 100
RETREAT_TOLERANCE_CM = 2.0
TARGET = (10.0, 20.0)


def _retreating_controller(step_count):
    """A controller parked mid-retreat, with only what _handle_retreating reads."""
    controller = PushController.__new__(PushController)
    controller._push_config = SimpleNamespace(
        retreat_steps=RETREAT_STEPS,
        retreat_tolerance=RETREAT_TOLERANCE_CM,
        retreat_speed=0.15,
        retreat_steer_gain=0.3,
    )
    controller._retreat_target = TARGET
    controller._retreat_is_backward = True
    controller._retreat_step_count = step_count
    controller._nav_controller = None
    controller._state = PushState.RETREATING
    return controller


def _obs(x, y):
    return SimpleNamespace(robot_x=x, robot_y=y, robot_theta=0.0)


def test_the_timeout_line_says_how_far_off_the_robot_stopped(capsys):
    """A near-miss and a robot that never moved must not log the same thing."""
    controller = _retreating_controller(RETREAT_STEPS * 2 - 1)

    controller._handle_retreating(_obs(TARGET[0], TARGET[1] - 12.0))

    line = capsys.readouterr().out
    assert "Retreat timeout" in line
    assert "stopped 12.0 cm from target" in line
    assert "tolerance 2.0 cm" in line


def test_the_timeout_line_records_both_poses_for_the_run_log(capsys):
    controller = _retreating_controller(RETREAT_STEPS * 2 - 1)

    controller._handle_retreating(_obs(4.0, 20.0))

    line = capsys.readouterr().out
    assert "robot at (4.0, 20.0)" in line
    assert "target (10.0, 20.0)" in line


def test_timing_out_still_finishes_the_push():
    """The state machine must not change; only the report gains a number."""
    controller = _retreating_controller(RETREAT_STEPS * 2 - 1)

    action = controller._handle_retreating(_obs(TARGET[0], TARGET[1] - 12.0))

    assert controller._state is PushState.FINISHED
    assert (action.left_speed, action.right_speed) == (0.0, 0.0)


def test_reaching_the_target_finishes_without_a_timeout_line(capsys):
    controller = _retreating_controller(0)

    controller._handle_retreating(_obs(TARGET[0], TARGET[1] - 1.0))

    assert controller._state is PushState.FINISHED
    assert "Retreat timeout" not in capsys.readouterr().out


@pytest.mark.parametrize("steps_remaining", [1, 5, 50])
def test_a_retreat_with_budget_left_keeps_driving(steps_remaining, capsys):
    """Wheel speeds are asymmetric here: reversing applies steering correction."""
    controller = _retreating_controller(RETREAT_STEPS * 2 - 1 - steps_remaining)

    action = controller._handle_retreating(_obs(TARGET[0], TARGET[1] - 12.0))

    assert controller._state is PushState.RETREATING
    assert (action.left_speed, action.right_speed) != (0.0, 0.0)
    assert "Retreat timeout" not in capsys.readouterr().out

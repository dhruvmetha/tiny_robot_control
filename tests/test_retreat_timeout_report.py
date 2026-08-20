"""What the retreat reports when it ends.

The retreat sets PushState.FINISHED on four paths: reaching its target, running
out of steps, the blind reverse fallback finishing, and the forward navigator
reporting done. Both look identical from outside the controller, so a
push that left the robot sitting against the object it just moved is
indistinguishable from a clean one. Six of ten real closed-loop pushes under
closed_loop_sessions/ took the timeout path, every one burning the full
200-step budget, so this is the common case rather than an edge case.

Displacement is the number that separates them. Recovering it from those runs
by joining wheel_commands.jsonl, mid_obs.jsonl and run.log gave 3.0 cm for
every arriving retreat and 0.2-0.5 cm for every timing-out one, while the
commanded wheel speeds were a clean -0.150 on both. Neither figure appeared in
any log, so both exits now report it.
"""

from types import SimpleNamespace

import pytest

from robot_control.controller.push import PushController, PushState

# Matches config/controller.yaml: 100 ticks, doubled by the safety cap, and a
# 2 cm arrival tolerance against a 5-15 cm retreat.
RETREAT_STEPS = 100
RETREAT_TOLERANCE_CM = 2.0
TARGET = (10.0, 20.0)
# 5 cm below the target, matching retreat_min_dist: every real retreat in the
# session logs aimed at a target exactly that far away.
RETREAT_START = (10.0, 15.0)


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
    controller._retreat_start_pose = RETREAT_START
    controller._nav_controller = None
    controller._state = PushState.RETREATING
    return controller


def _obs(x, y):
    return SimpleNamespace(robot_x=x, robot_y=y, robot_theta=0.0)


def test_a_timeout_reports_how_far_the_robot_actually_travelled(capsys):
    """The real failures moved 0.2-0.5 cm; that is the number to watch."""
    controller = _retreating_controller(RETREAT_STEPS * 2 - 1)

    controller._handle_retreating(_obs(RETREAT_START[0], RETREAT_START[1] + 0.3))

    line = capsys.readouterr().out
    assert "Retreat TIMEOUT" in line
    assert "moved 0.3 cm" in line
    assert "stopped 4.7 cm from target" in line


def test_an_arriving_retreat_reports_the_same_two_numbers(capsys):
    """Both exits set FINISHED, so both have to say what happened."""
    controller = _retreating_controller(0)

    controller._handle_retreating(_obs(TARGET[0], TARGET[1] - 1.0))

    line = capsys.readouterr().out
    assert "Retreat reached target" in line
    assert "moved 4.0 cm" in line
    assert "stopped 1.0 cm from target" in line


def test_the_report_carries_the_speed_that_produced_it(capsys):
    """The commanded speed is the suspect, so it belongs beside the outcome."""
    controller = _retreating_controller(RETREAT_STEPS * 2 - 1)

    controller._handle_retreating(_obs(RETREAT_START[0], RETREAT_START[1] + 0.3))

    assert "commanded speed 0.15" in capsys.readouterr().out


def test_the_report_records_both_poses_for_the_run_log(capsys):
    controller = _retreating_controller(RETREAT_STEPS * 2 - 1)

    controller._handle_retreating(_obs(4.0, 20.0))

    line = capsys.readouterr().out
    assert "robot at (4.0, 20.0)" in line
    assert "target (10.0, 20.0)" in line


def test_a_retreat_with_no_recorded_start_still_reports(capsys):
    """Displacement is unknown after a restart mid-retreat, not a crash."""
    controller = _retreating_controller(RETREAT_STEPS * 2 - 1)
    controller._retreat_start_pose = None

    controller._handle_retreating(_obs(4.0, 20.0))

    assert "moved 0.0 cm" in capsys.readouterr().out


def test_timing_out_still_finishes_the_push():
    """The state machine must not change; only the report gains a number."""
    controller = _retreating_controller(RETREAT_STEPS * 2 - 1)

    action = controller._handle_retreating(_obs(TARGET[0], TARGET[1] - 12.0))

    assert controller._state is PushState.FINISHED
    assert (action.left_speed, action.right_speed) == (0.0, 0.0)


def test_reaching_the_target_is_not_reported_as_a_timeout(capsys):
    controller = _retreating_controller(0)

    controller._handle_retreating(_obs(TARGET[0], TARGET[1] - 1.0))

    assert controller._state is PushState.FINISHED
    assert "TIMEOUT" not in capsys.readouterr().out


@pytest.mark.parametrize("steps_remaining", [1, 5, 50])
def test_a_retreat_with_budget_left_keeps_driving(steps_remaining, capsys):
    """Wheel speeds are asymmetric here: reversing applies steering correction."""
    controller = _retreating_controller(RETREAT_STEPS * 2 - 1 - steps_remaining)

    action = controller._handle_retreating(_obs(TARGET[0], TARGET[1] - 12.0))

    assert controller._state is PushState.RETREATING
    assert (action.left_speed, action.right_speed) != (0.0, 0.0)
    assert "Retreat" not in capsys.readouterr().out


# --- the two exits that used to end silently ---------------------------------
#
# The blind fallback runs when the wavefront search finds no free cell to aim
# at, and the forward branch runs when the retreat target is ahead of the robot.
# Neither fired in the ten real runs under closed_loop_sessions/, all of which
# were BACKWARD with a target found, which is why they were easy to miss.

def test_the_blind_fallback_reports_when_it_runs_out_of_steps(capsys):
    """It has no target to measure against, so displacement is all there is."""
    controller = _retreating_controller(RETREAT_STEPS)
    controller._retreat_target = None

    controller._blind_retreat(_obs(RETREAT_START[0], RETREAT_START[1] + 0.4))

    line = capsys.readouterr().out
    assert "Retreat blind reverse ended" in line
    assert "moved 0.4 cm" in line
    assert "no retreat target" in line
    assert "commanded speed 0.15" in line


def test_the_blind_fallback_still_finishes_the_push(capsys):
    controller = _retreating_controller(RETREAT_STEPS)
    controller._retreat_target = None

    action = controller._blind_retreat(_obs(*RETREAT_START))

    assert controller._state is PushState.FINISHED
    assert (action.left_speed, action.right_speed) == (0.0, 0.0)


def test_the_blind_fallback_with_budget_left_says_nothing(capsys):
    controller = _retreating_controller(RETREAT_STEPS - 1)
    controller._retreat_target = None

    action = controller._blind_retreat(_obs(*RETREAT_START))

    assert controller._state is PushState.RETREATING
    assert action.left_speed < 0
    assert "Retreat" not in capsys.readouterr().out


def test_the_forward_navigator_reports_when_it_finishes(capsys):
    """A forward retreat hands driving to the navigator, which owns the ending."""
    controller = _retreating_controller(0)
    controller._retreat_is_backward = False
    controller._nav_controller = SimpleNamespace(
        is_done=lambda obs, subgoal: True,
        cancel=lambda: None,
        step=lambda obs, subgoal: None,
    )

    controller._handle_retreating(_obs(TARGET[0], TARGET[1] - 3.0))

    line = capsys.readouterr().out
    assert "Retreat navigation reported done" in line
    assert "moved 2.0 cm" in line
    assert "stopped 3.0 cm from target" in line
    assert controller._state is PushState.FINISHED


def test_a_retreat_that_never_recorded_a_start_reports_zero(capsys):
    """Displacement is unknown after a restart mid-retreat, not a crash."""
    controller = _retreating_controller(RETREAT_STEPS)
    controller._retreat_target = None
    controller._retreat_start_pose = None

    controller._blind_retreat(_obs(*RETREAT_START))

    assert "moved 0.0 cm" in capsys.readouterr().out

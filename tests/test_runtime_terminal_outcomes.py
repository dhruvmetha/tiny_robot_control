"""Terminal outcomes must distinguish NAMO success from planning failure."""

from __future__ import annotations

from types import SimpleNamespace

from robot_control.core.types import Observation
from robot_control.runtime import Runtime


class _Planner:
    def __init__(self, complete: bool, subgoal=None):
        self.complete = complete
        self.subgoal = subgoal
        self.plan_calls = 0

    def is_complete(self, _obs):
        return self.complete

    def plan(self, _obs):
        self.plan_calls += 1
        return self.subgoal

    def get_drawings(self):
        return []


class _Executor:
    def is_done(self, _obs):
        return True

    def has_active_subgoal(self):
        return False


class _Window:
    def __init__(self):
        self.close_calls = 0

    def close_window(self):
        self.close_calls += 1


class _RealEnv:
    def __init__(self):
        self.stop_calls = 0

    def stop_robot(self):
        self.stop_calls += 1


def _obs() -> Observation:
    return Observation(
        robot_x=10.0,
        robot_y=10.0,
        robot_theta=90.0,
        objects={},
        timestamp=0.0,
    )


def _runtime(planner: _Planner) -> Runtime:
    runtime = Runtime.__new__(Runtime)
    runtime._planner = planner
    runtime._executor = _Executor()
    runtime._config = SimpleNamespace(
        step_confirm=False,
        quit_on_complete=True,
        dry_run=False,
    )
    runtime._terminal_outcome = None
    runtime._terminal_announced = False
    runtime._check_robot_connectivity = lambda **_kwargs: None
    return runtime


def test_no_subgoal_while_unreachable_is_planning_failure():
    runtime = _runtime(_Planner(complete=False, subgoal=None))

    _action, _drawings, status = runtime._autonomous_step(_obs())

    assert status == "Planning Failed"
    assert runtime._terminal_outcome == (
        "failure",
        "planner returned no subgoal while goal was unreachable",
    )


def test_reachability_is_complete_without_requesting_a_subgoal():
    planner = _Planner(complete=True, subgoal=None)
    runtime = _runtime(planner)

    _action, _drawings, status = runtime._autonomous_step(_obs())

    assert status == "Plan Complete"
    assert planner.plan_calls == 0
    assert runtime._terminal_outcome == (
        "success",
        "goal reachable in simulation",
    )


def test_retained_terminal_outcome_drives_summary_classification():
    runtime = Runtime.__new__(Runtime)
    runtime._terminal_outcome = (
        "failure",
        "planner returned no subgoal while goal was unreachable",
    )

    assert runtime._determine_outcome() == runtime._terminal_outcome


def test_planning_failure_uses_the_normal_terminal_shutdown_path():
    runtime = Runtime.__new__(Runtime)
    runtime._config = SimpleNamespace(quit_on_complete=True, dry_run=False)
    runtime._running = True
    runtime._terminal_announced = False
    runtime._env = _RealEnv()
    runtime._window = _Window()

    should_break = runtime._handle_autonomous_terminal_status(
        "Planning Failed", is_real=True
    )

    assert should_break is True
    assert runtime._running is False
    assert runtime._terminal_announced is True
    assert runtime._env.stop_calls == 1
    assert runtime._window.close_calls == 1

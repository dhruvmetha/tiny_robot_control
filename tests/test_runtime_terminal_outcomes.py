"""Terminal outcomes must distinguish NAMO success from planning failure."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

from robot_control.core.types import Action, NavigateSubgoal, Observation
from robot_control.diagnostics import DiagnosticsRecorder
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
    def __init__(self):
        self.subgoal = None

    def is_done(self, _obs):
        return True

    def has_active_subgoal(self):
        return False

    def set_subgoal(self, subgoal, _obs):
        self.subgoal = subgoal

    def step(self, _obs):
        return Action.stop()

    def get_drawings(self):
        return []

    def get_status(self):
        return "navigating"


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
    runtime._subgoal_start_time = None
    runtime._check_robot_connectivity = lambda **_kwargs: None
    runtime._log_subgoal_dispatch = lambda *_args, **_kwargs: None
    runtime._record_subgoal_start = lambda *_args, **_kwargs: None
    runtime._video_subgoal_start = lambda *_args, **_kwargs: None
    return runtime


def test_no_subgoal_while_unreachable_is_planning_failure():
    runtime = _runtime(_Planner(complete=False, subgoal=None))

    _action, _drawings, status = runtime._autonomous_step(_obs())

    assert status == "Planning Failed"
    assert runtime._terminal_outcome == (
        "failure",
        "planner returned no subgoal while goal was unreachable",
    )


def test_reachable_but_not_arrived_dispatches_navigation():
    navigate = NavigateSubgoal(x=29.7, y=71.0, theta=None)
    planner = _Planner(complete=False, subgoal=navigate)
    runtime = _runtime(planner)

    _action, _drawings, status = runtime._autonomous_step(_obs())

    assert status == "Autonomous: navigating"
    assert runtime._executor.subgoal == navigate
    assert runtime._terminal_outcome is None


def test_arrival_records_goal_reached_success_without_planning_again():
    planner = _Planner(complete=True, subgoal=None)
    runtime = _runtime(planner)

    _action, _drawings, status = runtime._autonomous_step(_obs())

    assert status == "Plan Complete"
    assert planner.plan_calls == 0
    assert runtime._terminal_outcome == ("success", "goal reached")


def test_retained_terminal_outcome_drives_summary_classification():
    runtime = Runtime.__new__(Runtime)
    runtime._terminal_outcome = (
        "failure",
        "planner returned no subgoal while goal was unreachable",
    )

    assert runtime._determine_outcome() == runtime._terminal_outcome


def test_summary_includes_accumulated_planning_metrics(tmp_path):
    recorder = DiagnosticsRecorder(tmp_path / "trial1", verbose=False)
    recorder.record_plan({
        "planning_operation": "fresh_search",
        "planning_wall_time_ms": 12.5,
        "simulations_used": 5,
        "success": True,
    })
    recorder.record_plan({
        "planning_operation": "reuse_verification",
        "planning_wall_time_ms": 4.0,
        "simulations_used": 1,
        "success": False,
    })
    runtime = Runtime.__new__(Runtime)
    runtime._diag = recorder
    runtime._started_at_epoch = 10.0
    runtime._ended_at_epoch = 20.0
    runtime._planner = SimpleNamespace(
        _robot_goal_cm=(40.0, 40.0),
        _goal_strategy="nearest",
        _algorithm="full_namo",
    )
    runtime._world = SimpleNamespace(get=lambda: _obs())
    runtime._offline_total_sec = 0.0
    runtime._offline_since = None
    runtime._online_at_start = True
    runtime._config = SimpleNamespace(mode="real")

    runtime._write_summary(
        "failure",
        "test outcome",
        {"jpg": None, "json": None, "xml": None},
        {"jpg": None, "json": None, "xml": None},
    )

    summary = json.loads((tmp_path / "trial1" / "summary.json").read_text())
    assert summary["planning"] == recorder.planning
    assert summary["planning"] == {
        "wall_time_ms_total": 16.5,
        "wall_time_ms_fresh_search": 12.5,
        "wall_time_ms_reuse_verification": 4.0,
        "simulations_used_total": 6,
        "simulations_used_fresh_search": 5,
        "simulations_used_reuse_verification": 1,
    }
    recorder.close()


def test_fallback_arrival_summary_reports_goal_reached():
    runtime = Runtime.__new__(Runtime)
    runtime._terminal_outcome = None
    runtime._planner = _Planner(complete=True, subgoal=None)
    runtime._world = SimpleNamespace(get=_obs)

    assert runtime._determine_outcome() == ("success", "goal reached")


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


def test_real_exp_command_does_not_enable_held_targeting():
    readme = (
        Path(__file__).resolve().parents[1] / "real_exp" / "README.md"
    ).read_text()
    section = readme.split("## First trial command", 1)[1]
    command = section.split("```bash", 1)[1].split("```", 1)[0]

    assert "--hold-region-target" not in command
    assert "--exec-mode" not in command
    assert "--capture-scene" in command

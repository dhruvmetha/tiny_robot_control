"""Holding one region boundary across pushes.

Without this the planner re-derives the whole problem on every replan: new
region graph, new boundary, new blocking object, freshly sampled points. With
one push executed per replan, a setup push -- which by definition does not open
anything -- can leave the next plan aimed elsewhere, stranding the work.

The load-bearing assertion in these tests is a *call count*: across two pushes
the boundary is selected once, not twice.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from robot_control.core.types import Observation, PushSubgoal
from robot_control.planner import namo_planner as planner_mod
from robot_control.planner.region_target import (
    STATUS_EXHAUSTED,
    STATUS_OPENED,
    RegionOpeningTarget,
)

POINTS = [(0.30, 0.40), (0.31, 0.40)]
BLOCKERS = ["obj_4"]
GOAL_CM = (40.0, 40.0)


def _choice(**overrides):
    base = dict(
        found=True,
        target_points_m=list(POINTS),
        blocker_real_ids=list(BLOCKERS),
        region_path=["robot", "goal"],
        goal_already_reachable=False,
        failure_reason="",
    )
    base.update(overrides)
    return SimpleNamespace(**base)


def _plan(**overrides):
    base = dict(
        subgoals=[PushSubgoal("obj_4", 17, 1)],
        success=True,
        already_open=False,
        boundary_exhausted=False,
        failure_reason="",
        resolved_target="goal",
    )
    base.update(overrides)
    return SimpleNamespace(**base)


class _FakeBridge:
    def __init__(self, *args, **kwargs):
        self.select_results = [_choice()]
        self.solve_results = [_plan()]
        self.select_calls = 0
        self.solve_calls = []
        self.last_search_time_ms = 1.0
        self.last_algorithm_stats = {}
        self.last_xml_content = "<mujoco/>"

    def select_boundary(self, *args, **kwargs):
        self.select_calls += 1
        return self.select_results.pop(0) if self.select_results else _choice(found=False)

    def solve_boundary(self, obs, goal, target, **kwargs):
        self.solve_calls.append(target)
        return self.solve_results.pop(0) if self.solve_results else _plan(success=False)

    def analyze_reachability(self, **kwargs):
        return {"goal_reachable": False, "objects": {}}


def _obs():
    return Observation(
        robot_x=0.0, robot_y=0.0, robot_theta=0.0, objects={}, timestamp=0.0
    )


def _planner(monkeypatch, tmp_path=None, hold=True):
    monkeypatch.setattr(planner_mod, "NAMOPlanBridge", _FakeBridge)
    planner = planner_mod.NAMOPlanner(
        robot_goal_cm=GOAL_CM,
        namo_config_path="unused.yaml",
        execution_mode="mpc",
        hold_region_target=hold,
        active_target_path=str(tmp_path / "active.json") if tmp_path else None,
        verbose=False,
    )
    planner._is_goal_reachable = lambda obs: False
    return planner, planner._bridge


def test_first_plan_selects_a_boundary_and_queues_its_push(monkeypatch):
    planner, bridge = _planner(monkeypatch)

    subgoal = planner.plan(_obs())

    assert bridge.select_calls == 1
    assert subgoal == PushSubgoal("obj_4", 17, 1)
    assert planner._active_target.blocker_real_ids == ("obj_4",)


def test_the_boundary_is_selected_once_across_two_pushes(monkeypatch):
    """The regression this whole feature exists to prevent."""
    planner, bridge = _planner(monkeypatch)
    bridge.solve_results = [_plan(), _plan(subgoals=[PushSubgoal("obj_4", 22, 1)])]

    planner.plan(_obs())
    planner.notify_subgoal_done(_obs(), failed=False)
    planner.plan(_obs())

    assert bridge.select_calls == 1
    assert len(bridge.solve_calls) == 2


def test_the_same_frozen_points_are_used_on_every_solve(monkeypatch):
    planner, bridge = _planner(monkeypatch)
    bridge.solve_results = [_plan(), _plan()]

    planner.plan(_obs())
    planner.notify_subgoal_done(_obs(), failed=False)
    planner.plan(_obs())

    assert {t.target_samples_m for t in bridge.solve_calls} == {tuple(POINTS)}


def test_an_opened_boundary_is_released_and_the_next_selected(monkeypatch):
    planner, bridge = _planner(monkeypatch)
    bridge.select_results = [_choice(), _choice(blocker_real_ids=["obj_9"])]
    bridge.solve_results = [_plan(already_open=True), _plan()]

    planner.plan(_obs())

    assert bridge.select_calls == 2
    assert planner._active_target.blocker_real_ids == ("obj_9",)


def test_an_exhausted_boundary_is_released_and_yields_no_plan(monkeypatch):
    planner, bridge = _planner(monkeypatch)
    bridge.solve_results = [_plan(success=False, boundary_exhausted=True)]

    subgoal = planner.plan(_obs())

    assert subgoal is None
    assert planner._active_target is None


def test_a_failed_push_is_recorded_against_the_subproblem(monkeypatch):
    planner, bridge = _planner(monkeypatch)
    planner.plan(_obs())

    planner.notify_subgoal_done(_obs(), failed=True)

    assert planner._active_target.failed_pushes == (("obj_4", 17),)


def test_no_boundary_available_yields_no_plan(monkeypatch):
    planner, bridge = _planner(monkeypatch)
    bridge.select_results = [_choice(found=False, failure_reason="region_path_exhausted")]

    assert planner.plan(_obs()) is None


def test_repeated_already_open_boundaries_do_not_spin(monkeypatch):
    planner, bridge = _planner(monkeypatch)
    bridge.select_results = [_choice() for _ in range(20)]
    bridge.solve_results = [_plan(already_open=True) for _ in range(20)]

    assert planner.plan(_obs()) is None
    assert bridge.select_calls <= planner.MAX_BOUNDARY_ADVANCES_PER_PLAN + 1


# --- persistence -------------------------------------------------------------

def test_the_target_is_written_where_another_process_can_read_it(monkeypatch, tmp_path):
    planner, _bridge = _planner(monkeypatch, tmp_path=tmp_path)

    planner.plan(_obs())

    revived = RegionOpeningTarget.load(tmp_path / "active.json")
    assert revived is not None
    assert revived.target_samples_m == tuple(POINTS)


def test_a_persisted_target_is_reused_instead_of_reselected(monkeypatch, tmp_path):
    """A fresh process must continue the subproblem, not start a new one."""
    first, _ = _planner(monkeypatch, tmp_path=tmp_path)
    first.plan(_obs())

    second, bridge2 = _planner(monkeypatch, tmp_path=tmp_path)
    second.plan(_obs())

    assert bridge2.select_calls == 0
    assert bridge2.solve_calls[0].target_samples_m == tuple(POINTS)


def test_a_released_target_is_not_resumed(monkeypatch, tmp_path):
    planner, bridge = _planner(monkeypatch, tmp_path=tmp_path)
    bridge.solve_results = [_plan(success=False, boundary_exhausted=True)]
    planner.plan(_obs())

    fresh, bridge2 = _planner(monkeypatch, tmp_path=tmp_path)
    fresh.plan(_obs())

    assert bridge2.select_calls == 1


# --- the feature is opt-in ---------------------------------------------------

def test_target_mode_off_never_selects_a_boundary(monkeypatch):
    planner, bridge = _planner(monkeypatch, hold=False)
    planner._bridge.plan = lambda **kwargs: []

    planner.plan(_obs())

    assert bridge.select_calls == 0
    assert bridge.solve_calls == []

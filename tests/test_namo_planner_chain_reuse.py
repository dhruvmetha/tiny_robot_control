from __future__ import annotations

import importlib
import sys
import types
from pathlib import Path
from types import SimpleNamespace

import pytest


def _load_module(module_name: str):
    """Import a robot_control module, standing in for optional dependencies.

    The stand-in packages are installed only if the real import fails. They are
    bare namespace modules, so shadowing a real package would break any later
    test that imports a name from its __init__ -- which is collection-order
    dependent and therefore invisible until an unrelated test file is added.
    """
    try:
        return importlib.import_module(module_name)
    except ImportError:
        pass

    root = Path(__file__).resolve().parents[1] / "src" / "robot_control"
    robot_control_pkg = sys.modules.get("robot_control")
    if robot_control_pkg is None:
        robot_control_pkg = types.ModuleType("robot_control")
        robot_control_pkg.__path__ = [str(root)]
        sys.modules["robot_control"] = robot_control_pkg

    for subpkg in ("core", "planner", "camera", "utils"):
        full_name = f"robot_control.{subpkg}"
        if full_name not in sys.modules:
            pkg = types.ModuleType(full_name)
            pkg.__path__ = [str(root / subpkg)]
            sys.modules[full_name] = pkg

    utils_pkg = sys.modules["robot_control.utils"]
    if not hasattr(utils_pkg, "NAMOXMLGenerator"):
        xml_generator_mod = importlib.import_module("robot_control.utils.xml_generator")
        utils_pkg.NAMOXMLGenerator = xml_generator_mod.NAMOXMLGenerator

    return importlib.import_module(module_name)


types_mod = _load_module("robot_control.core.types")
ObjectPose = types_mod.ObjectPose
Observation = types_mod.Observation
PushSubgoal = types_mod.PushSubgoal
planner_mod = _load_module("robot_control.planner.namo_planner")


class _FakeBridge:
    def __init__(self, *args, **kwargs):
        self.plan_results = []
        self.verify_results = []
        self.plan_calls = []
        self.verify_calls = []
        self.last_search_time_ms = 0.0
        self.last_algorithm_stats = {}
        self.last_xml_content = "<mujoco/>"

    def plan(self, **kwargs):
        self.plan_calls.append(kwargs)
        self.last_search_time_ms = 10.0
        return self.plan_results.pop(0)

    def verify_chain(self, **kwargs):
        self.verify_calls.append(list(kwargs["chain"]))
        return self.verify_results.pop(0)

    def analyze_reachability(self, **kwargs):
        return {"goal_reachable": False, "objects": {}}


class _Recorder:
    def __init__(self):
        self.records = []

    def record_plan(self, payload):
        self.records.append(payload)


def _obs() -> Observation:
    return Observation(
        robot_x=10.0,
        robot_y=10.0,
        robot_theta=90.0,
        objects={
            "obj_1": ObjectPose(
                x=20.0,
                y=20.0,
                theta=0.0,
                width=10.0,
                depth=5.0,
                height=4.0,
                is_static=False,
            )
        },
        timestamp=0.0,
        goal_x=40.0,
        goal_y=40.0,
    )


def _success_result(chain):
    return SimpleNamespace(
        success=True,
        verified_subgoals=list(chain),
        sim_pushes_tried=len(chain),
        failed_step_index=None,
        failure_reason=None,
        goal_reachable_after=True,
        verification_time_ms=5.0,
        planner_scene_xml="<mujoco/>",
        object_mapping={"real_to_sim": {}, "sim_to_real": {}},
    )


def _failure_result(failed_step_index: int, reason: str = "failed"):
    return SimpleNamespace(
        success=False,
        verified_subgoals=[],
        sim_pushes_tried=failed_step_index + 1 if failed_step_index is not None and failed_step_index >= 0 else 0,
        failed_step_index=failed_step_index,
        failure_reason=reason,
        goal_reachable_after=False,
        verification_time_ms=5.0,
        planner_scene_xml="<mujoco/>",
        object_mapping={"real_to_sim": {}, "sim_to_real": {}},
    )


def _make_planner(monkeypatch) -> tuple[planner_mod.NAMOPlanner, _FakeBridge]:
    monkeypatch.setattr(planner_mod, "NAMOPlanBridge", _FakeBridge)
    planner = planner_mod.NAMOPlanner(
        robot_goal_cm=(40.0, 40.0),
        namo_config_path="unused.yaml",
        execution_mode="mpc",
        verbose=False,
    )
    planner._is_goal_reachable = lambda obs: False
    return planner, planner._bridge


def test_mpc_reuses_suffix_before_full_replan(monkeypatch):
    planner, bridge = _make_planner(monkeypatch)
    a1 = PushSubgoal("obj_1", 4, 3)
    a2 = PushSubgoal("obj_1", 8, 2)
    bridge.plan_results = [[a1, a2]]
    bridge.verify_results = [_success_result([a2])]

    first = planner.plan(_obs())
    assert first == a1

    planner.notify_subgoal_done(_obs(), failed=False)
    second = planner.plan(_obs())

    assert second == a2
    assert len(bridge.plan_calls) == 1
    assert bridge.verify_calls == [[a2]]


def test_mpc_retries_full_chain_when_suffix_fails_on_first_step(monkeypatch):
    planner, bridge = _make_planner(monkeypatch)
    a1 = PushSubgoal("obj_1", 4, 3)
    a2 = PushSubgoal("obj_1", 8, 2)
    bridge.plan_results = [[a1, a2]]
    bridge.verify_results = [
        _failure_result(0, "suffix_first_step_failed"),
        _success_result([a1, a2]),
    ]

    assert planner.plan(_obs()) == a1
    planner.notify_subgoal_done(_obs(), failed=False)
    retried = planner.plan(_obs())

    assert retried == a1
    assert len(bridge.plan_calls) == 1
    assert bridge.verify_calls == [[a2], [a1, a2]]


def test_mpc_retries_single_step_chain_before_fresh_replan(monkeypatch):
    planner, bridge = _make_planner(monkeypatch)
    a1 = PushSubgoal("obj_1", 4, 3)
    bridge.plan_results = [[a1]]
    bridge.verify_results = [_success_result([a1])]

    assert planner.plan(_obs()) == a1
    planner.notify_subgoal_done(_obs(), failed=False)
    retried = planner.plan(_obs())

    assert retried == a1
    assert len(bridge.plan_calls) == 1
    assert bridge.verify_calls == [[a1]]


def test_mpc_falls_back_to_full_planning_on_later_suffix_failure(monkeypatch):
    planner, bridge = _make_planner(monkeypatch)
    a1 = PushSubgoal("obj_1", 4, 3)
    a2 = PushSubgoal("obj_1", 8, 2)
    a3 = PushSubgoal("obj_1", 12, 2)
    b1 = PushSubgoal("obj_1", 20, 4)
    bridge.plan_results = [[a1, a2, a3], [b1]]
    bridge.verify_results = [_failure_result(1, "suffix_later_step_failed")]

    assert planner.plan(_obs()) == a1
    planner.notify_subgoal_done(_obs(), failed=False)
    replanned = planner.plan(_obs())

    assert replanned == b1
    assert len(bridge.plan_calls) == 2
    assert bridge.verify_calls == [[a2, a3]]


def test_fresh_search_measures_outer_wall_time(monkeypatch):
    planner, bridge = _make_planner(monkeypatch)
    recorder = _Recorder()
    planner.set_diagnostics_recorder(recorder)
    planner._unreachable_contact_points = lambda _obs: set()
    bridge.plan_results = [[PushSubgoal("obj_1", 4, 3)]]
    bridge.last_algorithm_stats = {"simulation_budget_used_total": 5}
    clock = iter([10.0, 10.0125])
    monkeypatch.setattr(
        planner_mod,
        "time",
        SimpleNamespace(perf_counter=lambda: next(clock)),
        raising=False,
    )

    planner._generate_plan(_obs())

    assert recorder.records[0]["planning_operation"] == "fresh_search"
    assert recorder.records[0]["planning_wall_time_ms"] == pytest.approx(12.5)
    assert recorder.records[0]["simulations_used"] == 5


def test_failed_suffix_records_its_simulation_and_outer_time(monkeypatch):
    planner, bridge = _make_planner(monkeypatch)
    recorder = _Recorder()
    planner.set_diagnostics_recorder(recorder)
    a1 = PushSubgoal("obj_1", 4, 3)
    a2 = PushSubgoal("obj_1", 8, 2)
    planner._pending_reuse_chain = [a1, a2]
    planner._pending_reuse_origin = "fresh_plan"
    bridge.verify_results = [SimpleNamespace(
        success=False,
        verified_subgoals=[],
        sim_pushes_tried=1,
        failed_step_index=None,
        failure_reason="goal_not_reachable_after_chain",
        goal_reachable_after=False,
        verification_time_ms=5.0,
        planner_scene_xml="<mujoco/>",
        object_mapping={"real_to_sim": {}, "sim_to_real": {}},
    )]
    clock = iter([20.0, 20.004])
    monkeypatch.setattr(
        planner_mod,
        "time",
        SimpleNamespace(perf_counter=lambda: next(clock)),
        raising=False,
    )

    reused = planner._try_pending_chain_reuse(_obs())

    assert reused is False
    assert recorder.records[0]["planning_operation"] == "reuse_verification"
    assert recorder.records[0]["planning_wall_time_ms"] == pytest.approx(4.0)
    assert recorder.records[0]["simulations_used"] == 1
    assert recorder.records[0]["search_time_ms"] == 5.0

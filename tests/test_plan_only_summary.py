"""Plan-only diagnostics own their terminal outcome when Runtime is bypassed."""

from __future__ import annotations

import json
import sys
from importlib import import_module
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from robot_control.diagnostics.recorder import DiagnosticsRecorder
from robot_control.planner import namo_bridge as namo_bridge_module
from robot_control.planner.namo_bridge import NAMOPlanBridge


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
run_namo = import_module("run_namo")


def _args(**overrides):
    values = {
        "sim": True,
        "config": None,
        "goal": (11.0, 67.6),
        "strategy": "primitive",
        "algorithm": "full_namo",
        "exec_mode": "greedy_dfs",
        "local_search": "best_first",
        "best_first_prior": "model",
        "shuffle_seed": 0,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def _solution(*, success=True, outcome="success", pushes=1):
    return {
        "success": success,
        "outcome": outcome,
        "goal_cm": [11.0, 67.6],
        "algorithm": "full_namo",
        "strategy": "primitive",
        "plan": [
            {"object_id": "obj_1", "edge_idx": 7, "push_steps": 2}
            for _ in range(pushes)
        ],
        "search_stats": {
            "search_time_ms": 125.0,
            "pushes_in_plan": pushes,
            "sim_pushes_tried": 3,
        },
        "planner_scene_xml": "planner_scene.xml",
    }


def test_plan_only_success_summary_survives_the_generic_fallback(tmp_path):
    recorder = DiagnosticsRecorder(tmp_path / "trial1", verbose=False)
    args = _args()

    run_namo._write_plan_only_summary(
        args,
        recorder,
        _solution(),
        algorithm_stats={"exec_mode": "greedy_dfs"},
    )
    before = json.loads((recorder.root / "summary.json").read_text())
    run_namo._write_run_summary(args, recorder)
    after = json.loads((recorder.root / "summary.json").read_text())
    recorder.close()

    assert before == after
    assert after["outcome"] == "success"
    assert after["outcome_reason"] == "success"
    assert after["mode"] == "sim_plan_only"
    assert after["planning"]["simulations_used_total"] == 3
    assert after["planning"]["wall_time_ms_total"] == pytest.approx(125.0)
    assert after["plan_only"]["pushes_returned"] == 1
    assert after["plan_only"]["artifacts"]["solution_yaml"] == "solution.yaml"
    assert recorder.totals["plan_calls"] == 1


def test_plan_only_failure_summary_preserves_the_planner_reason(tmp_path):
    recorder = DiagnosticsRecorder(tmp_path / "trial1", verbose=False)
    args = _args()

    run_namo._write_plan_only_summary(
        args,
        recorder,
        _solution(
            success=False,
            outcome="greedy_depth_exhausted",
            pushes=0,
        ),
        algorithm_stats={"failure_kind": "greedy_depth_exhausted"},
    )
    summary = json.loads((recorder.root / "summary.json").read_text())
    recorder.close()

    assert summary["outcome"] == "planning_failed"
    assert summary["outcome_reason"] == "greedy_depth_exhausted"
    assert summary["planning"]["simulations_used_total"] == 3
    assert summary["plan_only"]["pushes_returned"] == 0


def test_explicit_zero_push_success_is_not_reclassified_as_no_plan(tmp_path):
    payload = run_namo._emit_plan_only_solution_yaml(
        tmp_path,
        (11.0, 67.6),
        algorithm="full_namo",
        strategy="primitive",
        plan_subgoals=[],
        search_time_ms=12.5,
        sim_pushes_tried=0,
        success_override=True,
    )

    written = yaml.safe_load((tmp_path / "solution.yaml").read_text())
    assert payload == written
    assert written["success"] is True
    assert written["outcome"] == "success"
    assert written["plan"] == []


@pytest.mark.parametrize(
    ("success", "error_message", "algorithm_stats", "expected_reason"),
    [
        (True, "", {"exec_mode": "greedy_dfs"}, None),
        (
            False,
            "Greedy DFS committed-push depth exhausted",
            {"failure_kind": "greedy_depth_exhausted"},
            "Greedy DFS committed-push depth exhausted",
        ),
        (False, "", {"failure_kind": "region_path_exhausted"}, "region_path_exhausted"),
    ],
)
def test_bridge_preserves_the_explicit_service_outcome(
    monkeypatch,
    tmp_path,
    success,
    error_message,
    algorithm_stats,
    expected_reason,
):
    xml_path = tmp_path / "scene.xml"
    xml_path.write_text("<mujoco/>")
    result = SimpleNamespace(
        success=success,
        actions=[],
        search_time_ms=25.0,
        error_message=error_message,
        algorithm_stats=algorithm_stats,
    )

    class _Service:
        def preload_goal_model(self, *_args, **_kwargs):
            return None

        def plan_from_xml(self, **_kwargs):
            return result

    bridge = NAMOPlanBridge.__new__(NAMOPlanBridge)
    bridge.last_search_time_ms = 0.0
    bridge.last_sim_pushes_tried = None
    bridge.last_algorithm_stats = None
    bridge.last_plan_success = None
    bridge.last_plan_failure_reason = None
    bridge.last_xml_content = None
    bridge._verbose = False
    bridge._max_push_steps = None
    bridge._show_push_scores = False
    bridge._generated_config_path = None
    bridge._debug_xml_path = str(xml_path)
    bridge._object_mapping = SimpleNamespace(
        get_sim_name=lambda name: name,
        get_real_name=lambda name: name,
    )
    bridge._generate_xml = lambda _observation, _goal: "<mujoco/>"
    bridge._write_xml = lambda _content: str(xml_path)
    bridge._cm_to_sim = lambda x, y: (x / 100.0, y / 100.0)
    bridge._get_planning_service = lambda: _Service()
    bridge._starting_robot_pose_sim = lambda _observation: None
    bridge._convert_to_subgoals = lambda _actions: []
    monkeypatch.setattr(namo_bridge_module, "resolve_namo_cpp_dir", lambda _path: tmp_path)

    returned = bridge.plan(SimpleNamespace(), (11.0, 67.6), failed_pushes=set())

    assert returned == []
    assert bridge.last_plan_success is success
    assert bridge.last_plan_failure_reason == expected_reason

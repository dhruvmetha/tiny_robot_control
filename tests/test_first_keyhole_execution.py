"""Observed-state handoff and simulation accounting for first-keyhole trials."""

from types import SimpleNamespace

import pytest

from tests.test_namo_planner_chain_reuse import (
    _FakeBridge, _obs, _success_result, _failure_result, _Recorder, planner_mod,
    PushSubgoal,
)
from robot_control.planner import namo_bridge as bridge_mod
from robot_control.planner.search_config import LocalSearchConfig, check_search_reaches_planner
from namo.planners.utils import PushAttemptBudget


TARGET = {"kind": "region", "blocking_objects": ["obj_1"],
          "target_points": [[0.3, 0.5], [0.4, 0.5]], "min_reachable": 1}


class BudgetBridge(_FakeBridge):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.verify_kwargs = []

    def plan(self, **kwargs):
        result = super().plan(**kwargs)
        kwargs["push_budget"].consume_or_raise()
        self.last_algorithm_stats = {"keyhole_target": TARGET, "simulations_used": 1,
                                     "plan_outcome": "keyhole_ready"}
        return result

    def verify_chain(self, **kwargs):
        self.verify_kwargs.append(kwargs)
        result = super().verify_chain(**kwargs)
        for _ in range(result.sim_pushes_tried):
            kwargs["push_budget"].consume_or_raise()
        return result


def make_planner(monkeypatch, limit=900):
    monkeypatch.setattr(planner_mod, "NAMOPlanBridge", BudgetBridge)
    planner = planner_mod.NAMOPlanner(
        robot_goal_cm=(40.0, 40.0), namo_config_path="unused.yaml", verbose=False,
        local_search=LocalSearchConfig(local_search="best_first", best_first_prior="uniform",
                                      planning_horizon="first_keyhole", keyhole_simulation_budget=limit),
    )
    planner._is_goal_reachable = lambda obs: False
    planner._unreachable_contact_points = lambda obs: set()
    planner._select_goal_retarget = lambda obs: None
    return planner, planner._bridge


def closed_target():
    return SimpleNamespace(success=False, verified_subgoals=[], sim_pushes_tried=0,
                           failed_step_index=None, failure_reason="target_not_open_after_chain",
                           goal_reachable_after=False, target_open_after=False,
                           verification_time_ms=0, planner_scene_xml="", object_mapping={})


def test_local_suffix_runs_before_goal_is_reachable(monkeypatch):
    planner, bridge = make_planner(monkeypatch)
    first, second = PushSubgoal("obj_1", 2, 3), PushSubgoal("obj_1", 4, 2)
    bridge.plan_results = [[first, second]]
    result = _success_result([second])
    result.goal_reachable_after = False
    result.target_open_after = True
    bridge.verify_results = [result]
    assert planner.plan(_obs()) == first
    planner.notify_subgoal_done(_obs())
    assert planner.plan(_obs()) == second
    assert bridge.verify_kwargs[0]["keyhole_target"] == TARGET
    assert bridge.verify_kwargs[0]["push_budget"] is bridge.plan_calls[0]["push_budget"]
    assert planner._trial_push_budget.used == 2
    assert not planner.is_complete(_obs())


def test_observed_opening_discards_stale_chain_and_plans_next_keyhole(monkeypatch):
    planner, bridge = make_planner(monkeypatch)
    first, stale, next_push = [PushSubgoal("obj_1", edge, 2) for edge in (2, 4, 6)]
    bridge.plan_results = [[first, stale], [next_push]]
    opened = closed_target()
    opened.failure_reason = "target_already_open_before_chain"
    opened.target_open_after = True
    bridge.verify_results = [opened]
    assert planner.plan(_obs()) == first
    observed = _obs()
    observed.objects["obj_1"].x += 2
    planner.notify_subgoal_done(observed)
    assert planner.plan(observed) == next_push
    assert bridge.plan_calls[1]["observation"] is observed
    assert "full_namo_active_keyhole" not in bridge.plan_calls[1]
    assert bridge.plan_calls[0]["push_budget"] is bridge.plan_calls[1]["push_budget"]
    assert planner._trial_push_budget.used == 2


def test_failed_reuse_replans_original_keyhole_with_remaining_trial_budget(monkeypatch):
    planner, bridge = make_planner(monkeypatch)
    first, second, third, replacement = [PushSubgoal("obj_1", edge, 2) for edge in (2, 4, 6, 8)]
    bridge.plan_results = [[first, second, third], [replacement]]
    bridge.verify_results = [_failure_result(1), closed_target()]
    recorder = _Recorder()
    planner.set_diagnostics_recorder(recorder)
    assert planner.plan(_obs()) == first
    planner.notify_subgoal_done(_obs())
    assert planner.plan(_obs()) == replacement
    assert bridge.plan_calls[1]["full_namo_active_keyhole"] == TARGET
    assert planner._trial_push_budget.used == 4
    assert sum(row["simulations_used"] for row in recorder.records) == 4


def test_exhausted_trial_does_not_start_another_search(monkeypatch):
    planner, bridge = make_planner(monkeypatch, limit=1)
    bridge.plan_results = [[PushSubgoal("obj_1", 2, 2)]]
    assert planner.plan(_obs()) is not None
    planner.notify_subgoal_done(_obs(), failed=True)
    bridge.verify_results = [closed_target()]
    assert planner.plan(_obs()) is None
    assert len(bridge.plan_calls) == 1
    assert planner._planning_failed
    assert planner._trial_push_budget.used == 1
    planner.reset()
    assert planner._trial_push_budget is None
    assert planner._keyhole_target is None


@pytest.mark.parametrize("changes", [
    {"exec_mode": "greedy_policy"}, {"exec_mode": "reactive"},
    {"budget_scope": "keyhole", "keyhole_simulation_budget": 900},
    {"local_search": "region_bfs"},
])
def test_incompatible_first_keyhole_modes_are_rejected(changes):
    kwargs = dict(local_search="best_first", best_first_prior="uniform", planning_horizon="first_keyhole")
    with pytest.raises(ValueError, match="first_keyhole"):
        LocalSearchConfig(**(kwargs | changes))


def test_new_horizon_uses_full_namo_and_rejects_legacy_held_path():
    config = LocalSearchConfig(local_search="best_first", best_first_prior="uniform", planning_horizon="first_keyhole")
    assert config.as_planner_kwargs()["full_namo_planning_horizon"] == "first_keyhole"
    check_search_reaches_planner("full_namo", "primitive", config, exec_mode_named=True)
    with pytest.raises(ValueError, match="first_keyhole"):
        check_search_reaches_planner("full_namo", "primitive", config, held_boundary=True)


def verification_bridge(monkeypatch, tmp_path):
    bridge = bridge_mod.NAMOPlanBridge.__new__(bridge_mod.NAMOPlanBridge)
    bridge._scale_factor = 1.0
    bridge._robot_model = "car"
    bridge._generated_config_path = None
    bridge._debug_xml_path = str(tmp_path / "scene.xml")
    bridge._object_mapping = bridge_mod.ObjectMapping()
    bridge._object_mapping.add("obj_1", "obstacle_1")
    bridge._generate_xml = lambda *_: "<mujoco/>"
    bridge._write_xml = lambda *_: bridge._debug_xml_path
    steps = []
    env = SimpleNamespace(set_robot_goal=lambda *_: None,
                          is_robot_goal_reachable=lambda: False,
                          count_reachable_points=lambda _: (len(steps), 0),
                          step=lambda a: steps.append(a) or SimpleNamespace(done=True, info={}))
    bridge._build_rl_env_for_scene = lambda *_: env
    monkeypatch.setattr(bridge_mod, "resolve_namo_cpp_dir", lambda *_: tmp_path)
    monkeypatch.setattr(bridge_mod, "load_canonical_namo_rl", lambda *_:
                        (SimpleNamespace(Action=SimpleNamespace), None, None))
    return bridge, steps


def test_verification_cannot_execute_a_push_past_shared_allowance(monkeypatch, tmp_path):
    bridge, steps = verification_bridge(monkeypatch, tmp_path)
    budget = PushAttemptBudget(limit=2, used=1)
    result = bridge.verify_chain(_obs(), (40, 40),
                                [PushSubgoal("obj_1", 2, 2), PushSubgoal("obj_1", 4, 2)],
                                keyhole_target=TARGET, push_budget=budget)
    assert not result.success
    assert result.failure_reason == "simulation_budget_exhausted"
    assert len(steps) == result.sim_pushes_tried == 1
    assert budget.used == 2


def test_observation_only_opening_check_costs_no_push_budget(monkeypatch, tmp_path):
    bridge, steps = verification_bridge(monkeypatch, tmp_path)
    steps.append(object())
    budget = PushAttemptBudget(limit=2, used=2)
    result = bridge.verify_chain(_obs(), (40, 40), [], keyhole_target=TARGET, push_budget=budget)
    assert result.target_open_after
    assert result.failure_reason == "target_already_open_before_chain"
    assert result.sim_pushes_tried == 0
    assert budget.used == 2


def test_clearance_verification_maps_real_object_identity(monkeypatch, tmp_path):
    bridge, steps = verification_bridge(monkeypatch, tmp_path)
    env = bridge._build_rl_env_for_scene()
    queried = []
    env.object_occupies_point = lambda obj, point: queried.append((obj, point)) or True
    target = {"kind": "goal_clearance", "blocking_objects": ["obj_1"], "witness_xy": [0.3, 0.5]}
    result = bridge.verify_chain(_obs(), (40, 40), [], keyhole_target=target)
    assert result.failure_reason == "target_not_open_after_chain"
    assert queried == [("obstacle_1", (0.3, 0.5)), ("obstacle_1", (0.3, 0.5))]
    assert steps == []


def test_fresh_search_reports_call_delta_and_round_trips_target_identity(monkeypatch, tmp_path):
    bridge, _ = verification_bridge(monkeypatch, tmp_path)
    bridge._max_push_steps = None
    bridge._show_push_scores = False
    bridge._verbose = False
    calls = []
    budget = PushAttemptBudget(limit=9, used=7)

    def search(**kwargs):
        calls.append(kwargs)
        budget.consume_or_raise()
        budget.consume_or_raise()
        return SimpleNamespace(success=True, search_time_ms=10, error_message="",
                               actions=[SimpleNamespace(object_id="obstacle_1", edge_idx=4, depth=1)],
                               algorithm_stats={"keyhole_target": dict(TARGET, blocking_objects=["obstacle_1"]),
                                                "simulation_budget_used": budget.used})

    bridge._get_planning_service = lambda: SimpleNamespace(preload_goal_model=lambda *_a, **_k: None,
                                                          plan_from_xml=search)
    result = bridge.plan(_obs(), (40, 40), full_namo_active_keyhole=TARGET, push_budget=budget)
    assert result[0].object_id == "obj_1"
    assert calls[0]["full_namo_active_keyhole"]["blocking_objects"] == ["obstacle_1"]
    assert bridge.last_algorithm_stats["keyhole_target"] == TARGET
    assert bridge.last_algorithm_stats["simulations_used"] == 2
    assert bridge.last_algorithm_stats["trial_simulation_budget_used"] == 9


def run_script():
    import importlib
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
    return importlib.import_module("run_namo")


def test_cli_horizon_overrides_yaml_and_default_is_unchanged(tmp_path):
    config = tmp_path / "real.yaml"
    config.write_text("namo:\n  planning_horizon: first_keyhole\n")
    args = SimpleNamespace(config=str(config), planning_horizon=None)
    script = run_script()
    assert script.resolve_planning_horizon(args) == "first_keyhole"
    args.planning_horizon = "full_goal"
    assert script.resolve_planning_horizon(args) == "full_goal"
    args.config = None
    args.planning_horizon = None
    assert script.resolve_planning_horizon(args) == "full_goal"


def test_plan_only_does_not_report_local_opening_as_full_problem_success(tmp_path):
    import json
    from robot_control.diagnostics.recorder import DiagnosticsRecorder
    script = run_script()
    recorder = DiagnosticsRecorder(tmp_path / "trial", verbose=False)
    args = SimpleNamespace(exec_mode="search", planning_horizon="first_keyhole")
    payload = {"success": True, "outcome": "keyhole_ready", "search_stats": {"pushes_in_plan": 2}}
    script._write_plan_only_summary(args, recorder, payload, algorithm_stats={
        "planning_horizon": "first_keyhole", "plan_outcome": "keyhole_ready", "goal_reachable": False,
    })
    recorder.close()
    summary = json.loads((recorder.root / "summary.json").read_text())
    assert summary["outcome"] == "keyhole_ready"
    assert summary["goal_reachable"] is False
